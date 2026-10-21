#include "my_platform.h"
#include "vl53l0x_api.h"
#include "servo.h"
#include <stdint.h>

/* ---------------- Configuration ---------------- */

#define PWM_MAX                 999U
#define FULL_DUTY               (PWM_MAX + 1U)
#define HALF_DUTY               (FULL_DUTY / 2U)

#define KICK_MS                 100U
#define MOVE_MS                 200U
#define BRAKE_MS                150U
#define WAIT_MS                 5000U

#define SERVO_ID                1U
#define SERVO_UNITS_PER_REV     4096U
#define SERVO_MOVE_TIME_MS      0U
#define SERVO_SPEED             1000U
#define SERVO_SETTLE_MS         5000U
#define SERVO_SETTLE_MS_NEXT_POS  200U
#define SERVO_STEP_RAW          22U

/* USART2: PCLK1 = 42 MHz, baud = 115200 */
#define UART2_BRR_115200        0x16DU
#define Z_STEP_MM               20U

/* Motor direction pins: PB10, PB12, PB13, PB14, PB0, PB1, PB5, PB6 */
#define DIR_MASK \
    ((1U << 10) | (1U << 12) | \
     (1U << 13) | (1U << 14) | \
     (1U << 0)  | (1U << 1)  | \
     (1U << 5)  | (1U << 6))

/* Forward direction: IN2, IN4, IN2, IN4 high */
#define FWD_SET \
    ((1U << 12) | (1U << 14) | \
     (1U << 1)  | (1U << 6))

#define FWD_CLR \
    ((1U << 10) | (1U << 13) | \
     (1U << 0)  | (1U << 5))

/* ---------------- Servo library globals ---------------- */
/* Required by the existing servo.c implementation. */
volatile uint8_t response[20];
volatile uint8_t request[20];
volatile uint8_t response_received = 0U;
volatile uint8_t response_error = 0U;

/* Distance along the pipe. */
volatile uint32_t z_mm = 0U;

/* ---------------- LiDAR ---------------- */
VL53L0X_RangingMeasurementData_t ranging_data;
volatile uint16_t distance = 0U;

/* ---------------- USART2 transmit helpers ---------------- */
static void uart2_tx_char(char c)
{
    while ((USART2->SR & USART_SR_TXE) == 0U) {
        /* Wait for transmit data register. */
    }
    USART2->DR = (uint8_t)c;
}

static void uart2_tx_text(const char *text)
{
    while (*text != '\0') {
        uart2_tx_char(*text++);
    }
}

static void uart2_tx_uint32(uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U && count < sizeof(digits));

    while (count > 0U) {
        uart2_tx_char(digits[--count]);
    }
}

static void uart2_tx_int32(int32_t value)
{
    if (value < 0) {
        uart2_tx_char('-');
        /* Avoid signed overflow for INT32_MIN. */
        uart2_tx_uint32((uint32_t)(-(value + 1)) + 1U);
    } else {
        uart2_tx_uint32((uint32_t)value);
    }
}

static void send_data_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    (void)RCC->AHB1ENR;
    (void)RCC->APB1ENR;

    /* PA2 = USART2_TX, AF7. PA3 RX is not needed: no ACKs. */
    GPIOA->MODER &= ~(3U << (2U * 2U));
    GPIOA->MODER |=  (2U << (2U * 2U));
    GPIOA->AFR[0] &= ~(0xFU << (4U * 2U));
    GPIOA->AFR[0] |=  (7U << (4U * 2U));
    GPIOA->OTYPER &= ~(1U << 2U);
    GPIOA->OSPEEDR |= (2U << (2U * 2U));

		/* PA3 = USART2_RX, AF7 */
		GPIOA->MODER &= ~(3U << (3U * 2U));
		GPIOA->MODER |=  (2U << (3U * 2U));

		GPIOA->AFR[0] &= ~(0xFU << (3U * 4U));
		GPIOA->AFR[0] |=  (7U << (3U * 4U));
		GPIOA->OSPEEDR |= (2U << (2U * 2U));
		
    USART2->CR1 = 0U;
    USART2->CR2 = 0U;
    USART2->CR3 = 0U;
    USART2->BRR = UART2_BRR_115200;
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE | USART_CR1_RE;
	
	
		GPIOA->MODER &= ~(3U << 30);
		GPIOA->MODER |= (1U << 30);
		GPIOA->ODR |= (1U << 15);
		GPIOA->OTYPER &= ~(1U << 15U);

		/* Keep ESP32 out of reset: PA15 high. */
		GPIOA->BSRR = (1U << 15U);
}

/*
 * Wi-Fi TCP bridge/Python protocol:
 * DATA,sequence,distance_mm,angle_raw,z_mm\r\n
 * No application ACKs are sent or expected. The sequence number is useful
 * for identifying measurements, but the rover never waits for Python.
 */
static uint32_t measurement_sequence = 0U;

static void send_data(uint32_t z, uint16_t angle_raw, int32_t distance_mm)
{
    uart2_tx_text("DATA,");
    uart2_tx_uint32(++measurement_sequence);
    uart2_tx_char(',');
    uart2_tx_int32(distance_mm);
    uart2_tx_char(',');
    uart2_tx_uint32((uint32_t)angle_raw);
    uart2_tx_char(',');
    uart2_tx_uint32(z);
    uart2_tx_text("\r\n");
}


static void send_data_message(const char *message)
{

		char ackLine[8];
    uint32_t elapsed;
    uint8_t ackLength;
    uint8_t gotAck;
    char c;


    while (1) {
        /* Discard any stale received bytes before sending. */
        while ((USART2->SR & USART_SR_RXNE) != 0U) {
            (void)USART2->DR;
        }

        /* Transmit the message. */
        uart2_tx_text(message);
        uart2_tx_text("\r\n");

        ackLength = 0U;
        gotAck = 0U;

        /* Wait up to 1000 ms for an ACK line. */
        for (elapsed = 0U; elapsed < 2000U; elapsed++) {
            while ((USART2->SR & USART_SR_RXNE) != 0U) {
                c = (char)(USART2->DR & 0xFFU);

                if (c == '\n') {
                    ackLine[ackLength] = '\0';

                    if ((ackLength > 0U) &&
                        (ackLine[ackLength - 1U] == '\r')) {
                        ackLine[ackLength - 1U] = '\0';
                    }

                    if (ackLine[0] == 'A') {
                        gotAck = 1U;
                    }

                    ackLength = 0U;

                    if (gotAck != 0U) {
                        break;
                    }
                } else if (ackLength < sizeof(ackLine) - 1U) {
                    ackLine[ackLength++] = c;
                } else {
                    ackLength = 0U;
                }
            }

            if (gotAck != 0U) {
                break;
            }

            delay_ms(1U);
        }

        if (gotAck != 0U) {
            return;
        }

        /*
         * ACK timed out.
         * PA15 low = reset, assuming PA15 controls ESP32 EN
         * or an active-low reset circuit.
         */
        GPIOA->ODR &= ~(1U << 15);
        delay_ms(100U);

        GPIOA->ODR |= (1U << 15U);
        delay_ms(2000U); /* Allow ESP32 time to reboot. */

        /* Retry sending the message and wait for ACK again. */
    }
}

/* ---------------- Servo positioning ---------------- */
/* This uses raw servo units, matching the known-working reference code. */
static void servo_move_to_raw_position(uint16_t raw_position)
{
    servo_set_position(
        SERVO_ID,
        raw_position,
        SERVO_MOVE_TIME_MS,
        SERVO_SPEED
    );
}

/* ---------------- GPIO helpers ---------------- */
static void gpio_output(GPIO_TypeDef *port, uint8_t pin)
{
    port->MODER &= ~(3U << (pin * 2U));
    port->MODER |=  (1U << (pin * 2U));
    port->OTYPER &= ~(1U << pin);
    port->OSPEEDR |= (3U << (pin * 2U));
}

static void gpio_af(GPIO_TypeDef *port, uint8_t pin, uint8_t af)
{
    port->MODER &= ~(3U << (pin * 2U));
    port->MODER |=  (2U << (pin * 2U));
    port->OSPEEDR |= (3U << (pin * 2U));

    port->AFR[pin >> 3U] &= ~(0xFU << ((pin & 7U) * 4U));
    port->AFR[pin >> 3U] |= ((uint32_t)af << ((pin & 7U) * 4U));
}

/* ---------------- Motor PWM: TIM3 CH1 on PA6 ---------------- */
static void pwm_init(void)
{
    TIM3->PSC = (SystemCoreClock / 1000000U) - 1U;
    TIM3->ARR = PWM_MAX;
    TIM3->CCR1 = 0U;
    TIM3->CCMR1 = (6U << 4U) | TIM_CCMR1_OC1PE;
    TIM3->CCER = TIM_CCER_CC1E;
    TIM3->CR1 = TIM_CR1_ARPE;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 |= TIM_CR1_CEN;
}

static void set_speed(uint32_t duty)
{
    if (duty > FULL_DUTY) {
        duty = FULL_DUTY;
    }
    TIM3->CCR1 = duty;
}

/* ---------------- Motor control ---------------- */
static void motors_coast(void)
{
    set_speed(0U);
    GPIOB->BSRR = (DIR_MASK << 16U);
}

static void motors_go(void)
{
    GPIOB->BSRR = FWD_SET | (FWD_CLR << 16U);

    set_speed(FULL_DUTY);
    delay_ms(KICK_MS);

    if (MOVE_MS > KICK_MS) {
        set_speed(HALF_DUTY);
        delay_ms(MOVE_MS - KICK_MS);
    }

    set_speed(0U);
    GPIOB->BSRR = (DIR_MASK << 16U);
    delay_ms(BRAKE_MS);
    motors_coast();
}

static void motors_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                    RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    (void)RCC->AHB1ENR;
    (void)RCC->APB1ENR;
    gpio_output(GPIOB, 10U);
    gpio_output(GPIOB, 12U);
    gpio_output(GPIOB, 13U);
    gpio_output(GPIOB, 14U);
    gpio_output(GPIOB, 0U);
    gpio_output(GPIOB, 1U);
    gpio_output(GPIOB, 5U);
    gpio_output(GPIOB, 6U);
    GPIOB->BSRR = (DIR_MASK << 16U);

    /* PA6 = TIM3_CH1, AF2. */
    gpio_af(GPIOA, 6U, 2U);
    pwm_init();
    motors_coast();
}

/* ---------------- LED: PC13, active-low ---------------- */
static void led_init(void)
{
    gpio_output(GPIOC, 13U);
    GPIOC->BSRR = (1U << 13U); /* LED off. */
}

static void led_set(uint8_t on)
{
    if (on != 0U) {
        GPIOC->BSRR = (1U << (13U + 16U));
    } else {
        GPIOC->BSRR = (1U << 13U);
    }
}

/* ---------------- One complete 360-degree scan ---------------- */
static void perform_scan(void)
{
    uint16_t angle_raw;

    //send_data_message("SCAN_START");
		
		servo_set_position(
		SERVO_ID,
		0,
		SERVO_MOVE_TIME_MS,
		SERVO_SPEED
		);
	
		delay_ms(SERVO_SETTLE_MS);
	
		
    /* Raw servo positions 0..4095; increment matches reference code. */
    for (angle_raw = 0U; angle_raw < 4096; angle_raw+=11) {
        int32_t distance_mm = -1;

        servo_set_position(
        SERVO_ID,
        angle_raw,
        SERVO_MOVE_TIME_MS,
        SERVO_SPEED
				);
			
        delay_ms(SERVO_SETTLE_MS_NEXT_POS);

        VL53L0X_PerformSingleRangingMeasurement(&dev, &ranging_data);

        if (ranging_data.RangeStatus == 0U) {
            distance = ranging_data.RangeMilliMeter;
            distance_mm = (int32_t)distance;
        }
				

        /* Transmit immediately; never wait for a Wi-Fi/Python ACK. */
        send_data(z_mm, angle_raw, distance_mm);
    }
		
    delay_ms(SERVO_SETTLE_MS);
    //send_data_message("SCAN_END");
}

/* ---------------- Move forward one step ---------------- */
static void move_to_next_position(void)
{
    //send_data_message("Z_MOVE_START");

    motors_go();

    z_mm += Z_STEP_MM;

    //send_data_message("Z_MOVE_END");
    delay_ms(WAIT_MS);
	GPIOA->ODR &= ~(1U << 15);
	delay_ms(100U);

	GPIOA->ODR |= (1U << 15U);
	delay_ms(2000U);
}

/* ---------------- Main ---------------- */
int main(void)
{
    /* Existing platform initializes clocks, SysTick, I2C, LiDAR, USART1. */
    init_func();

    motors_init();
    led_init();
    send_data_init();

    /* Keep the existing servo driver's response globals and initialization. */
    servo_ping(SERVO_ID);

    led_set(1U);
    //send_data_message("MAPPING_READY");
    //send_data_message("MAPPING_ENABLED_AUTO");

    while (1) {
        perform_scan();
        move_to_next_position();
    }
}
