#include "my_platform.h"
#include "vl53l0x_api.h"
#include "servo.h"


/*
Servo: PA9
Lidar: PB8, PB9
*/

volatile uint8_t response[20];
volatile uint8_t request[20];

volatile uint8_t response_received = 0;
volatile uint8_t response_error = 0;

#include "my_platform.h"
#include "vl53l0x_api.h"
#include "servo.h"
#include <stdint.h>

/* Servo and scanning configuration */
#define SERVO_ID             1U
#define ANGLE_STEP_DEG       2U
#define SERVO_UNITS_PER_REV  4096U
#define SERVO_MOVE_TIME_MS   0U
#define SERVO_SPEED          1000U
#define SERVO_SETTLE_MS      10U

/* USART2 configuration: PCLK1 = 42 MHz, baud = 115200 */
#define UART2_BRR_115200     0x16DU

/* LiDAR measurement data */
VL53L0X_RangingMeasurementData_t ranging_data;
volatile uint16_t distance = 0U;

/* Transmit one character through USART2. */
static void uart2_tx_char(char c)
{
    while ((USART2->SR & USART_SR_TXE) == 0U) {
        /* Wait for transmit data register to become empty. */
    }

    USART2->DR = (uint8_t)c;
}


/* Transmit a null-terminated string. */
static void uart2_tx_text(const char *text)
{
    while (*text != '\0') {
        uart2_tx_char(*text);
        text++;
    }
}


/* Transmit an unsigned integer. */
static void uart2_tx_uint32(uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do {
        digits[count] = (char)('0' + (value % 10U));
        count++;
        value /= 10U;
    } while (value != 0U && count < sizeof(digits));

    while (count > 0U) {
        count--;
        uart2_tx_char(digits[count]);
    }
}


/* Transmit a signed integer. */
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


/* Initialize USART2 TX on PA2 at 115200 baud. */
static void send_data_init(void)
{
    /* Enable GPIOA and USART2 clocks. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* Configure PA2 as alternate function AF7 (USART2_TX). */
    GPIOA->MODER &= ~(3U << (2U * 2U));
    GPIOA->MODER |=  (2U << (2U * 2U));

    GPIOA->AFR[0] &= ~(0xFU << (4U * 2U));
    GPIOA->AFR[0] |=  (7U << (4U * 2U));

    GPIOA->OTYPER &= ~(1U << 2U);
    GPIOA->OSPEEDR |= (2U << (2U * 2U));

    /* Configure USART2: transmit only, 8-N-1. */
    USART2->CR1 = 0U;
    USART2->CR2 = 0U;
    USART2->CR3 = 0U;

    USART2->BRR = UART2_BRR_115200;
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
}


/*
 * Send one LiDAR record to the ESP32.
 * Format: z_mm,angle_deg,distance_mm
 */
static void send_data(
    uint32_t z_mm,
    uint16_t angle_deg,
    int32_t distance_mm
)
{
    uart2_tx_uint32(z_mm);
    uart2_tx_char(',');

    uart2_tx_uint32((uint32_t)angle_deg);
    uart2_tx_char(',');

    uart2_tx_int32(distance_mm);

    uart2_tx_text("\r\n");
}


/* Send a status message followed by a newline. */
static void send_data_message(const char *message)
{
    uart2_tx_text(message);
    uart2_tx_text("\r\n");
}


/* Convert scan angle to the servo's assumed position units. */
static uint16_t angle_to_servo_position(uint16_t angle_deg)
{
    return (uint16_t)(
        ((uint32_t)angle_deg * SERVO_UNITS_PER_REV) / 360U
    );
}

int main(void)
{
    uint16_t angle_deg;
    const uint32_t z_mm = 0U;

    /* Initialize clock, I2C, LiDAR and existing platform setup. */
    init_func();

    /* Initialize UART2 for STM32-to-ESP32 communication. */
    send_data_init();

    /* Verify servo communication. */
    servo_ping(SERVO_ID);

    send_data_message("MAPPING_READY");
    send_data_message("MAPPING_ENABLED_AUTO");

    while (1) {

        send_data_message("SCAN_START");

        /* Scan 0, 2, 4, ... 358 degrees. */
        for (angle_deg = 0U;
             angle_deg < 360U;
             angle_deg += ANGLE_STEP_DEG) {

            int32_t distance_mm = -1;
            uint16_t servo_pos;

            servo_pos = angle_to_servo_position(angle_deg);

            /* Command the servo to the requested position. */
            servo_set_position(
                SERVO_ID,
                servo_pos,
                SERVO_MOVE_TIME_MS,
                SERVO_SPEED
            );

            /* Allow the servo time to settle. */
            delay_ms(SERVO_SETTLE_MS);

            /* Obtain a single LiDAR measurement. */
            VL53L0X_PerformSingleRangingMeasurement(
                &dev,
                &ranging_data
            );

            /* Accept only measurements with valid range status. */
            if (ranging_data.RangeStatus == 0U) {
                distance = ranging_data.RangeMilliMeter;
                distance_mm = (int32_t)distance;
            }

            /* Send the real measurement to the ESP32. */
            send_data(z_mm, angle_deg, distance_mm);
        }

        send_data_message("SCAN_END");

        /* Return the servo to its zero position. */
        servo_set_position(
            SERVO_ID,
            0U,
            SERVO_MOVE_TIME_MS,
            SERVO_SPEED
        );

        delay_ms(SERVO_SETTLE_MS);
    }
}