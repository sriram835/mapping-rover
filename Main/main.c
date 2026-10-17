#include "my_platform.h"
#include "vl53l0x_api.h"
#include "servo.h"


volatile uint8_t response[20];
volatile uint8_t request[20];

volatile uint8_t response_received = 0;
volatile uint8_t response_error = 0;

VL53L0X_RangingMeasurementData_t ranging_data;
volatile uint16_t distance;

void uart2_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER &= ~(3U << (2U * 2U));
    GPIOA->MODER |=  (2U << (2U * 2U));

    GPIOA->AFR[0] &= ~(0xFU << (2U * 4U));
    GPIOA->AFR[0] |=  (7U << (2U * 4U));

    GPIOA->OTYPER &= ~(1U << 2U);
    GPIOA->OSPEEDR |= (2U << (2U * 2U));

    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = 0;

    USART2->BRR = 0x16D;

    USART2->CR1 |= USART_CR1_TE;
    USART2->CR1 |= USART_CR1_UE;
}

void uart2_send_char(char c)
{
    while (!(USART2->SR & USART_SR_TXE))
    {
    }

    USART2->DR = (uint8_t)c;
}

void uart2_send_text(const char *text)
{
    while (*text)
    {
        uart2_send_char(*text++);
    }
}

void uart2_send_distance(uint16_t mm)
{
    char digits[5];
    uint8_t i = 0;

    do
    {
        digits[i++] = '0' + (mm % 10U);
        mm /= 10U;
    } while (mm > 0U && i < sizeof(digits));

    while (i > 0U)
    {
        uart2_send_char(digits[--i]);
    }

    uart2_send_text(" mm\r\n");
}


int main(void)
{
    init_func();
    uart2_init();

    while (1)
    {
        VL53L0X_PerformSingleRangingMeasurement(
            &dev,
            &ranging_data
        );

        /* VL53L0X RangeStatus 4 means out of range */
        if (ranging_data.RangeStatus != 4)
        {
            distance = ranging_data.RangeMilliMeter;
            uart2_send_distance(distance);
        }
        else
        {
            uart2_send_text("Out of range\r\n");
        }

        delay_ms(10);
    }
}

/*
int main(void){
	init_func();
	// servo_ping(1);
	uint8_t servo_id = 1;
	uint16_t servo_pos = 0;
	uint16_t servo_time = 0;
	uint16_t servo_speed = 1000;
	
	
	
	
	while(1){
		        VL53L0X_PerformSingleRangingMeasurement(
            &dev,
            &ranging_data
        );
				distance = ranging_data.RangeMilliMeter;
				delay_ms(10);
				// servo_set_position(servo_id,servo_pos,servo_time,servo_speed);
				// servo_pos+=22;

	}
	
	
}

*/