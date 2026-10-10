#include "stm32f4xx.h"

void delay_ms(uint32_t ms)
{
    for(uint32_t i = 0; i < ms; i++)
    {
        for(volatile uint32_t j = 0; j < 16000; j++);
    }
}

int main(void)
{
    /* =========================================================
       1. Enable clocks
       ========================================================= */

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;


    /* =========================================================
       2. PA0 = TIM2_CH1
          PA1 = IN1
          PA2 = IN2
       ========================================================= */

    // PA0, PA1, PA2 -> Alternate Function / Output
    GPIOA->MODER &= ~(
        (3U << (0 * 2)) |
        (3U << (1 * 2)) |
        (3U << (2 * 2))
    );

    // PA0 = Alternate Function
    GPIOA->MODER |= (2U << (0 * 2));

    // PA1, PA2 = General purpose output
    GPIOA->MODER |= (1U << (1 * 2));
    GPIOA->MODER |= (1U << (2 * 2));


    /* =========================================================
       3. PA0 Alternate Function = AF1 = TIM2_CH1
       ========================================================= */

    GPIOA->AFR[0] &= ~(0xFU << (0 * 4));
    GPIOA->AFR[0] |=  (1U << (0 * 4));


    /* =========================================================
       4. Push-pull outputs
       ========================================================= */

    GPIOA->OTYPER &= ~(
        (1U << 1) |
        (1U << 2)
    );


    /* =========================================================
       5. TIM2 PWM configuration
       
       Timer clock = 84 MHz

       Prescaler = 83
       Timer frequency = 84 MHz / 84 = 1 MHz

       Therefore:
       1 timer count = 1 us

       ARR = 999
       Period = 1000 us = 1 ms
       PWM frequency = 1 kHz
       ========================================================= */

    TIM2->PSC = 83;
    TIM2->ARR = 999;

    // 50% duty cycle initially
    TIM2->CCR1 = 500;


    /* =========================================================
       6. PWM mode 1 on TIM2_CH1
       ========================================================= */

    TIM2->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM2->CCMR1 |= (6U << 4);

    // Enable preload
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;


    /* =========================================================
       7. Enable TIM2_CH1 output
       ========================================================= */

    TIM2->CCER |= TIM_CCER_CC1E;


    /* =========================================================
       8. Enable ARR preload
       ========================================================= */

    TIM2->CR1 |= TIM_CR1_ARPE;


    /* =========================================================
       9. Generate update event
       ========================================================= */

    TIM2->EGR |= TIM_EGR_UG;


    /* =========================================================
       10. Start timer
       ========================================================= */

    TIM2->CR1 |= TIM_CR1_CEN;


    /* =========================================================
       11. Motor direction
       ========================================================= */

    // Forward
    GPIOA->ODR |=  (1U << 1);   // IN1 = 1
    GPIOA->ODR &= ~(1U << 2);   // IN2 = 0


    while(1)
    {
        /*
         * 0%   = stopped
         * 25%  = slow
         * 50%  = medium
         * 75%  = fast
         * 100% = maximum
         */

        TIM2->CCR1 = 250;       // 25% speed
        delay_ms(1000);

        TIM2->CCR1 = 500;       // 50% speed
        delay_ms(1000);

        TIM2->CCR1 = 750;       // 75% speed
        delay_ms(1000);

        TIM2->CCR1 = 999;       // 100% speed
        delay_ms(1000);

        TIM2->CCR1 = 0;         // Stop
        delay_ms(1000);


        // Reverse direction
        GPIOA->ODR &= ~(1U << 1);   // IN1 = 0
        GPIOA->ODR |=  (1U << 2);   // IN2 = 1

        TIM2->CCR1 = 500;       // 50%
        delay_ms(3000);

        TIM2->CCR1 = 0;
        delay_ms(1000);


        // Forward again
        GPIOA->ODR |=  (1U << 1);
        GPIOA->ODR &= ~(1U << 2);

        TIM2->CCR1 = 500;
        delay_ms(3000);
    }
}