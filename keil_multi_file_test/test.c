#include "test.h"

void my_init(void){
	RCC->AHB1ENR |= (1U << 2);
	GPIOC->MODER |= (1U << 26);
}
