// Blink the onboard LED (PC13) using a software delay

#include <stdint.h>
#include "../include/cortexm4.h"
#include "../include/STM32F411.h"
void delay(uint32_t dly)
{
    while(dly--);
}
int main()
{

	RCC->AHB1ENR |= (1 << 2); // Turn on Port C
	GPIOC->MODER |= (1 << 26); 
	GPIOC->MODER &= ~(1 << 27); // Make PC13 an output
    while(1)
    {
		GPIOC->ODR |= (1 << 13); // LED on
		delay(1000000);
		GPIOC->ODR &= ~(1 << 13); // LED off
		delay(1000000);        
    }
}
    
