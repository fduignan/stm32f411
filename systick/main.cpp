

#include <stdint.h>
#include "../include/cortexm4.h"
#include "../include/STM32F411.h"
void delay(uint32_t dly)
{
    while(dly--);
}
void initSystick()
{
    
    STK->CTRL |= ( 7 ); // enable systick, source = cpu clock, enable interrupt
// SysTick clock source = 100MHz.  Divide this down to create 1 millisecond period
    STK->LOAD = 100000-1;   
    STK->VAL = 10; // don't want long wait for counter to count down from initial high unknown value
}
void Systick_Handler()
{ 
    static uint32_t milliseconds = 0;
    milliseconds++;
    if (milliseconds == 1000)
    {
        milliseconds = 0;
         
    }
	GPIOC->ODR ^= (1 << 13); // toggle a port bit so sampling rate can be measured with an oscilloscope    
}
int main()
{

	RCC->AHB1ENR |= (1 << 2); // Turn on Port C
	GPIOC->MODER |= (1 << 26); 
	GPIOC->MODER &= ~(1 << 27); // Make PC13 an output
	initSystick();
	enable_interrupts();
	// Want to confirm the HSI clock speed so route it out on MCO_1 (PA8, AF0)
	RCC->AHB1ENR |= 1; // ensure GPIOA is on
	GPIOA->MODER &= ~(0x30000);
	GPIOA->MODER |= (1 << 17);  // Select alternate mode for GPIOA_8
	GPIOA->AFRH &= ~(0xf); // select AF0
	GPIOA->OSPEEDR |= ( (1 << 17) + (1 << 16) ); // make the pin high speed
	RCC->CFGR &= ~( ( 1 << 22) + (1 << 21) );  // route HSI to MCO_1
	RCC->CFGR &= ~(1 << 24); 
	RCC->CFGR |= ( (1 << 26) + (1 << 25) ); // divide MCO_1 out by 4 to facilitate reading
	// Measured MCO_1 out to be exactly 4MHz so something doesn't add up here - check the timer and PLL settings
    while(1)
    {		    
    }
}
    
