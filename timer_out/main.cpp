

#include <stdint.h>
#include "../include/cortexm4.h"
#include "../include/STM32F411.h"
void delay(uint32_t dly)
{
    while(dly--);
}
void initTimer()
{
	// Output a square wave on PA0 : TIM2_CH1
	RCC->AHB1ENR |= 1; // turn on GPIOA
	RCC->APB1ENR |= 1; // turn on Timer 2
	GPIOA->MODER &= ~1;
	GPIOA->MODER |= (1 << 1); // Set alternate mode for PA0
	GPIOA->AFRL &= ~(0xf); 
	GPIOA->AFRL |= (1); // Select alternative function 1
	TIM2->CR1 &= ~1; // disable Timer 2 before configuration
	TIM2->ARR = 200;
	TIM2->CCR1 = 100;
	TIM2->CCMR1_Output = BIT10 + BIT6 + BIT5 + BIT3 + BIT2; // PWM mode 1 for CH1, Fast mode enable, preload enable
	TIM2->CCER |= BIT0;    // Enable OC1 outputs.
    TIM2->CR1 |= BIT7; // Set the ARPE bit
    TIM2->EGR |= BIT0; // Force update of registers  
	TIM2->CR1 |= 1; // enable Timer 2 
}
int main()
{
	initTimer();
	
    while(1)
	{
    }
}
    
