// spi.h for stm32f030
// Author: Frank Duignan.  
// Updates posted on http://ioprog.com 

#include <stdint.h>
#include "../include/STM32F411.h"
#include "spi.h"
void spi::begin(void)
{
	int drain_count,drain;
	
	RCC->APB2ENR |= (1 << 12);		// turn on SPI1 	
/*
 * PA4  SPI1 NSS : AF5
 * PA5  SPI1 SCK : AF5
 * PA7  SPI1 MOSI : AF5
 * PA6  SPI1 MISO : AF5
 */
	
	// GPIOA bits 5 and 7 are used for SPI1 (Alternative functions 0)	
	RCC->AHB1ENR |= (1 << 0); 	// enable port A
    GPIOA->MODER &= ~( (1 << 14) + (1 << 12) + (1 << 10)+(1<<8)); // select Alternative function
    GPIOA->MODER |= ((1 << 15) + (1 << 13) + (1 << 11)+(1 << 9));  // for bits 4,5,6,7 
	GPIOA->AFRL &= 0x0000ffff;
    GPIOA->AFRL |= 0x55550000; // select Alt. Function 5	
	// set port bits up as high speed outputs
	GPIOA->OSPEEDR |= (1 << 15) + (1 << 14) + (1 << 13) + (1 << 12) +(1 << 11) + (1 << 10) + (1 << 9) + (1 << 8);
	
	
	// Now configure the SPI interface
	drain = SPI1->SR;				// dummy read of SR to clear MODF	
	// enable SSM, set SSI, enable SPI, PCLK/2, MSB First Master, Clock = 1 when idle
	// Will switch to software slave management
	SPI1->CR1 = (1 << 4) + (1 << 2) + (1 << 1) + (1 << 0); // update : set bit 5 to slow down the clock for debugging, software slave management, CPOL=1, CPH=1
	SPI1->CR2 = (1 << 2); 	// configure for 8 bit operation
   
    for (drain_count = 0; drain_count < 32; drain_count++)
		drain = transfer((uint8_t)0x00);
}

uint8_t spi::transfer(uint8_t data)
{	
    *((uint8_t*)&SPI1->DR) = data;        
	uint32_t Timeout=100;
	while (((SPI1->SR & (1 << 7))!=0)&&(Timeout--)); // Wait for Busy	
	return *((uint8_t*)&SPI1->DR);
}

uint16_t spi::transfer(uint16_t data)
{    
    SPI1->DR = data;        
	while (  (SPI1->SR & (1 << 1)) );
    return SPI1->DR;	
}
void spi::startTransaction(void)
{
	SPI1->CR1 |= (1 << 6); // Enable SPI (SPE = 1)
	//volatile unsigned Timeout = 100;    
	//while(Timeout--);
}
void spi::stopTransaction(void)
{
	volatile unsigned Timeout = 100;    
	while (((SPI1->SR & (1 << 0))!=0)&&(Timeout--)); // Wait for RXNE
	Timeout = 100;    
	while (((SPI1->SR & (1 << 1))==0)&&(Timeout--)); // Wait for TE
	Timeout = 100;    
	while (((SPI1->SR & (1 << 7))!=0)&&(Timeout--)); // Wait for Busy	
	SPI1->CR1 &= ~(1 << 6); // Disable SPI (SPE = 0)
	Timeout=10; // A short delay seems to be necessary here to allow the NSS line and the peripheral to recover
				// This corresonds to about 400ns @ 96MHz
	while (Timeout--);
}
