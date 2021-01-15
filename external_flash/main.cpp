/* The Black Pill STM32F411 board comes with pads for external SPI flash.  
 * This example interfaces with an ST M25P16 external flash
 * Wiring : 
 * /CS : PA4  (SPI1 NSS)
 * /DO : PB4  (SPI1 MISO) PA6!!
 * /WP : 3.3V
 * DI  : PA7  (SPI1 MOSI)
 * CLK : PA5  (SPI1 SCK)
 * /HOLD : 3.3V
 
 The "worst-case" maximum clock speed is 20MHz.  This is not 
 conveniently achieved with the divisors so will go with 12.5MHz for now
 (next speed up is 25MHz) 
 */


#include <stdint.h>
#include "../include/cortexm4.h"
#include "../include/STM32F411.h"
#include "serial.h"
#include "spi.h"
#include "spi_flash.h"
serial Serial;
spi SPI;
serial_flash Flash;

void delay(uint32_t dly)
{
    while(dly--);
}

int main()
{	
	uint8_t buffer[11];
	SPI.begin();
	Serial.begin();
	Flash.begin(SPI);
	Flash.deep_power_release();	
	Flash.read_id();	
	// uncomment any of the lines below to test.
	// Don't put write or erase functions in the main loop as they will
	// wear out the flash device
	// Flash.program_page(0,(uint8_t *)"Hello World",11); 
	// Flash.bulk_erase(); 
	// Flash.erase_sector(0);
	enable_interrupts();
    while(1)
    {	Flash.read_id();
		Flash.read_status();									
		Flash.read(0,buffer,11);
		for (int i=0;i<11;i++)
		{
			Serial.printHexByte(buffer[i]);
			Serial.print(" ");
		}
		Serial.print("\r\n");
		
		delay(5000);
    }
}
    
