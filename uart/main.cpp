

#include <stdint.h>
#include "../include/cortexm4.h"
#include "../include/STM32F411.h"
#include "serial.h"
serial Serial;
void delay(uint32_t dly)
{
    while(dly--);
}

int main()
{	
	Serial.begin();
	enable_interrupts();
    while(1)
    {		    
		Serial.print("Hello\r\n");
		delay(10000);		
    }
}
    
