void init(void);
void Default_Handler(void);
int main(void);

// The following are 'declared' in the linker script
extern unsigned char  INIT_DATA_VALUES;
extern unsigned char  INIT_DATA_START;
extern unsigned char  INIT_DATA_END;
extern unsigned char  BSS_START;
extern unsigned char  BSS_END;
// the section "vectors" is placed at the beginning of flash 
// by the linker script
typedef void (*fptr)(void);
const fptr Vectors[] __attribute__((section(".vectors"))) ={
	(fptr)0x20020000, 	/* Top of stack (128k) */ 
	init,   		     /* Reset Handler */
	Default_Handler,	/* NMI */
	Default_Handler,	/* Hard Fault */
	Default_Handler,	/* MemManage */
	Default_Handler,	/* Bus Fault  */
	Default_Handler,	/* Usage Fault */
	Default_Handler,	/* Reserved */ 
	Default_Handler,	/* Reserved */
	Default_Handler,	/* Reserved */
	Default_Handler,	/* Reserved */
	Default_Handler,	/* SVCall */
	Default_Handler,	/* Debug monitor */
	Default_Handler,	/* Reserved */
	Default_Handler,	/* PendSV */
	Default_Handler,    /* SysTick */	
	/* External interrupt handlers follow */
	Default_Handler, 	/* 0: WWDG */
	Default_Handler, 	/* 1: EXTI16/PVD */
	Default_Handler, 	/* 2: EXTI21/TAMP_STAMP */
	Default_Handler, 	/* 3: EXTI22/RTC */
	Default_Handler, 	/* 4: Flash */
	Default_Handler, 	/* 5: RCC */
	Default_Handler, 	/* 6: EXTI Line 0 */
	Default_Handler, 	/* 7: EXTI Line 1 */
	Default_Handler, 	/* 8: EXTI Line 2 */
	Default_Handler, 	/* 9: EXTI Line 3 */
	Default_Handler, 	/* 10: EXTI Line 4 */
	Default_Handler, 	/* 11: DMA1 Stream 0 */
	Default_Handler, 	/* 12: DMA1 Stream 1 */
	Default_Handler, 	/* 13: DMA1 Stream 2 */
	Default_Handler, 	/* 14: DMA1 Stream 3 */
	Default_Handler, 	/* 15: DMA1 Stream 4 */
	Default_Handler, 	/* 16: DMA1 Stream 5 */
	Default_Handler, 	/* 17: DMA1 Stream 6 */
	Default_Handler, 	/* 18: ADC  */
	Default_Handler, 	/* 19: Reserved */
	Default_Handler, 	/* 20: Reserved */
	Default_Handler, 	/* 21: Reserved */
	Default_Handler, 	/* 22: Reserved */
	Default_Handler, 	/* 23: EXTI Line[9:5] interrupts */
	Default_Handler, 	/* 24: TIM1 Break interrupt and TIM9 global*/
	Default_Handler, 	/* 25: TIM1 Update interrupt and and TIM10 global*/
	Default_Handler, 	/* 26: TIM1 Trigger and Commutation interrupts */
	Default_Handler, 	/* 27: TIM1 Capture Compare interrupt */
	Default_Handler, 	/* 28: TIM2 global interrupt */
	Default_Handler, 	/* 29: TIM3 global interrupt */
	Default_Handler, 	/* 30: TIM4 global interrupt */
	Default_Handler, 	/* 31: I2C1 event interrupt */
	Default_Handler, 	/* 32: I2C1 error interrupt */
	Default_Handler, 	/* 33: I2C2 event interrupt */
	Default_Handler, 	/* 34: I2C2 error interrupt */
	Default_Handler, 	/* 35: SPI1 global interrupt */
	Default_Handler, 	/* 36: SPI2 global interrupt */
	Default_Handler, 	/* 37: USART1 global interrupt */
	Default_Handler, 	/* 38: USART2 global interrupt */
	Default_Handler, 	/* 39: Reserved */
	Default_Handler, 	/* 40: EXTI Line[15:10] interrupts */
	Default_Handler, 	/* 41: RTC Alarms through EXTI17 line interrupt */
	Default_Handler, 	/* 42: EXTI18/OTG_FS Interrupt */
	Default_Handler, 	/* 43: Reserved */
	Default_Handler, 	/* 44: Reserved */
	Default_Handler, 	/* 45: Reserved */
	Default_Handler, 	/* 46: Reserved */
	Default_Handler, 	/* 47: DMA1 Stream 7 */
	Default_Handler, 	/* 48: Reserved */
	Default_Handler, 	/* 49: SDIO global interrupt */
	Default_Handler, 	/* 50: TIM5 global interrupt */
	Default_Handler, 	/* 51: SPI3 global interrupt */
	Default_Handler, 	/* 52: Reserved */
	Default_Handler, 	/* 53: Reserved */
	Default_Handler, 	/* 54: Reserved */
	Default_Handler, 	/* 55: Reserved */
	Default_Handler, 	/* 56: DMA2 stream 0 global interrupt */
	Default_Handler, 	/* 57: DMA2 stream 1 global interrupt */
	Default_Handler, 	/* 58: DMA2 stream 2 global interrupt */
	Default_Handler, 	/* 59: DMA2 stream 3 global interrupt */
	Default_Handler, 	/* 60: DMA2 stream 4 global interrupt */
	Default_Handler, 	/* 61: Reserved */
	Default_Handler, 	/* 62: Reserved */
	Default_Handler, 	/* 63: Reserved */
	Default_Handler, 	/* 64: Reserved */
	Default_Handler, 	/* 65: Reserved  */
	Default_Handler, 	/* 66: Reserved */    
	Default_Handler, 	/* 67: OTG_FS global interrupt */
	Default_Handler, 	/* 68: DMA2 stream 5 global interrupt  */    
	Default_Handler, 	/* 69: DMA2 stream 6 global interrupt  */    
	Default_Handler, 	/* 70: DMA2 stream 7 global interrupt  */    
	Default_Handler, 	/* 71: USART6 global interrupt  */    
	Default_Handler, 	/* 72: I2C3 event interrupt  */    
	Default_Handler, 	/* 73: I2C3 error interrupt  */    
	Default_Handler, 	/* 74: Reserved */    
	Default_Handler, 	/* 75: Reserved */    
	Default_Handler, 	/* 76: Reserved */    
	Default_Handler, 	/* 77: Reserved */    
	Default_Handler, 	/* 78: Reserved */    
	Default_Handler, 	/* 79: Reserved */    
	Default_Handler, 	/* 80: Reserved */    
	Default_Handler, 	/* 81: FPU global interrupt  */    
	Default_Handler, 	/* 82: Reserved */        
	Default_Handler, 	/* 83: Reserved */        
	Default_Handler, 	/* 84: SPI4 globale interrupt */        
	Default_Handler 	/* 85: SPI5 global interrupt  */        
	
};
void init()
{
	// do global/static data initialization
	unsigned char *src;
	unsigned char *dest;
	unsigned len;
	src= &INIT_DATA_VALUES;
	dest= &INIT_DATA_START;
	len= &INIT_DATA_END-&INIT_DATA_START;
	while (len--)
		*dest++ = *src++;
	// zero out the uninitialized global/static variables
	dest = &BSS_START;
	len = &BSS_END - &BSS_START;
	while (len--)
		*dest++=0;
	main();
	while(1);
}

void Default_Handler()
{
	while(1);
}
