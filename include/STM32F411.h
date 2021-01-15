/****************************************************************************************************//**
 *
 *  CMSIS-SVD SVD Consistency Checker / Header File Generator V3.2.17
 *  Copyright (C) 2010 - 2016 ARM Ltd and ARM Germany GmbH. All rights reserved.
 *
 * @brief    CMSIS HeaderFile
 *
 * @date     12. January 2021
 *
 * @note     Generated with SVDConv V3.2.17 on Tuesday, 12.01.2021 15:43:27
 *
 *           from CMSIS SVD File 'STM32F411.svd',
 *           created on Thursday, 12.10.2017 13:39:30, last modified on Thursday, 12.10.2017 13:39:30
 *
 *******************************************************************************************************/

/*
  
 */



/** @addtogroup 
  * @{
  */


/** @addtogroup STM32F411
  * @{
  */


#ifndef STM32F411_H
#define STM32F411_H

#ifdef __cplusplus
extern "C" {
#endif


/** @addtogroup Configuration_of_CMSIS
  * @{
  */



/* =========================================================================================================================== */
/* ================                                Interrupt Number Definition                                ================ */
/* =========================================================================================================================== */

typedef enum {
/* =======================================  ARM Cortex-M7 Specific Interrupt Numbers  ======================================== */
  Reset_IRQn                = -15,              /*!< -15  Reset Vector, invoked on Power up and warm reset                     */
  NonMaskableInt_IRQn       = -14,              /*!< -14  Non maskable Interrupt, cannot be stopped or preempted               */
  HardFault_IRQn            = -13,              /*!< -13  Hard Fault, all classes of Fault                                     */
  MemoryManagement_IRQn     = -12,              /*!< -12  Memory Management, MPU mismatch, including Access Violation
                                                      and No Match                                                             */
  BusFault_IRQn             = -11,              /*!< -11  Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory
                                                      related Fault                                                            */
  UsageFault_IRQn           = -10,              /*!< -10  Usage Fault, i.e. Undef Instruction, Illegal State Transition        */
  SVCall_IRQn               =  -5,              /*!< -5 System Service Call via SVC instruction                                */
  DebugMonitor_IRQn         =  -4,              /*!< -4 Debug Monitor                                                          */
  PendSV_IRQn               =  -2,              /*!< -2 Pendable request for system service                                    */
  SysTick_IRQn              =  -1,              /*!< -1 System Tick Timer                                                      */
/* =========================================  STM32F411 Specific Interrupt Numbers  ========================================== */
  PVD_IRQn                  =   1,              /*!< 1  PVD through EXTI line detection interrupt                              */
  TAMP_STAMP_IRQn           =   2,              /*!< 2  Tamper and TimeStamp interrupts through the EXTI line                  */
  RTC_WKUP_IRQn             =   3,              /*!< 3  RTC Wakeup interrupt through the EXTI line                             */
  FLASH_IRQn                =   4,              /*!< 4  FLASH global interrupt                                                 */
  RCC_IRQn                  =   5,              /*!< 5  RCC global interrupt                                                   */
  EXTI0_IRQn                =   6,              /*!< 6  EXTI Line0 interrupt                                                   */
  EXTI1_IRQn                =   7,              /*!< 7  EXTI Line1 interrupt                                                   */
  EXTI2_IRQn                =   8,              /*!< 8  EXTI Line2 interrupt                                                   */
  EXTI3_IRQn                =   9,              /*!< 9  EXTI Line3 interrupt                                                   */
  EXTI4_IRQn                =  10,              /*!< 10 EXTI Line4 interrupt                                                   */
  ADC_IRQn                  =  18,              /*!< 18 ADC1 global interrupt                                                  */
  EXTI9_5_IRQn              =  23,              /*!< 23 EXTI Line[9:5] interrupts                                              */
  TIM1_BRK_TIM9_IRQn        =  24,              /*!< 24 TIM1 Break interrupt and TIM9 global interrupt                         */
  TIM1_UP_TIM10_IRQn        =  25,              /*!< 25 TIM1 Update interrupt and TIM10 global interrupt                       */
  TIM1_TRG_COM_TIM11_IRQn   =  26,              /*!< 26 TIM1 Trigger and Commutation interrupts and TIM11 global
                                                      interrupt                                                                */
  TIM1_CC_IRQn              =  27,              /*!< 27 TIM1 Capture Compare interrupt                                         */
  TIM2_IRQn                 =  28,              /*!< 28 TIM2 global interrupt                                                  */
  TIM3_IRQn                 =  29,              /*!< 29 TIM3 global interrupt                                                  */
  I2C1_EV_IRQn              =  31,              /*!< 31 I2C1 event interrupt                                                   */
  I2C1_ER_IRQn              =  32,              /*!< 32 I2C1 error interrupt                                                   */
  I2C2_EV_IRQn              =  33,              /*!< 33 I2C2 event interrupt                                                   */
  I2C2_ER_IRQn              =  34,              /*!< 34 I2C2 error interrupt                                                   */
  SPI1_IRQn                 =  35,              /*!< 35 SPI1 global interrupt                                                  */
  SPI2_IRQn                 =  36,              /*!< 36 SPI2 global interrupt                                                  */
  EXTI15_10_IRQn            =  40,              /*!< 40 EXTI Line[15:10] interrupts                                            */
  RTC_Alarm_IRQn            =  41,              /*!< 41 RTC Alarms (A and B) through EXTI line interrupt                       */
  OTG_FS_WKUP_IRQn          =  42,              /*!< 42 USB On-The-Go FS Wakeup through EXTI line interrupt                    */
  SDIO_IRQn                 =  49,              /*!< 49 SDIO global interrupt                                                  */
  SPI3_IRQn                 =  51,              /*!< 51 SPI3 global interrupt                                                  */
  OTG_FS_IRQn               =  67,              /*!< 67 USB On The Go FS global interrupt                                      */
  I2C3_EV_IRQn              =  72,              /*!< 72 I2C3 event interrupt                                                   */
  I2C3_ER_IRQn              =  73,              /*!< 73 I2C3 error interrupt                                                   */
  FPU_IRQn                  =  81,              /*!< 81 FPU interrupt                                                          */
  SPI4_IRQn                 =  84               /*!< 84 SPI4 global interrupt                                                  */
} IRQn_Type;



/* =========================================================================================================================== */
/* ================                           Processor and Core Peripheral Section                           ================ */
/* =========================================================================================================================== */

/* ===========================  Configuration of the ARM Cortex-M7 Processor and Core Peripherals  =========================== */
#define __CM4_REV                 0x0100        /*!< CM4 Core Revision                                                         */
#define __MPU_PRESENT                  0        /*!< MPU present or not                                                        */
#define __NVIC_PRIO_BITS               3        /*!< Number of Bits used for Priority Levels                                   */
#define __Vendor_SysTickConfig         0        /*!< Set to 1 if different SysTick Config is used                              */
#define __FPU_PRESENT                  0        /*!< FPU present or not                                                        */


/** @} */ /* End of group Configuration_of_CMSIS */

#include "core_CM4.h"                           /*!< ARM Cortex-M7 processor and core peripherals                              */
#include "system_STM32F411.h"                   /*!< STM32F411 System                                                          */


/* ========================================  Start of section using anonymous unions  ======================================== */
#if defined(__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined(__ICCARM__)
  #pragma language=extended
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning 586
#else
  #warning Not supported compiler type
#endif


/* =========================================================================================================================== */
/* ================                            Device Specific Peripheral Section                             ================ */
/* =========================================================================================================================== */


/** @addtogroup Device_Peripheral_peripherals
  * @{
  */



/* =========================================================================================================================== */
/* ================                                        ADC_Common                                         ================ */
/* =========================================================================================================================== */


/**
  * @brief ADC common registers (ADC_Common)
  */

typedef struct {                                /*!< (@ 0x40012300) ADC_Common Structure                                       */
  __I  uint32_t   CSR;                          /*!< (@ 0x00000000) ADC Common status register                                 */
  __IO uint32_t   CCR;                          /*!< (@ 0x00000004) ADC common control register                                */
} ADC_Common_Type;                              /*!< Size = 8 (0x8)                                                            */



/* =========================================================================================================================== */
/* ================                                           ADC1                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Analog-to-digital converter (ADC1)
  */

typedef struct {                                /*!< (@ 0x40012000) ADC1 Structure                                             */
  __IO uint32_t   SR;                           /*!< (@ 0x00000000) status register                                            */
  __IO uint32_t   CR1;                          /*!< (@ 0x00000004) control register 1                                         */
  __IO uint32_t   CR2;                          /*!< (@ 0x00000008) control register 2                                         */
  __IO uint32_t   SMPR1;                        /*!< (@ 0x0000000C) sample time register 1                                     */
  __IO uint32_t   SMPR2;                        /*!< (@ 0x00000010) sample time register 2                                     */
  __IO uint32_t   JOFR1;                        /*!< (@ 0x00000014) injected channel data offset register x                    */
  __IO uint32_t   JOFR2;                        /*!< (@ 0x00000018) injected channel data offset register x                    */
  __IO uint32_t   JOFR3;                        /*!< (@ 0x0000001C) injected channel data offset register x                    */
  __IO uint32_t   JOFR4;                        /*!< (@ 0x00000020) injected channel data offset register x                    */
  __IO uint32_t   HTR;                          /*!< (@ 0x00000024) watchdog higher threshold register                         */
  __IO uint32_t   LTR;                          /*!< (@ 0x00000028) watchdog lower threshold register                          */
  __IO uint32_t   SQR1;                         /*!< (@ 0x0000002C) regular sequence register 1                                */
  __IO uint32_t   SQR2;                         /*!< (@ 0x00000030) regular sequence register 2                                */
  __IO uint32_t   SQR3;                         /*!< (@ 0x00000034) regular sequence register 3                                */
  __IO uint32_t   JSQR;                         /*!< (@ 0x00000038) injected sequence register                                 */
  __I  uint32_t   JDR1;                         /*!< (@ 0x0000003C) injected data register x                                   */
  __I  uint32_t   JDR2;                         /*!< (@ 0x00000040) injected data register x                                   */
  __I  uint32_t   JDR3;                         /*!< (@ 0x00000044) injected data register x                                   */
  __I  uint32_t   JDR4;                         /*!< (@ 0x00000048) injected data register x                                   */
  __I  uint32_t   DR;                           /*!< (@ 0x0000004C) regular data register                                      */
} ADC1_Type;                                    /*!< Size = 80 (0x50)                                                          */



/* =========================================================================================================================== */
/* ================                                            CRC                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Cryptographic processor (CRC)
  */

typedef struct {                                /*!< (@ 0x40023000) CRC Structure                                              */
  __IO uint32_t   DR;                           /*!< (@ 0x00000000) Data register                                              */
  __IO uint32_t   IDR;                          /*!< (@ 0x00000004) Independent Data register                                  */
  __O  uint32_t   CR;                           /*!< (@ 0x00000008) Control register                                           */
} CRC_Type;                                     /*!< Size = 12 (0xc)                                                           */



/* =========================================================================================================================== */
/* ================                                            DBG                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Debug support (DBG)
  */

typedef struct {                                /*!< (@ 0xE0042000) DBG Structure                                              */
  __I  uint32_t   DBGMCU_IDCODE;                /*!< (@ 0x00000000) IDCODE                                                     */
  __IO uint32_t   DBGMCU_CR;                    /*!< (@ 0x00000004) Control Register                                           */
  __IO uint32_t   DBGMCU_APB1_FZ;               /*!< (@ 0x00000008) Debug MCU APB1 Freeze registe                              */
  __IO uint32_t   DBGMCU_APB2_FZ;               /*!< (@ 0x0000000C) Debug MCU APB2 Freeze registe                              */
} DBG_Type;                                     /*!< Size = 16 (0x10)                                                          */



/* =========================================================================================================================== */
/* ================                                           EXTI                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief External interrupt/event  controller (EXTI)
  */

typedef struct {                                /*!< (@ 0x40013C00) EXTI Structure                                             */
  __IO uint32_t   IMR;                          /*!< (@ 0x00000000) Interrupt mask register (EXTI_IMR)                         */
  __IO uint32_t   EMR;                          /*!< (@ 0x00000004) Event mask register (EXTI_EMR)                             */
  __IO uint32_t   RTSR;                         /*!< (@ 0x00000008) Rising Trigger selection register (EXTI_RTSR)              */
  __IO uint32_t   FTSR;                         /*!< (@ 0x0000000C) Falling Trigger selection register (EXTI_FTSR)             */
  __IO uint32_t   SWIER;                        /*!< (@ 0x00000010) Software interrupt event register (EXTI_SWIER)             */
  __IO uint32_t   PR;                           /*!< (@ 0x00000014) Pending register (EXTI_PR)                                 */
} EXTI_Type;                                    /*!< Size = 24 (0x18)                                                          */



/* =========================================================================================================================== */
/* ================                                           FLASH                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief FLASH (FLASH)
  */

typedef struct {                                /*!< (@ 0x40023C00) FLASH Structure                                            */
  __IO uint32_t   ACR;                          /*!< (@ 0x00000000) Flash access control register                              */
  __O  uint32_t   KEYR;                         /*!< (@ 0x00000004) Flash key register                                         */
  __O  uint32_t   OPTKEYR;                      /*!< (@ 0x00000008) Flash option key register                                  */
  __IO uint32_t   SR;                           /*!< (@ 0x0000000C) Status register                                            */
  __IO uint32_t   CR;                           /*!< (@ 0x00000010) Control register                                           */
  __IO uint32_t   OPTCR;                        /*!< (@ 0x00000014) Flash option control register                              */
} FLASH_Type;                                   /*!< Size = 24 (0x18)                                                          */



/* =========================================================================================================================== */
/* ================                                           IWDG                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Independent watchdog (IWDG)
  */

typedef struct {                                /*!< (@ 0x40003000) IWDG Structure                                             */
  __O  uint32_t   KR;                           /*!< (@ 0x00000000) Key register                                               */
  __IO uint32_t   PR;                           /*!< (@ 0x00000004) Prescaler register                                         */
  __IO uint32_t   RLR;                          /*!< (@ 0x00000008) Reload register                                            */
  __I  uint32_t   SR;                           /*!< (@ 0x0000000C) Status register                                            */
} IWDG_Type;                                    /*!< Size = 16 (0x10)                                                          */



/* =========================================================================================================================== */
/* ================                                       OTG_FS_DEVICE                                       ================ */
/* =========================================================================================================================== */


/**
  * @brief USB on the go full speed (OTG_FS_DEVICE)
  */

typedef struct {                                /*!< (@ 0x50000800) OTG_FS_DEVICE Structure                                    */
  __IO uint32_t   FS_DCFG;                      /*!< (@ 0x00000000) OTG_FS device configuration register (OTG_FS_DCFG)         */
  __IO uint32_t   FS_DCTL;                      /*!< (@ 0x00000004) OTG_FS device control register (OTG_FS_DCTL)               */
  __I  uint32_t   FS_DSTS;                      /*!< (@ 0x00000008) OTG_FS device status register (OTG_FS_DSTS)                */
  __I  uint32_t   RESERVED;
  __IO uint32_t   FS_DIEPMSK;                   /*!< (@ 0x00000010) OTG_FS device IN endpoint common interrupt mask
                                                                    register (OTG_FS_DIEPMSK)                                  */
  __IO uint32_t   FS_DOEPMSK;                   /*!< (@ 0x00000014) OTG_FS device OUT endpoint common interrupt mask
                                                                    register (OTG_FS_DOEPMSK)                                  */
  __I  uint32_t   FS_DAINT;                     /*!< (@ 0x00000018) OTG_FS device all endpoints interrupt register
                                                                    (OTG_FS_DAINT)                                             */
  __IO uint32_t   FS_DAINTMSK;                  /*!< (@ 0x0000001C) OTG_FS all endpoints interrupt mask register
                                                                    (OTG_FS_DAINTMSK)                                          */
  __I  uint32_t   RESERVED1[2];
  __IO uint32_t   DVBUSDIS;                     /*!< (@ 0x00000028) OTG_FS device VBUS discharge time register                 */
  __IO uint32_t   DVBUSPULSE;                   /*!< (@ 0x0000002C) OTG_FS device VBUS pulsing time register                   */
  __I  uint32_t   RESERVED2;
  __IO uint32_t   DIEPEMPMSK;                   /*!< (@ 0x00000034) OTG_FS device IN endpoint FIFO empty interrupt
                                                                    mask register                                              */
  __I  uint32_t   RESERVED3[50];
  __IO uint32_t   FS_DIEPCTL0;                  /*!< (@ 0x00000100) OTG_FS device control IN endpoint 0 control register
                                                                    (OTG_FS_DIEPCTL0)                                          */
  __I  uint32_t   RESERVED4;
  __IO uint32_t   DIEPINT0;                     /*!< (@ 0x00000108) device endpoint-x interrupt register                       */
  __I  uint32_t   RESERVED5;
  __IO uint32_t   DIEPTSIZ0;                    /*!< (@ 0x00000110) device endpoint-0 transfer size register                   */
  __I  uint32_t   RESERVED6;
  __I  uint32_t   DTXFSTS0;                     /*!< (@ 0x00000118) OTG_FS device IN endpoint transmit FIFO status
                                                                    register                                                   */
  __I  uint32_t   RESERVED7;
  __IO uint32_t   DIEPCTL1;                     /*!< (@ 0x00000120) OTG device endpoint-1 control register                     */
  __I  uint32_t   RESERVED8;
  __IO uint32_t   DIEPINT1;                     /*!< (@ 0x00000128) device endpoint-1 interrupt register                       */
  __I  uint32_t   RESERVED9;
  __IO uint32_t   DIEPTSIZ1;                    /*!< (@ 0x00000130) device endpoint-1 transfer size register                   */
  __I  uint32_t   RESERVED10;
  __I  uint32_t   DTXFSTS1;                     /*!< (@ 0x00000138) OTG_FS device IN endpoint transmit FIFO status
                                                                    register                                                   */
  __I  uint32_t   RESERVED11;
  __IO uint32_t   DIEPCTL2;                     /*!< (@ 0x00000140) OTG device endpoint-2 control register                     */
  __I  uint32_t   RESERVED12;
  __IO uint32_t   DIEPINT2;                     /*!< (@ 0x00000148) device endpoint-2 interrupt register                       */
  __I  uint32_t   RESERVED13;
  __IO uint32_t   DIEPTSIZ2;                    /*!< (@ 0x00000150) device endpoint-2 transfer size register                   */
  __I  uint32_t   RESERVED14;
  __I  uint32_t   DTXFSTS2;                     /*!< (@ 0x00000158) OTG_FS device IN endpoint transmit FIFO status
                                                                    register                                                   */
  __I  uint32_t   RESERVED15;
  __IO uint32_t   DIEPCTL3;                     /*!< (@ 0x00000160) OTG device endpoint-3 control register                     */
  __I  uint32_t   RESERVED16;
  __IO uint32_t   DIEPINT3;                     /*!< (@ 0x00000168) device endpoint-3 interrupt register                       */
  __I  uint32_t   RESERVED17;
  __IO uint32_t   DIEPTSIZ3;                    /*!< (@ 0x00000170) device endpoint-3 transfer size register                   */
  __I  uint32_t   RESERVED18;
  __I  uint32_t   DTXFSTS3;                     /*!< (@ 0x00000178) OTG_FS device IN endpoint transmit FIFO status
                                                                    register                                                   */
  __I  uint32_t   RESERVED19[97];
  __IO uint32_t   DOEPCTL0;                     /*!< (@ 0x00000300) device endpoint-0 control register                         */
  __I  uint32_t   RESERVED20;
  __IO uint32_t   DOEPINT0;                     /*!< (@ 0x00000308) device endpoint-0 interrupt register                       */
  __I  uint32_t   RESERVED21;
  __IO uint32_t   DOEPTSIZ0;                    /*!< (@ 0x00000310) device OUT endpoint-0 transfer size register               */
  __I  uint32_t   RESERVED22[3];
  __IO uint32_t   DOEPCTL1;                     /*!< (@ 0x00000320) device endpoint-1 control register                         */
  __I  uint32_t   RESERVED23;
  __IO uint32_t   DOEPINT1;                     /*!< (@ 0x00000328) device endpoint-1 interrupt register                       */
  __I  uint32_t   RESERVED24;
  __IO uint32_t   DOEPTSIZ1;                    /*!< (@ 0x00000330) device OUT endpoint-1 transfer size register               */
  __I  uint32_t   RESERVED25[3];
  __IO uint32_t   DOEPCTL2;                     /*!< (@ 0x00000340) device endpoint-2 control register                         */
  __I  uint32_t   RESERVED26;
  __IO uint32_t   DOEPINT2;                     /*!< (@ 0x00000348) device endpoint-2 interrupt register                       */
  __I  uint32_t   RESERVED27;
  __IO uint32_t   DOEPTSIZ2;                    /*!< (@ 0x00000350) device OUT endpoint-2 transfer size register               */
  __I  uint32_t   RESERVED28[3];
  __IO uint32_t   DOEPCTL3;                     /*!< (@ 0x00000360) device endpoint-3 control register                         */
  __I  uint32_t   RESERVED29;
  __IO uint32_t   DOEPINT3;                     /*!< (@ 0x00000368) device endpoint-3 interrupt register                       */
  __I  uint32_t   RESERVED30;
  __IO uint32_t   DOEPTSIZ3;                    /*!< (@ 0x00000370) device OUT endpoint-3 transfer size register               */
} OTG_FS_DEVICE_Type;                           /*!< Size = 884 (0x374)                                                        */



/* =========================================================================================================================== */
/* ================                                       OTG_FS_GLOBAL                                       ================ */
/* =========================================================================================================================== */


/**
  * @brief USB on the go full speed (OTG_FS_GLOBAL)
  */

typedef struct {                                /*!< (@ 0x50000000) OTG_FS_GLOBAL Structure                                    */
  __IO uint32_t   FS_GOTGCTL;                   /*!< (@ 0x00000000) OTG_FS control and status register (OTG_FS_GOTGCTL)        */
  __IO uint32_t   FS_GOTGINT;                   /*!< (@ 0x00000004) OTG_FS interrupt register (OTG_FS_GOTGINT)                 */
  __IO uint32_t   FS_GAHBCFG;                   /*!< (@ 0x00000008) OTG_FS AHB configuration register (OTG_FS_GAHBCFG)         */
  __IO uint32_t   FS_GUSBCFG;                   /*!< (@ 0x0000000C) OTG_FS USB configuration register (OTG_FS_GUSBCFG)         */
  __IO uint32_t   FS_GRSTCTL;                   /*!< (@ 0x00000010) OTG_FS reset register (OTG_FS_GRSTCTL)                     */
  __IO uint32_t   FS_GINTSTS;                   /*!< (@ 0x00000014) OTG_FS core interrupt register (OTG_FS_GINTSTS)            */
  __IO uint32_t   FS_GINTMSK;                   /*!< (@ 0x00000018) OTG_FS interrupt mask register (OTG_FS_GINTMSK)            */
  
  union {
    __I  uint32_t FS_GRXSTSR_Device;            /*!< (@ 0x0000001C) OTG_FS Receive status debug read(Device mode)              */
    __I  uint32_t FS_GRXSTSR_Host;              /*!< (@ 0x0000001C) OTG_FS Receive status debug read(Host mode)                */
  };
  __I  uint32_t   RESERVED;
  __IO uint32_t   FS_GRXFSIZ;                   /*!< (@ 0x00000024) OTG_FS Receive FIFO size register (OTG_FS_GRXFSIZ)         */
  
  union {
    __IO uint32_t FS_GNPTXFSIZ_Device;          /*!< (@ 0x00000028) OTG_FS non-periodic transmit FIFO size register
                                                                    (Device mode)                                              */
    __IO uint32_t FS_GNPTXFSIZ_Host;            /*!< (@ 0x00000028) OTG_FS non-periodic transmit FIFO size register
                                                                    (Host mode)                                                */
  };
  __I  uint32_t   FS_GNPTXSTS;                  /*!< (@ 0x0000002C) OTG_FS non-periodic transmit FIFO/queue status
                                                                    register (OTG_FS_GNPTXSTS)                                 */
  __I  uint32_t   RESERVED1[2];
  __IO uint32_t   FS_GCCFG;                     /*!< (@ 0x00000038) OTG_FS general core configuration register (OTG_FS_GCCFG)  */
  __IO uint32_t   FS_CID;                       /*!< (@ 0x0000003C) core ID register                                           */
  __I  uint32_t   RESERVED2[48];
  __IO uint32_t   FS_HPTXFSIZ;                  /*!< (@ 0x00000100) OTG_FS Host periodic transmit FIFO size register
                                                                    (OTG_FS_HPTXFSIZ)                                          */
  __IO uint32_t   FS_DIEPTXF1;                  /*!< (@ 0x00000104) OTG_FS device IN endpoint transmit FIFO size
                                                                    register (OTG_FS_DIEPTXF2)                                 */
  __IO uint32_t   FS_DIEPTXF2;                  /*!< (@ 0x00000108) OTG_FS device IN endpoint transmit FIFO size
                                                                    register (OTG_FS_DIEPTXF3)                                 */
  __IO uint32_t   FS_DIEPTXF3;                  /*!< (@ 0x0000010C) OTG_FS device IN endpoint transmit FIFO size
                                                                    register (OTG_FS_DIEPTXF4)                                 */
} OTG_FS_GLOBAL_Type;                           /*!< Size = 272 (0x110)                                                        */



/* =========================================================================================================================== */
/* ================                                        OTG_FS_HOST                                        ================ */
/* =========================================================================================================================== */


/**
  * @brief USB on the go full speed (OTG_FS_HOST)
  */

typedef struct {                                /*!< (@ 0x50000400) OTG_FS_HOST Structure                                      */
  __IO uint32_t   FS_HCFG;                      /*!< (@ 0x00000000) OTG_FS host configuration register (OTG_FS_HCFG)           */
  __IO uint32_t   HFIR;                         /*!< (@ 0x00000004) OTG_FS Host frame interval register                        */
  __I  uint32_t   FS_HFNUM;                     /*!< (@ 0x00000008) OTG_FS host frame number/frame time remaining
                                                                    register (OTG_FS_HFNUM)                                    */
  __I  uint32_t   RESERVED;
  __IO uint32_t   FS_HPTXSTS;                   /*!< (@ 0x00000010) OTG_FS_Host periodic transmit FIFO/queue status
                                                                    register (OTG_FS_HPTXSTS)                                  */
  __I  uint32_t   HAINT;                        /*!< (@ 0x00000014) OTG_FS Host all channels interrupt register                */
  __IO uint32_t   HAINTMSK;                     /*!< (@ 0x00000018) OTG_FS host all channels interrupt mask register           */
  __I  uint32_t   RESERVED1[9];
  __IO uint32_t   FS_HPRT;                      /*!< (@ 0x00000040) OTG_FS host port control and status register
                                                                    (OTG_FS_HPRT)                                              */
  __I  uint32_t   RESERVED2[47];
  __IO uint32_t   FS_HCCHAR0;                   /*!< (@ 0x00000100) OTG_FS host channel-0 characteristics register
                                                                    (OTG_FS_HCCHAR0)                                           */
  __I  uint32_t   RESERVED3;
  __IO uint32_t   FS_HCINT0;                    /*!< (@ 0x00000108) OTG_FS host channel-0 interrupt register (OTG_FS_HCINT0)   */
  __IO uint32_t   FS_HCINTMSK0;                 /*!< (@ 0x0000010C) OTG_FS host channel-0 mask register (OTG_FS_HCINTMSK0)     */
  __IO uint32_t   FS_HCTSIZ0;                   /*!< (@ 0x00000110) OTG_FS host channel-0 transfer size register               */
  __I  uint32_t   RESERVED4[3];
  __IO uint32_t   FS_HCCHAR1;                   /*!< (@ 0x00000120) OTG_FS host channel-1 characteristics register
                                                                    (OTG_FS_HCCHAR1)                                           */
  __I  uint32_t   RESERVED5;
  __IO uint32_t   FS_HCINT1;                    /*!< (@ 0x00000128) OTG_FS host channel-1 interrupt register (OTG_FS_HCINT1)   */
  __IO uint32_t   FS_HCINTMSK1;                 /*!< (@ 0x0000012C) OTG_FS host channel-1 mask register (OTG_FS_HCINTMSK1)     */
  __IO uint32_t   FS_HCTSIZ1;                   /*!< (@ 0x00000130) OTG_FS host channel-1 transfer size register               */
  __I  uint32_t   RESERVED6[3];
  __IO uint32_t   FS_HCCHAR2;                   /*!< (@ 0x00000140) OTG_FS host channel-2 characteristics register
                                                                    (OTG_FS_HCCHAR2)                                           */
  __I  uint32_t   RESERVED7;
  __IO uint32_t   FS_HCINT2;                    /*!< (@ 0x00000148) OTG_FS host channel-2 interrupt register (OTG_FS_HCINT2)   */
  __IO uint32_t   FS_HCINTMSK2;                 /*!< (@ 0x0000014C) OTG_FS host channel-2 mask register (OTG_FS_HCINTMSK2)     */
  __IO uint32_t   FS_HCTSIZ2;                   /*!< (@ 0x00000150) OTG_FS host channel-2 transfer size register               */
  __I  uint32_t   RESERVED8[3];
  __IO uint32_t   FS_HCCHAR3;                   /*!< (@ 0x00000160) OTG_FS host channel-3 characteristics register
                                                                    (OTG_FS_HCCHAR3)                                           */
  __I  uint32_t   RESERVED9;
  __IO uint32_t   FS_HCINT3;                    /*!< (@ 0x00000168) OTG_FS host channel-3 interrupt register (OTG_FS_HCINT3)   */
  __IO uint32_t   FS_HCINTMSK3;                 /*!< (@ 0x0000016C) OTG_FS host channel-3 mask register (OTG_FS_HCINTMSK3)     */
  __IO uint32_t   FS_HCTSIZ3;                   /*!< (@ 0x00000170) OTG_FS host channel-3 transfer size register               */
  __I  uint32_t   RESERVED10[3];
  __IO uint32_t   FS_HCCHAR4;                   /*!< (@ 0x00000180) OTG_FS host channel-4 characteristics register
                                                                    (OTG_FS_HCCHAR4)                                           */
  __I  uint32_t   RESERVED11;
  __IO uint32_t   FS_HCINT4;                    /*!< (@ 0x00000188) OTG_FS host channel-4 interrupt register (OTG_FS_HCINT4)   */
  __IO uint32_t   FS_HCINTMSK4;                 /*!< (@ 0x0000018C) OTG_FS host channel-4 mask register (OTG_FS_HCINTMSK4)     */
  __IO uint32_t   FS_HCTSIZ4;                   /*!< (@ 0x00000190) OTG_FS host channel-x transfer size register               */
  __I  uint32_t   RESERVED12[3];
  __IO uint32_t   FS_HCCHAR5;                   /*!< (@ 0x000001A0) OTG_FS host channel-5 characteristics register
                                                                    (OTG_FS_HCCHAR5)                                           */
  __I  uint32_t   RESERVED13;
  __IO uint32_t   FS_HCINT5;                    /*!< (@ 0x000001A8) OTG_FS host channel-5 interrupt register (OTG_FS_HCINT5)   */
  __IO uint32_t   FS_HCINTMSK5;                 /*!< (@ 0x000001AC) OTG_FS host channel-5 mask register (OTG_FS_HCINTMSK5)     */
  __IO uint32_t   FS_HCTSIZ5;                   /*!< (@ 0x000001B0) OTG_FS host channel-5 transfer size register               */
  __I  uint32_t   RESERVED14[3];
  __IO uint32_t   FS_HCCHAR6;                   /*!< (@ 0x000001C0) OTG_FS host channel-6 characteristics register
                                                                    (OTG_FS_HCCHAR6)                                           */
  __I  uint32_t   RESERVED15;
  __IO uint32_t   FS_HCINT6;                    /*!< (@ 0x000001C8) OTG_FS host channel-6 interrupt register (OTG_FS_HCINT6)   */
  __IO uint32_t   FS_HCINTMSK6;                 /*!< (@ 0x000001CC) OTG_FS host channel-6 mask register (OTG_FS_HCINTMSK6)     */
  __IO uint32_t   FS_HCTSIZ6;                   /*!< (@ 0x000001D0) OTG_FS host channel-6 transfer size register               */
  __I  uint32_t   RESERVED16[3];
  __IO uint32_t   FS_HCCHAR7;                   /*!< (@ 0x000001E0) OTG_FS host channel-7 characteristics register
                                                                    (OTG_FS_HCCHAR7)                                           */
  __I  uint32_t   RESERVED17;
  __IO uint32_t   FS_HCINT7;                    /*!< (@ 0x000001E8) OTG_FS host channel-7 interrupt register (OTG_FS_HCINT7)   */
  __IO uint32_t   FS_HCINTMSK7;                 /*!< (@ 0x000001EC) OTG_FS host channel-7 mask register (OTG_FS_HCINTMSK7)     */
  __IO uint32_t   FS_HCTSIZ7;                   /*!< (@ 0x000001F0) OTG_FS host channel-7 transfer size register               */
} OTG_FS_HOST_Type;                             /*!< Size = 500 (0x1f4)                                                        */



/* =========================================================================================================================== */
/* ================                                       OTG_FS_PWRCLK                                       ================ */
/* =========================================================================================================================== */


/**
  * @brief USB on the go full speed (OTG_FS_PWRCLK)
  */

typedef struct {                                /*!< (@ 0x50000E00) OTG_FS_PWRCLK Structure                                    */
  __IO uint32_t   FS_PCGCCTL;                   /*!< (@ 0x00000000) OTG_FS power and clock gating control register             */
} OTG_FS_PWRCLK_Type;                           /*!< Size = 4 (0x4)                                                            */



/* =========================================================================================================================== */
/* ================                                            PWR                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Power control (PWR)
  */

typedef struct {                                /*!< (@ 0x40007000) PWR Structure                                              */
  __IO uint32_t   CR;                           /*!< (@ 0x00000000) power control register                                     */
  __IO uint32_t   CSR;                          /*!< (@ 0x00000004) power control/status register                              */
} PWR_Type;                                     /*!< Size = 8 (0x8)                                                            */



/* =========================================================================================================================== */
/* ================                                            RCC                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Reset and clock control (RCC)
  */

typedef struct {                                /*!< (@ 0x40023800) RCC Structure                                              */
  __IO uint32_t   CR;                           /*!< (@ 0x00000000) clock control register                                     */
  __IO uint32_t   PLLCFGR;                      /*!< (@ 0x00000004) PLL configuration register                                 */
  __IO uint32_t   CFGR;                         /*!< (@ 0x00000008) clock configuration register                               */
  __IO uint32_t   CIR;                          /*!< (@ 0x0000000C) clock interrupt register                                   */
  __IO uint32_t   AHB1RSTR;                     /*!< (@ 0x00000010) AHB1 peripheral reset register                             */
  __IO uint32_t   AHB2RSTR;                     /*!< (@ 0x00000014) AHB2 peripheral reset register                             */
  __I  uint32_t   RESERVED[2];
  __IO uint32_t   APB1RSTR;                     /*!< (@ 0x00000020) APB1 peripheral reset register                             */
  __IO uint32_t   APB2RSTR;                     /*!< (@ 0x00000024) APB2 peripheral reset register                             */
  __I  uint32_t   RESERVED1[2];
  __IO uint32_t   AHB1ENR;                      /*!< (@ 0x00000030) AHB1 peripheral clock register                             */
  __IO uint32_t   AHB2ENR;                      /*!< (@ 0x00000034) AHB2 peripheral clock enable register                      */
  __I  uint32_t   RESERVED2[2];
  __IO uint32_t   APB1ENR;                      /*!< (@ 0x00000040) APB1 peripheral clock enable register                      */
  __IO uint32_t   APB2ENR;                      /*!< (@ 0x00000044) APB2 peripheral clock enable register                      */
  __I  uint32_t   RESERVED3[2];
  __IO uint32_t   AHB1LPENR;                    /*!< (@ 0x00000050) AHB1 peripheral clock enable in low power mode
                                                                    register                                                   */
  __IO uint32_t   AHB2LPENR;                    /*!< (@ 0x00000054) AHB2 peripheral clock enable in low power mode
                                                                    register                                                   */
  __I  uint32_t   RESERVED4[2];
  __IO uint32_t   APB1LPENR;                    /*!< (@ 0x00000060) APB1 peripheral clock enable in low power mode
                                                                    register                                                   */
  __IO uint32_t   APB2LPENR;                    /*!< (@ 0x00000064) APB2 peripheral clock enabled in low power mode
                                                                    register                                                   */
  __I  uint32_t   RESERVED5[2];
  __IO uint32_t   BDCR;                         /*!< (@ 0x00000070) Backup domain control register                             */
  __IO uint32_t   CSR;                          /*!< (@ 0x00000074) clock control & status register                            */
  __I  uint32_t   RESERVED6[2];
  __IO uint32_t   SSCGR;                        /*!< (@ 0x00000080) spread spectrum clock generation register                  */
  __IO uint32_t   PLLI2SCFGR;                   /*!< (@ 0x00000084) PLLI2S configuration register                              */
} RCC_Type;                                     /*!< Size = 136 (0x88)                                                         */



/* =========================================================================================================================== */
/* ================                                            RTC                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Real-time clock (RTC)
  */

typedef struct {                                /*!< (@ 0x40002800) RTC Structure                                              */
  __IO uint32_t   TR;                           /*!< (@ 0x00000000) time register                                              */
  __IO uint32_t   DR;                           /*!< (@ 0x00000004) date register                                              */
  __IO uint32_t   CR;                           /*!< (@ 0x00000008) control register                                           */
  __IO uint32_t   ISR;                          /*!< (@ 0x0000000C) initialization and status register                         */
  __IO uint32_t   PRER;                         /*!< (@ 0x00000010) prescaler register                                         */
  __IO uint32_t   WUTR;                         /*!< (@ 0x00000014) wakeup timer register                                      */
  __IO uint32_t   CALIBR;                       /*!< (@ 0x00000018) calibration register                                       */
  __IO uint32_t   ALRMAR;                       /*!< (@ 0x0000001C) alarm A register                                           */
  __IO uint32_t   ALRMBR;                       /*!< (@ 0x00000020) alarm B register                                           */
  __O  uint32_t   WPR;                          /*!< (@ 0x00000024) write protection register                                  */
  __I  uint32_t   SSR;                          /*!< (@ 0x00000028) sub second register                                        */
  __O  uint32_t   SHIFTR;                       /*!< (@ 0x0000002C) shift control register                                     */
  __I  uint32_t   TSTR;                         /*!< (@ 0x00000030) time stamp time register                                   */
  __I  uint32_t   TSDR;                         /*!< (@ 0x00000034) time stamp date register                                   */
  __I  uint32_t   TSSSR;                        /*!< (@ 0x00000038) timestamp sub second register                              */
  __IO uint32_t   CALR;                         /*!< (@ 0x0000003C) calibration register                                       */
  __IO uint32_t   TAFCR;                        /*!< (@ 0x00000040) tamper and alternate function configuration register       */
  __IO uint32_t   ALRMASSR;                     /*!< (@ 0x00000044) alarm A sub second register                                */
  __IO uint32_t   ALRMBSSR;                     /*!< (@ 0x00000048) alarm B sub second register                                */
  __I  uint32_t   RESERVED;
  __IO uint32_t   BKP0R;                        /*!< (@ 0x00000050) backup register                                            */
  __IO uint32_t   BKP1R;                        /*!< (@ 0x00000054) backup register                                            */
  __IO uint32_t   BKP2R;                        /*!< (@ 0x00000058) backup register                                            */
  __IO uint32_t   BKP3R;                        /*!< (@ 0x0000005C) backup register                                            */
  __IO uint32_t   BKP4R;                        /*!< (@ 0x00000060) backup register                                            */
  __IO uint32_t   BKP5R;                        /*!< (@ 0x00000064) backup register                                            */
  __IO uint32_t   BKP6R;                        /*!< (@ 0x00000068) backup register                                            */
  __IO uint32_t   BKP7R;                        /*!< (@ 0x0000006C) backup register                                            */
  __IO uint32_t   BKP8R;                        /*!< (@ 0x00000070) backup register                                            */
  __IO uint32_t   BKP9R;                        /*!< (@ 0x00000074) backup register                                            */
  __IO uint32_t   BKP10R;                       /*!< (@ 0x00000078) backup register                                            */
  __IO uint32_t   BKP11R;                       /*!< (@ 0x0000007C) backup register                                            */
  __IO uint32_t   BKP12R;                       /*!< (@ 0x00000080) backup register                                            */
  __IO uint32_t   BKP13R;                       /*!< (@ 0x00000084) backup register                                            */
  __IO uint32_t   BKP14R;                       /*!< (@ 0x00000088) backup register                                            */
  __IO uint32_t   BKP15R;                       /*!< (@ 0x0000008C) backup register                                            */
  __IO uint32_t   BKP16R;                       /*!< (@ 0x00000090) backup register                                            */
  __IO uint32_t   BKP17R;                       /*!< (@ 0x00000094) backup register                                            */
  __IO uint32_t   BKP18R;                       /*!< (@ 0x00000098) backup register                                            */
  __IO uint32_t   BKP19R;                       /*!< (@ 0x0000009C) backup register                                            */
} RTC_Type;                                     /*!< Size = 160 (0xa0)                                                         */



/* =========================================================================================================================== */
/* ================                                           SDIO                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Secure digital input/output  interface (SDIO)
  */

typedef struct {                                /*!< (@ 0x40012C00) SDIO Structure                                             */
  __IO uint32_t   POWER;                        /*!< (@ 0x00000000) power control register                                     */
  __IO uint32_t   CLKCR;                        /*!< (@ 0x00000004) SDI clock control register                                 */
  __IO uint32_t   ARG;                          /*!< (@ 0x00000008) argument register                                          */
  __IO uint32_t   CMD;                          /*!< (@ 0x0000000C) command register                                           */
  __I  uint32_t   RESPCMD;                      /*!< (@ 0x00000010) command response register                                  */
  __I  uint32_t   RESP1;                        /*!< (@ 0x00000014) response 1..4 register                                     */
  __I  uint32_t   RESP2;                        /*!< (@ 0x00000018) response 1..4 register                                     */
  __I  uint32_t   RESP3;                        /*!< (@ 0x0000001C) response 1..4 register                                     */
  __I  uint32_t   RESP4;                        /*!< (@ 0x00000020) response 1..4 register                                     */
  __IO uint32_t   DTIMER;                       /*!< (@ 0x00000024) data timer register                                        */
  __IO uint32_t   DLEN;                         /*!< (@ 0x00000028) data length register                                       */
  __IO uint32_t   DCTRL;                        /*!< (@ 0x0000002C) data control register                                      */
  __I  uint32_t   DCOUNT;                       /*!< (@ 0x00000030) data counter register                                      */
  __I  uint32_t   STA;                          /*!< (@ 0x00000034) status register                                            */
  __IO uint32_t   ICR;                          /*!< (@ 0x00000038) interrupt clear register                                   */
  __IO uint32_t   MASK;                         /*!< (@ 0x0000003C) mask register                                              */
  __I  uint32_t   RESERVED[2];
  __I  uint32_t   FIFOCNT;                      /*!< (@ 0x00000048) FIFO counter register                                      */
  __I  uint32_t   RESERVED1[13];
  __IO uint32_t   FIFO;                         /*!< (@ 0x00000080) data FIFO register                                         */
} SDIO_Type;                                    /*!< Size = 132 (0x84)                                                         */



/* =========================================================================================================================== */
/* ================                                          SYSCFG                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief System configuration controller (SYSCFG)
  */

typedef struct {                                /*!< (@ 0x40013800) SYSCFG Structure                                           */
  __IO uint32_t   MEMRM;                        /*!< (@ 0x00000000) memory remap register                                      */
  __IO uint32_t   PMC;                          /*!< (@ 0x00000004) peripheral mode configuration register                     */
  __IO uint32_t   EXTICR1;                      /*!< (@ 0x00000008) external interrupt configuration register 1                */
  __IO uint32_t   EXTICR2;                      /*!< (@ 0x0000000C) external interrupt configuration register 2                */
  __IO uint32_t   EXTICR3;                      /*!< (@ 0x00000010) external interrupt configuration register 3                */
  __IO uint32_t   EXTICR4;                      /*!< (@ 0x00000014) external interrupt configuration register 4                */
  __I  uint32_t   RESERVED[2];
  __I  uint32_t   CMPCR;                        /*!< (@ 0x00000020) Compensation cell control register                         */
} SYSCFG_Type;                                  /*!< Size = 36 (0x24)                                                          */



/* =========================================================================================================================== */
/* ================                                           TIM1                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Advanced-timers (TIM1)
  */

typedef struct {                                /*!< (@ 0x40010000) TIM1 Structure                                             */
  __IO uint32_t   CR1;                          /*!< (@ 0x00000000) control register 1                                         */
  __IO uint32_t   CR2;                          /*!< (@ 0x00000004) control register 2                                         */
  __IO uint32_t   SMCR;                         /*!< (@ 0x00000008) slave mode control register                                */
  __IO uint32_t   DIER;                         /*!< (@ 0x0000000C) DMA/Interrupt enable register                              */
  __IO uint32_t   SR;                           /*!< (@ 0x00000010) status register                                            */
  __O  uint32_t   EGR;                          /*!< (@ 0x00000014) event generation register                                  */
  
  union {
    __IO uint32_t CCMR1_Output;                 /*!< (@ 0x00000018) capture/compare mode register 1 (output mode)              */
    __IO uint32_t CCMR1_Input;                  /*!< (@ 0x00000018) capture/compare mode register 1 (input mode)               */
  };
  
  union {
    __IO uint32_t CCMR2_Output;                 /*!< (@ 0x0000001C) capture/compare mode register 2 (output mode)              */
    __IO uint32_t CCMR2_Input;                  /*!< (@ 0x0000001C) capture/compare mode register 2 (input mode)               */
  };
  __IO uint32_t   CCER;                         /*!< (@ 0x00000020) capture/compare enable register                            */
  __IO uint32_t   CNT;                          /*!< (@ 0x00000024) counter                                                    */
  __IO uint32_t   PSC;                          /*!< (@ 0x00000028) prescaler                                                  */
  __IO uint32_t   ARR;                          /*!< (@ 0x0000002C) auto-reload register                                       */
  __IO uint32_t   RCR;                          /*!< (@ 0x00000030) repetition counter register                                */
  __IO uint32_t   CCR1;                         /*!< (@ 0x00000034) capture/compare register 1                                 */
  __IO uint32_t   CCR2;                         /*!< (@ 0x00000038) capture/compare register 2                                 */
  __IO uint32_t   CCR3;                         /*!< (@ 0x0000003C) capture/compare register 3                                 */
  __IO uint32_t   CCR4;                         /*!< (@ 0x00000040) capture/compare register 4                                 */
  __IO uint32_t   BDTR;                         /*!< (@ 0x00000044) break and dead-time register                               */
  __IO uint32_t   DCR;                          /*!< (@ 0x00000048) DMA control register                                       */
  __IO uint32_t   DMAR;                         /*!< (@ 0x0000004C) DMA address for full transfer                              */
} TIM1_Type;                                    /*!< Size = 80 (0x50)                                                          */



/* =========================================================================================================================== */
/* ================                                           TIM10                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief General-purpose-timers (TIM10)
  */

typedef struct {                                /*!< (@ 0x40014400) TIM10 Structure                                            */
  __IO uint32_t   CR1;                          /*!< (@ 0x00000000) control register 1                                         */
  __I  uint32_t   RESERVED[2];
  __IO uint32_t   DIER;                         /*!< (@ 0x0000000C) DMA/Interrupt enable register                              */
  __IO uint32_t   SR;                           /*!< (@ 0x00000010) status register                                            */
  __O  uint32_t   EGR;                          /*!< (@ 0x00000014) event generation register                                  */
  
  union {
    __IO uint32_t CCMR1_Output;                 /*!< (@ 0x00000018) capture/compare mode register 1 (output mode)              */
    __IO uint32_t CCMR1_Input;                  /*!< (@ 0x00000018) capture/compare mode register 1 (input mode)               */
  };
  __I  uint32_t   RESERVED1;
  __IO uint32_t   CCER;                         /*!< (@ 0x00000020) capture/compare enable register                            */
  __IO uint32_t   CNT;                          /*!< (@ 0x00000024) counter                                                    */
  __IO uint32_t   PSC;                          /*!< (@ 0x00000028) prescaler                                                  */
  __IO uint32_t   ARR;                          /*!< (@ 0x0000002C) auto-reload register                                       */
  __I  uint32_t   RESERVED2;
  __IO uint32_t   CCR1;                         /*!< (@ 0x00000034) capture/compare register 1                                 */
} TIM10_Type;                                   /*!< Size = 56 (0x38)                                                          */



/* =========================================================================================================================== */
/* ================                                           TIM11                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief General-purpose-timers (TIM11)
  */

typedef struct {                                /*!< (@ 0x40014800) TIM11 Structure                                            */
  __IO uint32_t   CR1;                          /*!< (@ 0x00000000) control register 1                                         */
  __I  uint32_t   RESERVED[2];
  __IO uint32_t   DIER;                         /*!< (@ 0x0000000C) DMA/Interrupt enable register                              */
  __IO uint32_t   SR;                           /*!< (@ 0x00000010) status register                                            */
  __O  uint32_t   EGR;                          /*!< (@ 0x00000014) event generation register                                  */
  
  union {
    __IO uint32_t CCMR1_Output;                 /*!< (@ 0x00000018) capture/compare mode register 1 (output mode)              */
    __IO uint32_t CCMR1_Input;                  /*!< (@ 0x00000018) capture/compare mode register 1 (input mode)               */
  };
  __I  uint32_t   RESERVED1;
  __IO uint32_t   CCER;                         /*!< (@ 0x00000020) capture/compare enable register                            */
  __IO uint32_t   CNT;                          /*!< (@ 0x00000024) counter                                                    */
  __IO uint32_t   PSC;                          /*!< (@ 0x00000028) prescaler                                                  */
  __IO uint32_t   ARR;                          /*!< (@ 0x0000002C) auto-reload register                                       */
  __I  uint32_t   RESERVED2;
  __IO uint32_t   CCR1;                         /*!< (@ 0x00000034) capture/compare register 1                                 */
  __I  uint32_t   RESERVED3[6];
  __IO uint32_t   OR;                           /*!< (@ 0x00000050) option register                                            */
} TIM11_Type;                                   /*!< Size = 84 (0x54)                                                          */



/* =========================================================================================================================== */
/* ================                                           TIM2                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief General purpose timers (TIM2)
  */

typedef struct {                                /*!< (@ 0x40000000) TIM2 Structure                                             */
  __IO uint32_t   CR1;                          /*!< (@ 0x00000000) control register 1                                         */
  __IO uint32_t   CR2;                          /*!< (@ 0x00000004) control register 2                                         */
  __IO uint32_t   SMCR;                         /*!< (@ 0x00000008) slave mode control register                                */
  __IO uint32_t   DIER;                         /*!< (@ 0x0000000C) DMA/Interrupt enable register                              */
  __IO uint32_t   SR;                           /*!< (@ 0x00000010) status register                                            */
  __O  uint32_t   EGR;                          /*!< (@ 0x00000014) event generation register                                  */
  
  union {
    __IO uint32_t CCMR1_Output;                 /*!< (@ 0x00000018) capture/compare mode register 1 (output mode)              */
    __IO uint32_t CCMR1_Input;                  /*!< (@ 0x00000018) capture/compare mode register 1 (input mode)               */
  };
  
  union {
    __IO uint32_t CCMR2_Output;                 /*!< (@ 0x0000001C) capture/compare mode register 2 (output mode)              */
    __IO uint32_t CCMR2_Input;                  /*!< (@ 0x0000001C) capture/compare mode register 2 (input mode)               */
  };
  __IO uint32_t   CCER;                         /*!< (@ 0x00000020) capture/compare enable register                            */
  __IO uint32_t   CNT;                          /*!< (@ 0x00000024) counter                                                    */
  __IO uint32_t   PSC;                          /*!< (@ 0x00000028) prescaler                                                  */
  __IO uint32_t   ARR;                          /*!< (@ 0x0000002C) auto-reload register                                       */
  __I  uint32_t   RESERVED;
  __IO uint32_t   CCR1;                         /*!< (@ 0x00000034) capture/compare register 1                                 */
  __IO uint32_t   CCR2;                         /*!< (@ 0x00000038) capture/compare register 2                                 */
  __IO uint32_t   CCR3;                         /*!< (@ 0x0000003C) capture/compare register 3                                 */
  __IO uint32_t   CCR4;                         /*!< (@ 0x00000040) capture/compare register 4                                 */
  __I  uint32_t   RESERVED1;
  __IO uint32_t   DCR;                          /*!< (@ 0x00000048) DMA control register                                       */
  __IO uint32_t   DMAR;                         /*!< (@ 0x0000004C) DMA address for full transfer                              */
  __IO uint32_t   OR;                           /*!< (@ 0x00000050) TIM5 option register                                       */
} TIM2_Type;                                    /*!< Size = 84 (0x54)                                                          */



/* =========================================================================================================================== */
/* ================                                           TIM3                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief General purpose timers (TIM3)
  */

typedef struct {                                /*!< (@ 0x40000400) TIM3 Structure                                             */
  __IO uint32_t   CR1;                          /*!< (@ 0x00000000) control register 1                                         */
  __IO uint32_t   CR2;                          /*!< (@ 0x00000004) control register 2                                         */
  __IO uint32_t   SMCR;                         /*!< (@ 0x00000008) slave mode control register                                */
  __IO uint32_t   DIER;                         /*!< (@ 0x0000000C) DMA/Interrupt enable register                              */
  __IO uint32_t   SR;                           /*!< (@ 0x00000010) status register                                            */
  __O  uint32_t   EGR;                          /*!< (@ 0x00000014) event generation register                                  */
  
  union {
    __IO uint32_t CCMR1_Output;                 /*!< (@ 0x00000018) capture/compare mode register 1 (output mode)              */
    __IO uint32_t CCMR1_Input;                  /*!< (@ 0x00000018) capture/compare mode register 1 (input mode)               */
  };
  
  union {
    __IO uint32_t CCMR2_Output;                 /*!< (@ 0x0000001C) capture/compare mode register 2 (output mode)              */
    __IO uint32_t CCMR2_Input;                  /*!< (@ 0x0000001C) capture/compare mode register 2 (input mode)               */
  };
  __IO uint32_t   CCER;                         /*!< (@ 0x00000020) capture/compare enable register                            */
  __IO uint32_t   CNT;                          /*!< (@ 0x00000024) counter                                                    */
  __IO uint32_t   PSC;                          /*!< (@ 0x00000028) prescaler                                                  */
  __IO uint32_t   ARR;                          /*!< (@ 0x0000002C) auto-reload register                                       */
  __I  uint32_t   RESERVED;
  __IO uint32_t   CCR1;                         /*!< (@ 0x00000034) capture/compare register 1                                 */
  __IO uint32_t   CCR2;                         /*!< (@ 0x00000038) capture/compare register 2                                 */
  __IO uint32_t   CCR3;                         /*!< (@ 0x0000003C) capture/compare register 3                                 */
  __IO uint32_t   CCR4;                         /*!< (@ 0x00000040) capture/compare register 4                                 */
  __I  uint32_t   RESERVED1;
  __IO uint32_t   DCR;                          /*!< (@ 0x00000048) DMA control register                                       */
  __IO uint32_t   DMAR;                         /*!< (@ 0x0000004C) DMA address for full transfer                              */
} TIM3_Type;                                    /*!< Size = 80 (0x50)                                                          */



/* =========================================================================================================================== */
/* ================                                           TIM5                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief General-purpose-timers (TIM5)
  */

typedef struct {                                /*!< (@ 0x40000C00) TIM5 Structure                                             */
  __IO uint32_t   CR1;                          /*!< (@ 0x00000000) control register 1                                         */
  __IO uint32_t   CR2;                          /*!< (@ 0x00000004) control register 2                                         */
  __IO uint32_t   SMCR;                         /*!< (@ 0x00000008) slave mode control register                                */
  __IO uint32_t   DIER;                         /*!< (@ 0x0000000C) DMA/Interrupt enable register                              */
  __IO uint32_t   SR;                           /*!< (@ 0x00000010) status register                                            */
  __O  uint32_t   EGR;                          /*!< (@ 0x00000014) event generation register                                  */
  
  union {
    __IO uint32_t CCMR1_Output;                 /*!< (@ 0x00000018) capture/compare mode register 1 (output mode)              */
    __IO uint32_t CCMR1_Input;                  /*!< (@ 0x00000018) capture/compare mode register 1 (input mode)               */
  };
  
  union {
    __IO uint32_t CCMR2_Output;                 /*!< (@ 0x0000001C) capture/compare mode register 2 (output mode)              */
    __IO uint32_t CCMR2_Input;                  /*!< (@ 0x0000001C) capture/compare mode register 2 (input mode)               */
  };
  __IO uint32_t   CCER;                         /*!< (@ 0x00000020) capture/compare enable register                            */
  __IO uint32_t   CNT;                          /*!< (@ 0x00000024) counter                                                    */
  __IO uint32_t   PSC;                          /*!< (@ 0x00000028) prescaler                                                  */
  __IO uint32_t   ARR;                          /*!< (@ 0x0000002C) auto-reload register                                       */
  __I  uint32_t   RESERVED;
  __IO uint32_t   CCR1;                         /*!< (@ 0x00000034) capture/compare register 1                                 */
  __IO uint32_t   CCR2;                         /*!< (@ 0x00000038) capture/compare register 2                                 */
  __IO uint32_t   CCR3;                         /*!< (@ 0x0000003C) capture/compare register 3                                 */
  __IO uint32_t   CCR4;                         /*!< (@ 0x00000040) capture/compare register 4                                 */
  __I  uint32_t   RESERVED1;
  __IO uint32_t   DCR;                          /*!< (@ 0x00000048) DMA control register                                       */
  __IO uint32_t   DMAR;                         /*!< (@ 0x0000004C) DMA address for full transfer                              */
  __IO uint32_t   OR;                           /*!< (@ 0x00000050) TIM5 option register                                       */
} TIM5_Type;                                    /*!< Size = 84 (0x54)                                                          */



/* =========================================================================================================================== */
/* ================                                           TIM9                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief General purpose timers (TIM9)
  */

typedef struct {                                /*!< (@ 0x40014000) TIM9 Structure                                             */
  __IO uint32_t   CR1;                          /*!< (@ 0x00000000) control register 1                                         */
  __IO uint32_t   CR2;                          /*!< (@ 0x00000004) control register 2                                         */
  __IO uint32_t   SMCR;                         /*!< (@ 0x00000008) slave mode control register                                */
  __IO uint32_t   DIER;                         /*!< (@ 0x0000000C) DMA/Interrupt enable register                              */
  __IO uint32_t   SR;                           /*!< (@ 0x00000010) status register                                            */
  __O  uint32_t   EGR;                          /*!< (@ 0x00000014) event generation register                                  */
  
  union {
    __IO uint32_t CCMR1_Output;                 /*!< (@ 0x00000018) capture/compare mode register 1 (output mode)              */
    __IO uint32_t CCMR1_Input;                  /*!< (@ 0x00000018) capture/compare mode register 1 (input mode)               */
  };
  __I  uint32_t   RESERVED;
  __IO uint32_t   CCER;                         /*!< (@ 0x00000020) capture/compare enable register                            */
  __IO uint32_t   CNT;                          /*!< (@ 0x00000024) counter                                                    */
  __IO uint32_t   PSC;                          /*!< (@ 0x00000028) prescaler                                                  */
  __IO uint32_t   ARR;                          /*!< (@ 0x0000002C) auto-reload register                                       */
  __I  uint32_t   RESERVED1;
  __IO uint32_t   CCR1;                         /*!< (@ 0x00000034) capture/compare register 1                                 */
  __IO uint32_t   CCR2;                         /*!< (@ 0x00000038) capture/compare register 2                                 */
} TIM9_Type;                                    /*!< Size = 60 (0x3c)                                                          */



/* =========================================================================================================================== */
/* ================                                          USART1                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief Universal synchronous asynchronous receiver  transmitter (USART1)
  */

typedef struct {                                /*!< (@ 0x40011000) USART1 Structure                                           */
  __IO uint32_t   SR;                           /*!< (@ 0x00000000) Status register                                            */
  __IO uint32_t   DR;                           /*!< (@ 0x00000004) Data register                                              */
  __IO uint32_t   BRR;                          /*!< (@ 0x00000008) Baud rate register                                         */
  __IO uint32_t   CR1;                          /*!< (@ 0x0000000C) Control register 1                                         */
  __IO uint32_t   CR2;                          /*!< (@ 0x00000010) Control register 2                                         */
  __IO uint32_t   CR3;                          /*!< (@ 0x00000014) Control register 3                                         */
  __IO uint32_t   GTPR;                         /*!< (@ 0x00000018) Guard time and prescaler register                          */
} USART1_Type;                                  /*!< Size = 28 (0x1c)                                                          */



/* =========================================================================================================================== */
/* ================                                           WWDG                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Window watchdog (WWDG)
  */

typedef struct {                                /*!< (@ 0x40002C00) WWDG Structure                                             */
  __IO uint32_t   CR;                           /*!< (@ 0x00000000) Control register                                           */
  __IO uint32_t   CFR;                          /*!< (@ 0x00000004) Configuration register                                     */
  __IO uint32_t   SR;                           /*!< (@ 0x00000008) Status register                                            */
} WWDG_Type;                                    /*!< Size = 12 (0xc)                                                           */



/* =========================================================================================================================== */
/* ================                                           DMA2                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief DMA controller (DMA2)
  */

typedef struct {                                /*!< (@ 0x40026400) DMA2 Structure                                             */
  __I  uint32_t   LISR;                         /*!< (@ 0x00000000) low interrupt status register                              */
  __I  uint32_t   HISR;                         /*!< (@ 0x00000004) high interrupt status register                             */
  __O  uint32_t   LIFCR;                        /*!< (@ 0x00000008) low interrupt flag clear register                          */
  __O  uint32_t   HIFCR;                        /*!< (@ 0x0000000C) high interrupt flag clear register                         */
  __IO uint32_t   S0CR;                         /*!< (@ 0x00000010) stream x configuration register                            */
  __IO uint32_t   S0NDTR;                       /*!< (@ 0x00000014) stream x number of data register                           */
  __IO uint32_t   S0PAR;                        /*!< (@ 0x00000018) stream x peripheral address register                       */
  __IO uint32_t   S0M0AR;                       /*!< (@ 0x0000001C) stream x memory 0 address register                         */
  __IO uint32_t   S0M1AR;                       /*!< (@ 0x00000020) stream x memory 1 address register                         */
  __IO uint32_t   S0FCR;                        /*!< (@ 0x00000024) stream x FIFO control register                             */
  __IO uint32_t   S1CR;                         /*!< (@ 0x00000028) stream x configuration register                            */
  __IO uint32_t   S1NDTR;                       /*!< (@ 0x0000002C) stream x number of data register                           */
  __IO uint32_t   S1PAR;                        /*!< (@ 0x00000030) stream x peripheral address register                       */
  __IO uint32_t   S1M0AR;                       /*!< (@ 0x00000034) stream x memory 0 address register                         */
  __IO uint32_t   S1M1AR;                       /*!< (@ 0x00000038) stream x memory 1 address register                         */
  __IO uint32_t   S1FCR;                        /*!< (@ 0x0000003C) stream x FIFO control register                             */
  __IO uint32_t   S2CR;                         /*!< (@ 0x00000040) stream x configuration register                            */
  __IO uint32_t   S2NDTR;                       /*!< (@ 0x00000044) stream x number of data register                           */
  __IO uint32_t   S2PAR;                        /*!< (@ 0x00000048) stream x peripheral address register                       */
  __IO uint32_t   S2M0AR;                       /*!< (@ 0x0000004C) stream x memory 0 address register                         */
  __IO uint32_t   S2M1AR;                       /*!< (@ 0x00000050) stream x memory 1 address register                         */
  __IO uint32_t   S2FCR;                        /*!< (@ 0x00000054) stream x FIFO control register                             */
  __IO uint32_t   S3CR;                         /*!< (@ 0x00000058) stream x configuration register                            */
  __IO uint32_t   S3NDTR;                       /*!< (@ 0x0000005C) stream x number of data register                           */
  __IO uint32_t   S3PAR;                        /*!< (@ 0x00000060) stream x peripheral address register                       */
  __IO uint32_t   S3M0AR;                       /*!< (@ 0x00000064) stream x memory 0 address register                         */
  __IO uint32_t   S3M1AR;                       /*!< (@ 0x00000068) stream x memory 1 address register                         */
  __IO uint32_t   S3FCR;                        /*!< (@ 0x0000006C) stream x FIFO control register                             */
  __IO uint32_t   S4CR;                         /*!< (@ 0x00000070) stream x configuration register                            */
  __IO uint32_t   S4NDTR;                       /*!< (@ 0x00000074) stream x number of data register                           */
  __IO uint32_t   S4PAR;                        /*!< (@ 0x00000078) stream x peripheral address register                       */
  __IO uint32_t   S4M0AR;                       /*!< (@ 0x0000007C) stream x memory 0 address register                         */
  __IO uint32_t   S4M1AR;                       /*!< (@ 0x00000080) stream x memory 1 address register                         */
  __IO uint32_t   S4FCR;                        /*!< (@ 0x00000084) stream x FIFO control register                             */
  __IO uint32_t   S5CR;                         /*!< (@ 0x00000088) stream x configuration register                            */
  __IO uint32_t   S5NDTR;                       /*!< (@ 0x0000008C) stream x number of data register                           */
  __IO uint32_t   S5PAR;                        /*!< (@ 0x00000090) stream x peripheral address register                       */
  __IO uint32_t   S5M0AR;                       /*!< (@ 0x00000094) stream x memory 0 address register                         */
  __IO uint32_t   S5M1AR;                       /*!< (@ 0x00000098) stream x memory 1 address register                         */
  __IO uint32_t   S5FCR;                        /*!< (@ 0x0000009C) stream x FIFO control register                             */
  __IO uint32_t   S6CR;                         /*!< (@ 0x000000A0) stream x configuration register                            */
  __IO uint32_t   S6NDTR;                       /*!< (@ 0x000000A4) stream x number of data register                           */
  __IO uint32_t   S6PAR;                        /*!< (@ 0x000000A8) stream x peripheral address register                       */
  __IO uint32_t   S6M0AR;                       /*!< (@ 0x000000AC) stream x memory 0 address register                         */
  __IO uint32_t   S6M1AR;                       /*!< (@ 0x000000B0) stream x memory 1 address register                         */
  __IO uint32_t   S6FCR;                        /*!< (@ 0x000000B4) stream x FIFO control register                             */
  __IO uint32_t   S7CR;                         /*!< (@ 0x000000B8) stream x configuration register                            */
  __IO uint32_t   S7NDTR;                       /*!< (@ 0x000000BC) stream x number of data register                           */
  __IO uint32_t   S7PAR;                        /*!< (@ 0x000000C0) stream x peripheral address register                       */
  __IO uint32_t   S7M0AR;                       /*!< (@ 0x000000C4) stream x memory 0 address register                         */
  __IO uint32_t   S7M1AR;                       /*!< (@ 0x000000C8) stream x memory 1 address register                         */
  __IO uint32_t   S7FCR;                        /*!< (@ 0x000000CC) stream x FIFO control register                             */
} DMA2_Type;                                    /*!< Size = 208 (0xd0)                                                         */



/* =========================================================================================================================== */
/* ================                                           GPIOH                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief General-purpose I/Os (GPIOH)
  */

typedef struct {                                /*!< (@ 0x40021C00) GPIOH Structure                                            */
  __IO uint32_t   MODER;                        /*!< (@ 0x00000000) GPIO port mode register                                    */
  __IO uint32_t   OTYPER;                       /*!< (@ 0x00000004) GPIO port output type register                             */
  __IO uint32_t   OSPEEDR;                      /*!< (@ 0x00000008) GPIO port output speed register                            */
  __IO uint32_t   PUPDR;                        /*!< (@ 0x0000000C) GPIO port pull-up/pull-down register                       */
  __I  uint32_t   IDR;                          /*!< (@ 0x00000010) GPIO port input data register                              */
  __IO uint32_t   ODR;                          /*!< (@ 0x00000014) GPIO port output data register                             */
  __O  uint32_t   BSRR;                         /*!< (@ 0x00000018) GPIO port bit set/reset register                           */
  __IO uint32_t   LCKR;                         /*!< (@ 0x0000001C) GPIO port configuration lock register                      */
  __IO uint32_t   AFRL;                         /*!< (@ 0x00000020) GPIO alternate function low register                       */
  __IO uint32_t   AFRH;                         /*!< (@ 0x00000024) GPIO alternate function high register                      */
} GPIOH_Type;                                   /*!< Size = 40 (0x28)                                                          */



/* =========================================================================================================================== */
/* ================                                           GPIOB                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief General-purpose I/Os (GPIOB)
  */

typedef struct {                                /*!< (@ 0x40020400) GPIOB Structure                                            */
  __IO uint32_t   MODER;                        /*!< (@ 0x00000000) GPIO port mode register                                    */
  __IO uint32_t   OTYPER;                       /*!< (@ 0x00000004) GPIO port output type register                             */
  __IO uint32_t   OSPEEDR;                      /*!< (@ 0x00000008) GPIO port output speed register                            */
  __IO uint32_t   PUPDR;                        /*!< (@ 0x0000000C) GPIO port pull-up/pull-down register                       */
  __I  uint32_t   IDR;                          /*!< (@ 0x00000010) GPIO port input data register                              */
  __IO uint32_t   ODR;                          /*!< (@ 0x00000014) GPIO port output data register                             */
  __O  uint32_t   BSRR;                         /*!< (@ 0x00000018) GPIO port bit set/reset register                           */
  __IO uint32_t   LCKR;                         /*!< (@ 0x0000001C) GPIO port configuration lock register                      */
  __IO uint32_t   AFRL;                         /*!< (@ 0x00000020) GPIO alternate function low register                       */
  __IO uint32_t   AFRH;                         /*!< (@ 0x00000024) GPIO alternate function high register                      */
} GPIOB_Type;                                   /*!< Size = 40 (0x28)                                                          */



/* =========================================================================================================================== */
/* ================                                           GPIOA                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief General-purpose I/Os (GPIOA)
  */

typedef struct {                                /*!< (@ 0x40020000) GPIOA Structure                                            */
  __IO uint32_t   MODER;                        /*!< (@ 0x00000000) GPIO port mode register                                    */
  __IO uint32_t   OTYPER;                       /*!< (@ 0x00000004) GPIO port output type register                             */
  __IO uint32_t   OSPEEDR;                      /*!< (@ 0x00000008) GPIO port output speed register                            */
  __IO uint32_t   PUPDR;                        /*!< (@ 0x0000000C) GPIO port pull-up/pull-down register                       */
  __I  uint32_t   IDR;                          /*!< (@ 0x00000010) GPIO port input data register                              */
  __IO uint32_t   ODR;                          /*!< (@ 0x00000014) GPIO port output data register                             */
  __O  uint32_t   BSRR;                         /*!< (@ 0x00000018) GPIO port bit set/reset register                           */
  __IO uint32_t   LCKR;                         /*!< (@ 0x0000001C) GPIO port configuration lock register                      */
  __IO uint32_t   AFRL;                         /*!< (@ 0x00000020) GPIO alternate function low register                       */
  __IO uint32_t   AFRH;                         /*!< (@ 0x00000024) GPIO alternate function high register                      */
} GPIOA_Type;                                   /*!< Size = 40 (0x28)                                                          */



/* =========================================================================================================================== */
/* ================                                           I2C3                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Inter-integrated circuit (I2C3)
  */

typedef struct {                                /*!< (@ 0x40005C00) I2C3 Structure                                             */
  __IO uint32_t   CR1;                          /*!< (@ 0x00000000) Control register 1                                         */
  __IO uint32_t   CR2;                          /*!< (@ 0x00000004) Control register 2                                         */
  __IO uint32_t   OAR1;                         /*!< (@ 0x00000008) Own address register 1                                     */
  __IO uint32_t   OAR2;                         /*!< (@ 0x0000000C) Own address register 2                                     */
  __IO uint32_t   DR;                           /*!< (@ 0x00000010) Data register                                              */
  __IO uint32_t   SR1;                          /*!< (@ 0x00000014) Status register 1                                          */
  __I  uint32_t   SR2;                          /*!< (@ 0x00000018) Status register 2                                          */
  __IO uint32_t   CCR;                          /*!< (@ 0x0000001C) Clock control register                                     */
  __IO uint32_t   TRISE;                        /*!< (@ 0x00000020) TRISE register                                             */
} I2C3_Type;                                    /*!< Size = 36 (0x24)                                                          */



/* =========================================================================================================================== */
/* ================                                          I2S2ext                                          ================ */
/* =========================================================================================================================== */


/**
  * @brief Serial peripheral interface (I2S2ext)
  */

typedef struct {                                /*!< (@ 0x40003400) I2S2ext Structure                                          */
  __IO uint32_t   CR1;                          /*!< (@ 0x00000000) control register 1                                         */
  __IO uint32_t   CR2;                          /*!< (@ 0x00000004) control register 2                                         */
  __IO uint32_t   SR;                           /*!< (@ 0x00000008) status register                                            */
  __IO uint32_t   DR;                           /*!< (@ 0x0000000C) data register                                              */
  __IO uint32_t   CRCPR;                        /*!< (@ 0x00000010) CRC polynomial register                                    */
  __I  uint32_t   RXCRCR;                       /*!< (@ 0x00000014) RX CRC register                                            */
  __I  uint32_t   TXCRCR;                       /*!< (@ 0x00000018) TX CRC register                                            */
  __IO uint32_t   I2SCFGR;                      /*!< (@ 0x0000001C) I2S configuration register                                 */
  __IO uint32_t   I2SPR;                        /*!< (@ 0x00000020) I2S prescaler register                                     */
} I2S2ext_Type;                                 /*!< Size = 36 (0x24)                                                          */



/* =========================================================================================================================== */
/* ================                                           NVIC                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Nested Vectored Interrupt  Controller (NVIC)
  */

typedef struct {                                /*!< (@ 0xE000E100) NVIC Structure                                             */
  __IO uint32_t   ISER0;                        /*!< (@ 0x00000000) Interrupt Set-Enable Register                              */
  __IO uint32_t   ISER1;                        /*!< (@ 0x00000004) Interrupt Set-Enable Register                              */
  __IO uint32_t   ISER2;                        /*!< (@ 0x00000008) Interrupt Set-Enable Register                              */
  __I  uint32_t   RESERVED[29];
  __IO uint32_t   ICER0;                        /*!< (@ 0x00000080) Interrupt Clear-Enable Register                            */
  __IO uint32_t   ICER1;                        /*!< (@ 0x00000084) Interrupt Clear-Enable Register                            */
  __IO uint32_t   ICER2;                        /*!< (@ 0x00000088) Interrupt Clear-Enable Register                            */
  __I  uint32_t   RESERVED1[29];
  __IO uint32_t   ISPR0;                        /*!< (@ 0x00000100) Interrupt Set-Pending Register                             */
  __IO uint32_t   ISPR1;                        /*!< (@ 0x00000104) Interrupt Set-Pending Register                             */
  __IO uint32_t   ISPR2;                        /*!< (@ 0x00000108) Interrupt Set-Pending Register                             */
  __I  uint32_t   RESERVED2[29];
  __IO uint32_t   ICPR0;                        /*!< (@ 0x00000180) Interrupt Clear-Pending Register                           */
  __IO uint32_t   ICPR1;                        /*!< (@ 0x00000184) Interrupt Clear-Pending Register                           */
  __IO uint32_t   ICPR2;                        /*!< (@ 0x00000188) Interrupt Clear-Pending Register                           */
  __I  uint32_t   RESERVED3[29];
  __I  uint32_t   IABR0;                        /*!< (@ 0x00000200) Interrupt Active Bit Register                              */
  __I  uint32_t   IABR1;                        /*!< (@ 0x00000204) Interrupt Active Bit Register                              */
  __I  uint32_t   IABR2;                        /*!< (@ 0x00000208) Interrupt Active Bit Register                              */
  __I  uint32_t   RESERVED4[61];
  __IO uint32_t   IPR0;                         /*!< (@ 0x00000300) Interrupt Priority Register                                */
  __IO uint32_t   IPR1;                         /*!< (@ 0x00000304) Interrupt Priority Register                                */
  __IO uint32_t   IPR2;                         /*!< (@ 0x00000308) Interrupt Priority Register                                */
  __IO uint32_t   IPR3;                         /*!< (@ 0x0000030C) Interrupt Priority Register                                */
  __IO uint32_t   IPR4;                         /*!< (@ 0x00000310) Interrupt Priority Register                                */
  __IO uint32_t   IPR5;                         /*!< (@ 0x00000314) Interrupt Priority Register                                */
  __IO uint32_t   IPR6;                         /*!< (@ 0x00000318) Interrupt Priority Register                                */
  __IO uint32_t   IPR7;                         /*!< (@ 0x0000031C) Interrupt Priority Register                                */
  __IO uint32_t   IPR8;                         /*!< (@ 0x00000320) Interrupt Priority Register                                */
  __IO uint32_t   IPR9;                         /*!< (@ 0x00000324) Interrupt Priority Register                                */
  __IO uint32_t   IPR10;                        /*!< (@ 0x00000328) Interrupt Priority Register                                */
  __IO uint32_t   IPR11;                        /*!< (@ 0x0000032C) Interrupt Priority Register                                */
  __IO uint32_t   IPR12;                        /*!< (@ 0x00000330) Interrupt Priority Register                                */
  __IO uint32_t   IPR13;                        /*!< (@ 0x00000334) Interrupt Priority Register                                */
  __IO uint32_t   IPR14;                        /*!< (@ 0x00000338) Interrupt Priority Register                                */
  __IO uint32_t   IPR15;                        /*!< (@ 0x0000033C) Interrupt Priority Register                                */
  __IO uint32_t   IPR16;                        /*!< (@ 0x00000340) Interrupt Priority Register                                */
  __IO uint32_t   IPR17;                        /*!< (@ 0x00000344) Interrupt Priority Register                                */
  __IO uint32_t   IPR18;                        /*!< (@ 0x00000348) Interrupt Priority Register                                */
  __IO uint32_t   IPR19;                        /*!< (@ 0x0000034C) Interrupt Priority Register                                */
} NVIC_Type;                                    /*!< Size = 848 (0x350)                                                        */



/* =========================================================================================================================== */
/* ================                                            FPU                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Floting point unit (FPU)
  */

typedef struct {                                /*!< (@ 0xE000EF34) FPU Structure                                              */
  __IO uint32_t   FPCCR;                        /*!< (@ 0x00000000) Floating-point context control register                    */
  __IO uint32_t   FPCAR;                        /*!< (@ 0x00000004) Floating-point context address register                    */
  __IO uint32_t   FPSCR;                        /*!< (@ 0x00000008) Floating-point status control register                     */
} FPU_Type;                                     /*!< Size = 12 (0xc)                                                           */



/* =========================================================================================================================== */
/* ================                                            MPU                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Memory protection unit (MPU)
  */

typedef struct {                                /*!< (@ 0xE000ED90) MPU Structure                                              */
  __I  uint32_t   MPU_TYPER;                    /*!< (@ 0x00000000) MPU type register                                          */
  __I  uint32_t   MPU_CTRL;                     /*!< (@ 0x00000004) MPU control register                                       */
  __IO uint32_t   MPU_RNR;                      /*!< (@ 0x00000008) MPU region number register                                 */
  __IO uint32_t   MPU_RBAR;                     /*!< (@ 0x0000000C) MPU region base address register                           */
  __IO uint32_t   MPU_RASR;                     /*!< (@ 0x00000010) MPU region attribute and size register                     */
} MPU_Type;                                     /*!< Size = 20 (0x14)                                                          */



/* =========================================================================================================================== */
/* ================                                            STK                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief SysTick timer (STK)
  */

typedef struct {                                /*!< (@ 0xE000E010) STK Structure                                              */
  __IO uint32_t   CTRL;                         /*!< (@ 0x00000000) SysTick control and status register                        */
  __IO uint32_t   LOAD;                         /*!< (@ 0x00000004) SysTick reload value register                              */
  __IO uint32_t   VAL;                          /*!< (@ 0x00000008) SysTick current value register                             */
  __IO uint32_t   CALIB;                        /*!< (@ 0x0000000C) SysTick calibration value register                         */
} STK_Type;                                     /*!< Size = 16 (0x10)                                                          */



/* =========================================================================================================================== */
/* ================                                            SCB                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief System control block (SCB)
  */

typedef struct {                                /*!< (@ 0xE000ED00) SCB Structure                                              */
  __I  uint32_t   CPUID;                        /*!< (@ 0x00000000) CPUID base register                                        */
  __IO uint32_t   ICSR;                         /*!< (@ 0x00000004) Interrupt control and state register                       */
  __IO uint32_t   VTOR;                         /*!< (@ 0x00000008) Vector table offset register                               */
  __IO uint32_t   AIRCR;                        /*!< (@ 0x0000000C) Application interrupt and reset control register           */
  __IO uint32_t   SCR;                          /*!< (@ 0x00000010) System control register                                    */
  __IO uint32_t   CCR;                          /*!< (@ 0x00000014) Configuration and control register                         */
  __IO uint32_t   SHPR1;                        /*!< (@ 0x00000018) System handler priority registers                          */
  __IO uint32_t   SHPR2;                        /*!< (@ 0x0000001C) System handler priority registers                          */
  __IO uint32_t   SHPR3;                        /*!< (@ 0x00000020) System handler priority registers                          */
  __IO uint32_t   SHCRS;                        /*!< (@ 0x00000024) System handler control and state register                  */
  __IO uint32_t   CFSR_UFSR_BFSR_MMFSR;         /*!< (@ 0x00000028) Configurable fault status register                         */
  __IO uint32_t   HFSR;                         /*!< (@ 0x0000002C) Hard fault status register                                 */
  __I  uint32_t   RESERVED;
  __IO uint32_t   MMFAR;                        /*!< (@ 0x00000034) Memory management fault address register                   */
  __IO uint32_t   BFAR;                         /*!< (@ 0x00000038) Bus fault address register                                 */
  __IO uint32_t   AFSR;                         /*!< (@ 0x0000003C) Auxiliary fault status register                            */
} SCB_Type;                                     /*!< Size = 64 (0x40)                                                          */



/* =========================================================================================================================== */
/* ================                                         NVIC_STIR                                         ================ */
/* =========================================================================================================================== */


/**
  * @brief Nested vectored interrupt  controller (NVIC_STIR)
  */

typedef struct {                                /*!< (@ 0xE000EF00) NVIC_STIR Structure                                        */
  __IO uint32_t   STIR;                         /*!< (@ 0x00000000) Software trigger interrupt register                        */
} NVIC_STIR_Type;                               /*!< Size = 4 (0x4)                                                            */



/* =========================================================================================================================== */
/* ================                                         FPU_CPACR                                         ================ */
/* =========================================================================================================================== */


/**
  * @brief Floating point unit CPACR (FPU_CPACR)
  */

typedef struct {                                /*!< (@ 0xE000ED88) FPU_CPACR Structure                                        */
  __IO uint32_t   CPACR;                        /*!< (@ 0x00000000) Coprocessor access control register                        */
} FPU_CPACR_Type;                               /*!< Size = 4 (0x4)                                                            */



/* =========================================================================================================================== */
/* ================                                         SCB_ACTRL                                         ================ */
/* =========================================================================================================================== */


/**
  * @brief System control block ACTLR (SCB_ACTRL)
  */

typedef struct {                                /*!< (@ 0xE000E008) SCB_ACTRL Structure                                        */
  __IO uint32_t   ACTRL;                        /*!< (@ 0x00000000) Auxiliary control register                                 */
} SCB_ACTRL_Type;                               /*!< Size = 4 (0x4)                                                            */


/** @} */ /* End of group Device_Peripheral_peripherals */


/* =========================================  End of section using anonymous unions  ========================================= */
#if defined(__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined(__ICCARM__)
  #pragma language=extended
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning 586
#else
  #warning Not supported compiler type
#endif


/* =========================================================================================================================== */
/* ================                          Device Specific Peripheral Address Map                           ================ */
/* =========================================================================================================================== */


/** @addtogroup Device_Peripheral_peripheralAddr
  * @{
  */

#define ADC_Common_BASE             0x40012300UL
#define ADC1_BASE                   0x40012000UL
#define CRC_BASE                    0x40023000UL
#define DBG_BASE                    0xE0042000UL
#define EXTI_BASE                   0x40013C00UL
#define FLASH_BASE                  0x40023C00UL
#define IWDG_BASE                   0x40003000UL
#define OTG_FS_DEVICE_BASE          0x50000800UL
#define OTG_FS_GLOBAL_BASE          0x50000000UL
#define OTG_FS_HOST_BASE            0x50000400UL
#define OTG_FS_PWRCLK_BASE          0x50000E00UL
#define PWR_BASE                    0x40007000UL
#define RCC_BASE                    0x40023800UL
#define RTC_BASE                    0x40002800UL
#define SDIO_BASE                   0x40012C00UL
#define SYSCFG_BASE                 0x40013800UL
#define TIM1_BASE                   0x40010000UL
#define TIM8_BASE                   0x40010400UL
#define TIM10_BASE                  0x40014400UL
#define TIM11_BASE                  0x40014800UL
#define TIM2_BASE                   0x40000000UL
#define TIM3_BASE                   0x40000400UL
#define TIM4_BASE                   0x40000800UL
#define TIM5_BASE                   0x40000C00UL
#define TIM9_BASE                   0x40014000UL
#define USART1_BASE                 0x40011000UL
#define USART2_BASE                 0x40004400UL
#define USART6_BASE                 0x40011400UL
#define WWDG_BASE                   0x40002C00UL
#define DMA2_BASE                   0x40026400UL
#define DMA1_BASE                   0x40026000UL
#define GPIOH_BASE                  0x40021C00UL
#define GPIOE_BASE                  0x40021000UL
#define GPIOD_BASE                  0x40020C00UL
#define GPIOC_BASE                  0x40020800UL
#define GPIOB_BASE                  0x40020400UL
#define GPIOA_BASE                  0x40020000UL
#define I2C3_BASE                   0x40005C00UL
#define I2C2_BASE                   0x40005800UL
#define I2C1_BASE                   0x40005400UL
#define I2S2ext_BASE                0x40003400UL
#define I2S3ext_BASE                0x40004000UL
#define SPI1_BASE                   0x40013000UL
#define SPI2_BASE                   0x40003800UL
#define SPI3_BASE                   0x40003C00UL
#define SPI4_BASE                   0x40013400UL
#define SPI5_BASE                   0x40015000UL
#define NVIC_BASE                   0xE000E100UL
#define FPU_BASE                    0xE000EF34UL
#define MPU_BASE                    0xE000ED90UL
#define STK_BASE                    0xE000E010UL
#define SCB_BASE                    0xE000ED00UL
#define NVIC_STIR_BASE              0xE000EF00UL
#define FPU_CPACR_BASE              0xE000ED88UL
#define SCB_ACTRL_BASE              0xE000E008UL

/** @} */ /* End of group Device_Peripheral_peripheralAddr */


/* =========================================================================================================================== */
/* ================                                  Peripheral declaration                                   ================ */
/* =========================================================================================================================== */


/** @addtogroup Device_Peripheral_declaration
  * @{
  */

#define ADC_Common                  ((ADC_Common_Type*)        ADC_Common_BASE)
#define ADC1                        ((ADC1_Type*)              ADC1_BASE)
#define CRC                         ((CRC_Type*)               CRC_BASE)
#define DBG                         ((DBG_Type*)               DBG_BASE)
#define EXTI                        ((EXTI_Type*)              EXTI_BASE)
#define FLASH                       ((FLASH_Type*)             FLASH_BASE)
#define IWDG                        ((IWDG_Type*)              IWDG_BASE)
#define OTG_FS_DEVICE               ((OTG_FS_DEVICE_Type*)     OTG_FS_DEVICE_BASE)
#define OTG_FS_GLOBAL               ((OTG_FS_GLOBAL_Type*)     OTG_FS_GLOBAL_BASE)
#define OTG_FS_HOST                 ((OTG_FS_HOST_Type*)       OTG_FS_HOST_BASE)
#define OTG_FS_PWRCLK               ((OTG_FS_PWRCLK_Type*)     OTG_FS_PWRCLK_BASE)
#define PWR                         ((PWR_Type*)               PWR_BASE)
#define RCC                         ((RCC_Type*)               RCC_BASE)
#define RTC                         ((RTC_Type*)               RTC_BASE)
#define SDIO                        ((SDIO_Type*)              SDIO_BASE)
#define SYSCFG                      ((SYSCFG_Type*)            SYSCFG_BASE)
#define TIM1                        ((TIM1_Type*)              TIM1_BASE)
#define TIM8                        ((TIM1_Type*)              TIM8_BASE)
#define TIM10                       ((TIM10_Type*)             TIM10_BASE)
#define TIM11                       ((TIM11_Type*)             TIM11_BASE)
#define TIM2                        ((TIM2_Type*)              TIM2_BASE)
#define TIM3                        ((TIM3_Type*)              TIM3_BASE)
#define TIM4                        ((TIM3_Type*)              TIM4_BASE)
#define TIM5                        ((TIM5_Type*)              TIM5_BASE)
#define TIM9                        ((TIM9_Type*)              TIM9_BASE)
#define USART1                      ((USART1_Type*)            USART1_BASE)
#define USART2                      ((USART1_Type*)            USART2_BASE)
#define USART6                      ((USART1_Type*)            USART6_BASE)
#define WWDG                        ((WWDG_Type*)              WWDG_BASE)
#define DMA2                        ((DMA2_Type*)              DMA2_BASE)
#define DMA1                        ((DMA2_Type*)              DMA1_BASE)
#define GPIOH                       ((GPIOH_Type*)             GPIOH_BASE)
#define GPIOE                       ((GPIOH_Type*)             GPIOE_BASE)
#define GPIOD                       ((GPIOH_Type*)             GPIOD_BASE)
#define GPIOC                       ((GPIOH_Type*)             GPIOC_BASE)
#define GPIOB                       ((GPIOB_Type*)             GPIOB_BASE)
#define GPIOA                       ((GPIOA_Type*)             GPIOA_BASE)
#define I2C3                        ((I2C3_Type*)              I2C3_BASE)
#define I2C2                        ((I2C3_Type*)              I2C2_BASE)
#define I2C1                        ((I2C3_Type*)              I2C1_BASE)
#define I2S2ext                     ((I2S2ext_Type*)           I2S2ext_BASE)
#define I2S3ext                     ((I2S2ext_Type*)           I2S3ext_BASE)
#define SPI1                        ((I2S2ext_Type*)           SPI1_BASE)
#define SPI2                        ((I2S2ext_Type*)           SPI2_BASE)
#define SPI3                        ((I2S2ext_Type*)           SPI3_BASE)
#define SPI4                        ((I2S2ext_Type*)           SPI4_BASE)
#define SPI5                        ((I2S2ext_Type*)           SPI5_BASE)
#define NVIC                        ((NVIC_Type*)              NVIC_BASE)
#define FPU                         ((FPU_Type*)               FPU_BASE)
#define MPU                         ((MPU_Type*)               MPU_BASE)
#define STK                         ((STK_Type*)               STK_BASE)
#define SCB                         ((SCB_Type*)               SCB_BASE)
#define NVIC_STIR                   ((NVIC_STIR_Type*)         NVIC_STIR_BASE)
#define FPU_CPACR                   ((FPU_CPACR_Type*)         FPU_CPACR_BASE)
#define SCB_ACTRL                   ((SCB_ACTRL_Type*)         SCB_ACTRL_BASE)

/** @} */ /* End of group Device_Peripheral_declaration */


#ifdef __cplusplus
}
#endif

#endif /* STM32F411_H */


/** @} */ /* End of group STM32F411 */

/** @} */ /* End of group  */
