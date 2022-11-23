/******************************************************************************************
  Filename    : IntVect.c
  
  Core        : ARM Cortex®-M7
  
  MCU         : STM32F746ZGT6
    
  Author      : Chalandi Amine
 
  Owner       : Chalandi Amine
  
  Date        : 22.11.2022
  
  Description : Interrupt Vector Table for STM32F746ZGT6
  
******************************************************************************************/
//=============================================================================
// Linker symbols
//=============================================================================
extern void __SYS_STACK_TOP(void);

//=============================================================================
// Extern function prototype
//=============================================================================
extern void Startup_Init(void);

//=============================================================================
// Functions prototype
//=============================================================================
static void UndefinedHandler(void);

void Isr_NonMaskableInt_IRQn     (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_HardFault_IRQn          (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_MemoryManagement_IRQn   (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_BusFault_IRQn           (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_UsageFault_IRQn         (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_SVCall_IRQn             (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_DebugMonitor_IRQn       (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_PendSV_IRQn             (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_SysTick_IRQn            (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_WWDG_IRQn               (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_PVD_IRQn                (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_TAMP_STAMP_IRQn         (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_RTC_WKUP_IRQn           (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_FLASH_IRQn              (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_RCC_IRQn                (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_EXTI0_IRQn              (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_EXTI1_IRQn              (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_EXTI2_IRQn              (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_EXTI3_IRQn              (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_EXTI4_IRQn              (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_DMA1_Stream0_IRQn       (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_DMA1_Stream1_IRQn       (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_DMA1_Stream2_IRQn       (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_DMA1_Stream3_IRQn       (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_DMA1_Stream4_IRQn       (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_DMA1_Stream5_IRQn       (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_DMA1_Stream6_IRQn       (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_ADC_IRQn                (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_CAN1_TX_IRQn            (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_CAN1_RX0_IRQn           (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_CAN1_RX1_IRQn           (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_CAN1_SCE_IRQn           (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_EXTI9_5_IRQn            (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_TIM1_BRK_TIM9_IRQn      (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_TIM1_UP_TIM10_IRQn      (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_TIM1_TRG_COM_TIM11_IRQn (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_TIM1_CC_IRQn            (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_TIM2_IRQn               (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_TIM3_IRQn               (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_TIM4_IRQn               (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_I2C1_EV_IRQn            (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_I2C1_ER_IRQn            (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_I2C2_EV_IRQn            (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_I2C2_ER_IRQn            (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_SPI1_IRQn               (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_SPI2_IRQn               (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_USART1_IRQn             (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_USART2_IRQn             (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_USART3_IRQn             (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_EXTI15_10_IRQn          (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_RTC_ALARM_IRQn          (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_OTG_FS_WKUP_IRQn        (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_TIM8_BRK_TIM12_IRQn     (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_TIM8_UP_TIM13_IRQn      (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_TIM8_TRG_COM_TIM14_IRQn (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_TIM8_CC_IRQn            (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_DMA1_Stream7_IRQn       (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_FMC_IRQn                (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_SDMMC1_IRQn             (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_TIM5_IRQn               (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_SPI3_IRQn               (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_UART4_IRQn              (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_UART5_IRQn              (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_TIM6_DAC_IRQn           (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_TIM7_IRQn               (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_DMA2_Stream0_IRQn       (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_DMA2_Stream1_IRQn       (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_DMA2_Stream2_IRQn       (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_DMA2_Stream3_IRQn       (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_DMA2_Stream4_IRQn       (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_ETH_IRQn                (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_ETH_WKUP_IRQn           (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_CAN2_TX_IRQn            (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_CAN2_RX0_IRQn           (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_CAN2_RX1_IRQn           (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_CAN2_SCE_IRQn           (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_OTG_FS_IRQn             (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_DMA2_Stream5_IRQn       (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_DMA2_Stream6_IRQn       (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_DMA2_Stream7_IRQn       (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_USART6_IRQn             (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_I2C3_EV_IRQn            (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_I2C3_ER_IRQn            (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_OTG_HS_EP1_OUT_IRQn     (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_OTG_HS_EP1_IN_IRQn      (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_OTG_HS_WKUP_IRQn        (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_OTG_HS_IRQn             (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_DCMI_IRQn               (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_CRYP_IRQn               (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_HASH_RNG_IRQn           (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_FPU_IRQn                (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_UART7_IRQn              (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_UART8_IRQn              (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_SPI4_IRQn               (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_SPI5_IRQn               (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_SPI6_IRQn               (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_SAI1_IRQn               (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_LCD_TFT_IRQn            (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_LTDC_ER_IRQn            (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_DMA2D_IRQn              (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_SAI2_IRQn               (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_QuadSPI_IRQn            (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_LPTimer1_IRQn           (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_HDMI_CEC_IRQn           (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_I2C4_EV_IRQn            (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_I2C4_ER_IRQn            (void) __attribute__((weak, alias("UndefinedHandler")));
void Isr_SPDIFRX_IRQn            (void) __attribute__((weak, alias("UndefinedHandler")));

//=============================================================================
// Types definition
//=============================================================================
typedef void (*InterruptHandler)(void);

//=============================================================================
// Local function
//=============================================================================
static void UndefinedHandler(void)
{
  for(;;);
}

//=============================================================================
// Interrupt vector table
//=============================================================================
const InterruptHandler __attribute__((section (".intvect"), aligned(512))) InterruptVectorTable[] =
{
    (InterruptHandler)__SYS_STACK_TOP,                 /* Stack pointer                                                                   */
    (InterruptHandler)&Startup_Init,                   /* Reset Vector, invoked on Power up and warm reset                                */
    (InterruptHandler)&Isr_NonMaskableInt_IRQn,        /* Non maskable Interrupt, cannot be stopped or preempted                          */
    (InterruptHandler)&Isr_HardFault_IRQn,             /* Hard Fault, all classes of Fault                                                */
    (InterruptHandler)&Isr_MemoryManagement_IRQn,      /* Memory Management, MPU mismatch, including Access Violation and No Match        */
    (InterruptHandler)&Isr_BusFault_IRQn,              /* Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory related Fault  */
    (InterruptHandler)&Isr_UsageFault_IRQn,            /* Usage Fault, i.e. Undef Instruction, Illegal State Transition                   */
    (InterruptHandler)&UndefinedHandler,               /* Reserved                                                                        */
    (InterruptHandler)&UndefinedHandler,               /* Reserved                                                                        */
    (InterruptHandler)&UndefinedHandler,               /* Reserved                                                                        */
    (InterruptHandler)&UndefinedHandler,               /* Reserved                                                                        */
    (InterruptHandler)&Isr_SVCall_IRQn,                /* System Service Call via SVC instruction                                         */
    (InterruptHandler)&UndefinedHandler,               /* Reserved                                                                        */
    (InterruptHandler)&UndefinedHandler,               /* Reserved                                                                        */
    (InterruptHandler)&Isr_PendSV_IRQn,                /* Pendable request for system service                                             */
    (InterruptHandler)&Isr_SysTick_IRQn,               /* System Tick Timer                                                               */
    (InterruptHandler)&Isr_WWDG_IRQn,                  /* Window Watchdog interrupt                                                       */
    (InterruptHandler)&Isr_PVD_IRQn,                   /* PVD through EXTI line detection INTERRUPT                                       */
    (InterruptHandler)&Isr_TAMP_STAMP_IRQn,            /* Tamper and TimeStamp interrupts through the EXTI line                           */
    (InterruptHandler)&Isr_RTC_WKUP_IRQn,              /* RTC Tamper or TimeStamp /CSS on LSE through EXTI line 19 interrupts             */
    (InterruptHandler)&Isr_FLASH_IRQn,                 /* Flash global interrupt                                                          */
    (InterruptHandler)&Isr_RCC_IRQn,                   /* RCC global interrupt                                                            */
    (InterruptHandler)&Isr_EXTI0_IRQn,                 /* EXTI Line0 interrupt                                                            */
    (InterruptHandler)&Isr_EXTI1_IRQn,                 /* EXTI Line1 interrupt                                                            */
    (InterruptHandler)&Isr_EXTI2_IRQn,                 /* EXTI Line2 interrupt                                                            */
    (InterruptHandler)&Isr_EXTI3_IRQn,                 /* EXTI Line3 interrupt                                                            */
    (InterruptHandler)&Isr_EXTI4_IRQn,                 /* EXTI Line4 interrupt                                                            */
    (InterruptHandler)&Isr_DMA1_Stream0_IRQn,          /* DMA1 Stream0 global interrupt                                                   */
    (InterruptHandler)&Isr_DMA1_Stream1_IRQn,          /* DMA1 Stream1 global interrupt                                                   */
    (InterruptHandler)&Isr_DMA1_Stream2_IRQn,          /* DMA1 Stream2 global interrupt                                                   */
    (InterruptHandler)&Isr_DMA1_Stream3_IRQn,          /* DMA1 Stream3 global interrupt                                                   */
    (InterruptHandler)&Isr_DMA1_Stream4_IRQn,          /* DMA1 Stream4 global interrupt                                                   */
    (InterruptHandler)&Isr_DMA1_Stream5_IRQn,          /* DMA1 Stream5 global interrupt                                                   */
    (InterruptHandler)&Isr_DMA1_Stream6_IRQn,          /* DMA1 Stream6 global interrupt                                                   */
    (InterruptHandler)&Isr_ADC_IRQn,                   /* ADC1 global interrupt                                                           */
    (InterruptHandler)&Isr_CAN1_TX_IRQn,               /* CAN1 TX interrupts                                                              */
    (InterruptHandler)&Isr_CAN1_RX0_IRQn,              /* CAN1 RX0 interrupts                                                             */
    (InterruptHandler)&Isr_CAN1_RX1_IRQn,              /* CAN1 RX1 interrupts                                                             */
    (InterruptHandler)&Isr_CAN1_SCE_IRQn,              /* CAN1 SCE interrupt                                                              */
    (InterruptHandler)&Isr_EXTI9_5_IRQn,               /* EXTI Line[9:5] interrupts                                                       */
    (InterruptHandler)&Isr_TIM1_BRK_TIM9_IRQn,         /* TIM1 Break interrupt and TIM9 global interrupt                                  */
    (InterruptHandler)&Isr_TIM1_UP_TIM10_IRQn,         /* TIM1 Update interrupt and TIM10                                                 */
    (InterruptHandler)&Isr_TIM1_TRG_COM_TIM11_IRQn,    /* TIM1 Trigger and Commutation interrupts and TIM11 global interrupt              */
    (InterruptHandler)&Isr_TIM1_CC_IRQn,               /* TIM1 Capture Compare interrupt                                                  */
    (InterruptHandler)&Isr_TIM2_IRQn,                  /* TIM2 global interrupt                                                           */
    (InterruptHandler)&Isr_TIM3_IRQn,                  /* TIM3 global interrupt                                                           */
    (InterruptHandler)&Isr_TIM4_IRQn,                  /* TIM4 global interrupt                                                           */
    (InterruptHandler)&Isr_I2C1_EV_IRQn,               /* I2C1 event interrupt                                                            */
    (InterruptHandler)&Isr_I2C1_ER_IRQn,               /* I2C1 error interrupt                                                            */
    (InterruptHandler)&Isr_I2C2_EV_IRQn,               /* I2C2 event interrupt                                                            */
    (InterruptHandler)&Isr_I2C2_ER_IRQn,               /* I2C2 error interrupt                                                            */
    (InterruptHandler)&Isr_SPI1_IRQn,                  /* SPI1 global interrupt                                                           */
    (InterruptHandler)&Isr_SPI2_IRQn,                  /* SPI2 global interrupt                                                           */
    (InterruptHandler)&Isr_USART1_IRQn,                /* USART1 global interrupt                                                         */
    (InterruptHandler)&Isr_USART2_IRQn,                /* USART2 global interrupt                                                         */
    (InterruptHandler)&Isr_USART3_IRQn,                /* USART3 global interrupt                                                         */
    (InterruptHandler)&Isr_EXTI15_10_IRQn,             /* EXTI Line[15:10] interrupts                                                     */
    (InterruptHandler)&Isr_RTC_ALARM_IRQn,             /* RTC alarms through EXTI line 18 interrupts                                      */
    (InterruptHandler)&Isr_OTG_FS_WKUP_IRQn,           /* USB On-The-Go FS Wakeup through EXTI line interrupt                             */
    (InterruptHandler)&Isr_TIM8_BRK_TIM12_IRQn,        /* TIM8 Break interrupt and TIM12 global interrupt                                 */
    (InterruptHandler)&Isr_TIM8_UP_TIM13_IRQn,         /* TIM8 Update interrupt and TIM13 global interrupt                                */
    (InterruptHandler)&Isr_TIM8_TRG_COM_TIM14_IRQn,    /* TIM8 Trigger and Commutation interrupts and TIM14 global interrupt              */
    (InterruptHandler)&Isr_TIM8_CC_IRQn,               /* TIM8 Capture Compare interrupt                                                  */
    (InterruptHandler)&Isr_DMA1_Stream7_IRQn,          /* DMA1 Stream7 global interrupt                                                   */
    (InterruptHandler)&Isr_FMC_IRQn,                   /* FMC global interrupt                                                            */
    (InterruptHandler)&Isr_SDMMC1_IRQn,                /* SDMMC1 global interrupt                                                         */
    (InterruptHandler)&Isr_TIM5_IRQn,                  /* TIM5 global interrupt                                                           */
    (InterruptHandler)&Isr_SPI3_IRQn,                  /* SPI3 global interrupt                                                           */
    (InterruptHandler)&Isr_UART4_IRQn,                 /* UART4 global interrupt                                                          */
    (InterruptHandler)&Isr_UART5_IRQn,                 /* UART5 global interrupt                                                          */
    (InterruptHandler)&Isr_TIM6_DAC_IRQn,              /* TIM6 global interrupt, DAC1 and DAC2 underrun error interrupt                   */
    (InterruptHandler)&Isr_TIM7_IRQn,                  /* TIM7 global interrupt                                                           */
    (InterruptHandler)&Isr_DMA2_Stream0_IRQn,          /* DMA2 Stream0 global interrupt                                                   */
    (InterruptHandler)&Isr_DMA2_Stream1_IRQn,          /* DMA2 Stream1 global interrupt                                                   */
    (InterruptHandler)&Isr_DMA2_Stream2_IRQn,          /* DMA2 Stream2 global interrupt                                                   */
    (InterruptHandler)&Isr_DMA2_Stream3_IRQn,          /* DMA2 Stream3 global interrupt                                                   */
    (InterruptHandler)&Isr_DMA2_Stream4_IRQn,          /* DMA2 Stream4 global interrupt                                                   */
    (InterruptHandler)&Isr_ETH_IRQn,                   /* Ethernet global interrupt                                                       */
    (InterruptHandler)&Isr_ETH_WKUP_IRQn,              /* Ethernet Wakeup through EXTI line interrupt                                     */
    (InterruptHandler)&Isr_CAN2_TX_IRQn,               /* CAN2 TX interrupts                                                              */
    (InterruptHandler)&Isr_CAN2_RX0_IRQn,              /* CAN2 RX0 interrupts                                                             */
    (InterruptHandler)&Isr_CAN2_RX1_IRQn,              /* CAN2 RX1 interrupts                                                             */
    (InterruptHandler)&Isr_CAN2_SCE_IRQn,              /* CAN2 SCE interrupt                                                              */
    (InterruptHandler)&Isr_OTG_FS_IRQn,                /* USB On The Go FS global interrupt                                               */
    (InterruptHandler)&Isr_DMA2_Stream5_IRQn,          /* DMA2 Stream5 global interrupt                                                   */
    (InterruptHandler)&Isr_DMA2_Stream6_IRQn,          /* DMA2 Stream6 global interrupt                                                   */
    (InterruptHandler)&Isr_DMA2_Stream7_IRQn,          /* DMA2 Stream7 global interrupt                                                   */
    (InterruptHandler)&Isr_USART6_IRQn,                /* USART6 global interrupt                                                         */
    (InterruptHandler)&Isr_I2C3_EV_IRQn,               /* I2C3 event interrupt                                                            */
    (InterruptHandler)&Isr_I2C3_ER_IRQn,               /* I2C3 error interrupt                                                            */
    (InterruptHandler)&Isr_OTG_HS_EP1_OUT_IRQn,        /* USB On The Go HS End Point 1 Out global interrupt                               */
    (InterruptHandler)&Isr_OTG_HS_EP1_IN_IRQn,         /* USB On The Go HS End Point 1 In global interrupt                                */
    (InterruptHandler)&Isr_OTG_HS_WKUP_IRQn,           /* USB On The Go HS Wakeup through EXTI interrupt                                  */
    (InterruptHandler)&Isr_OTG_HS_IRQn,                /* USB On The Go HS global interrupt                                               */
    (InterruptHandler)&Isr_DCMI_IRQn,                  /* DCMI global interrupt                                                           */
    (InterruptHandler)&Isr_CRYP_IRQn,                  /* CRYP crypto global interrupt                                                    */
    (InterruptHandler)&Isr_HASH_RNG_IRQn,              /* Hash and Rng global interrupt                                                   */
    (InterruptHandler)&Isr_FPU_IRQn,                   /* Floating point unit interrupt                                                   */
    (InterruptHandler)&Isr_UART7_IRQn,                 /* UART7 global interrupt                                                          */
    (InterruptHandler)&Isr_UART8_IRQn,                 /* UART 8 global interrupt                                                         */
    (InterruptHandler)&Isr_SPI4_IRQn,                  /* SPI 4 global interrupt                                                          */
    (InterruptHandler)&Isr_SPI5_IRQn,                  /* SPI 5 global interrupt                                                          */
    (InterruptHandler)&Isr_SPI6_IRQn,                  /* SPI 6 global interrupt                                                          */
    (InterruptHandler)&Isr_SAI1_IRQn,                  /* SAI1 global interrupt                                                           */
    (InterruptHandler)&Isr_LCD_TFT_IRQn,               /* LTDC global interrupt                                                           */
    (InterruptHandler)&Isr_LTDC_ER_IRQn,               /* LTDC Error global interrupt                                                     */
    (InterruptHandler)&Isr_DMA2D_IRQn,                 /* DMA2D global interrupt                                                          */
    (InterruptHandler)&Isr_SAI2_IRQn,                  /* SAI2 global interrupt                                                           */
    (InterruptHandler)&Isr_QuadSPI_IRQn,               /* QuadSPI global interrupt                                                        */
    (InterruptHandler)&Isr_LPTimer1_IRQn,              /* LP Timer1 global interrupt                                                      */
    (InterruptHandler)&Isr_HDMI_CEC_IRQn,              /* HDMI-CEC global interrupt                                                       */
    (InterruptHandler)&Isr_I2C4_EV_IRQn,               /* I2C4 event interrupt                                                            */
    (InterruptHandler)&Isr_I2C4_ER_IRQn,               /* I2C4 Error interrupt                                                            */
    (InterruptHandler)&Isr_SPDIFRX_IRQn                /* SPDIFRX global interrupt                                                        */
};