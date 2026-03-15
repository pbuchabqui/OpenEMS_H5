/**
 * @file startup_stm32h562.s
 * @brief Código de startup e tabela de vetores para STM32H562RGT6
 *
 * Responsabilidades:
 *   1. Tabela de vetores de interrupção (posicionada via .isr_vector)
 *   2. Reset_Handler: inicializa C runtime (.bss zerado, .data copiado)
 *      e chama main()
 *   3. Default_Handler: loop infinito para ISRs não implementadas
 *      (IWDG reseta o sistema em 100 ms)
 *
 * Posicionamento dos IRQs (RM0481 §Table 87, confirmado via
 * cmsis-device-h5/stm32h562xx.h):
 *   IRQ 37 = ADC1_2        (ADC1_2_IRQHandler)
 *   IRQ 44 = TIM1_CC       (TIM1_CC_IRQHandler)
 *   IRQ 48 = TIM5          (TIM5_IRQHandler)
 *   Todos os demais → Default_Handler (weak)
 *
 * SysTick não está na tabela de IRQs — é exceção ARM core (posição 15).
 */

    .syntax unified
    .cpu cortex-m33
    .fpu fpv5-sp-d16
    .thumb

/* ── Tabela de vetores ────────────────────────────────────────────────────── */
/* Colocada na seção .isr_vector → mapeada para início da Flash Bank1          */
/* (0x08000000) pelo linker script. O boot ROM lê [0] como SP e [1] como PC.  */

    .section .isr_vector, "a", %progbits
    .type  g_pfnVectors, %object

g_pfnVectors:
    /* ── ARM Cortex-M33 core exceptions (posições 0–15) ── */
    .word  _estack                     /* 00: Stack pointer inicial        */
    .word  Reset_Handler               /* 01: Reset                        */
    .word  NMI_Handler                 /* 02: NMI                          */
    .word  HardFault_Handler           /* 03: Hard Fault                   */
    .word  MemManage_Handler           /* 04: MPU Fault (MemManage)        */
    .word  BusFault_Handler            /* 05: Bus Fault                    */
    .word  UsageFault_Handler          /* 06: Usage Fault                  */
    .word  SecureFault_Handler         /* 07: Secure Fault (TrustZone)     */
    .word  0                           /* 08: reservado                    */
    .word  0                           /* 09: reservado                    */
    .word  0                           /* 10: reservado                    */
    .word  SVC_Handler                 /* 11: SVCall                       */
    .word  DebugMon_Handler            /* 12: Debug Monitor                */
    .word  0                           /* 13: reservado                    */
    .word  PendSV_Handler              /* 14: PendSV                       */
    .word  SysTick_Handler             /* 15: SysTick (1 ms)               */

    /* ── STM32H562 IRQs (posições 16..163 = IRQ 0..147) ── */
    /* Fonte: RM0481 Table 87 + stm32h562xx.h (STM32CubeH5)               */
    .word  WWDG_IRQHandler             /* IRQ  0: WWDG                     */
    .word  PVD_AVD_IRQHandler          /* IRQ  1: PVD/AVD (EXTI 16)        */
    .word  RTC_IRQHandler              /* IRQ  2: RTC (tamper + wakeup)    */
    .word  RTC_S_IRQHandler            /* IRQ  3: RTC Secure               */
    .word  TAMP_IRQHandler             /* IRQ  4: Tamper                   */
    .word  RAMCFG_IRQHandler           /* IRQ  5: RAMCFG                   */
    .word  FLASH_IRQHandler            /* IRQ  6: Flash global             */
    .word  FLASH_S_IRQHandler          /* IRQ  7: Flash Secure             */
    .word  GTZC_IRQHandler             /* IRQ  8: GTZC global              */
    .word  RCC_IRQHandler              /* IRQ  9: RCC global               */
    .word  RCC_S_IRQHandler            /* IRQ 10: RCC Secure               */
    .word  EXTI0_IRQHandler            /* IRQ 11: EXTI line 0              */
    .word  EXTI1_IRQHandler            /* IRQ 12: EXTI line 1              */
    .word  EXTI2_IRQHandler            /* IRQ 13: EXTI line 2              */
    .word  EXTI3_IRQHandler            /* IRQ 14: EXTI line 3              */
    .word  EXTI4_IRQHandler            /* IRQ 15: EXTI line 4              */
    .word  EXTI5_IRQHandler            /* IRQ 16: EXTI line 5              */
    .word  EXTI6_IRQHandler            /* IRQ 17: EXTI line 6              */
    .word  EXTI7_IRQHandler            /* IRQ 18: EXTI line 7              */
    .word  EXTI8_IRQHandler            /* IRQ 19: EXTI line 8              */
    .word  EXTI9_IRQHandler            /* IRQ 20: EXTI line 9              */
    .word  EXTI10_IRQHandler           /* IRQ 21: EXTI line 10             */
    .word  EXTI11_IRQHandler           /* IRQ 22: EXTI line 11             */
    .word  EXTI12_IRQHandler           /* IRQ 23: EXTI line 12             */
    .word  EXTI13_IRQHandler           /* IRQ 24: EXTI line 13             */
    .word  EXTI14_IRQHandler           /* IRQ 25: EXTI line 14             */
    .word  EXTI15_IRQHandler           /* IRQ 26: EXTI line 15             */
    .word  GPDMA1_Channel0_IRQHandler  /* IRQ 27: GPDMA1 Channel 0        */
    .word  GPDMA1_Channel1_IRQHandler  /* IRQ 28: GPDMA1 Channel 1        */
    .word  GPDMA1_Channel2_IRQHandler  /* IRQ 29: GPDMA1 Channel 2        */
    .word  GPDMA1_Channel3_IRQHandler  /* IRQ 30: GPDMA1 Channel 3        */
    .word  GPDMA1_Channel4_IRQHandler  /* IRQ 31: GPDMA1 Channel 4        */
    .word  GPDMA1_Channel5_IRQHandler  /* IRQ 32: GPDMA1 Channel 5        */
    .word  GPDMA1_Channel6_IRQHandler  /* IRQ 33: GPDMA1 Channel 6        */
    .word  GPDMA1_Channel7_IRQHandler  /* IRQ 34: GPDMA1 Channel 7        */
    .word  IWDG_IRQHandler             /* IRQ 35: IWDG global              */
    .word  SAES_IRQHandler             /* IRQ 36: SAES                     */
    .word  ADC1_2_IRQHandler           /* IRQ 37: ADC1/ADC2 ← usados!     */
    .word  DAC1_IRQHandler             /* IRQ 38: DAC1                     */
    .word  FDCAN1_IT0_IRQHandler       /* IRQ 39: FDCAN1 interrupt 0       */
    .word  FDCAN1_IT1_IRQHandler       /* IRQ 40: FDCAN1 interrupt 1       */
    .word  FDCAN2_IT0_IRQHandler       /* IRQ 41: FDCAN2 interrupt 0       */
    .word  FDCAN2_IT1_IRQHandler       /* IRQ 42: FDCAN2 interrupt 1       */
    .word  TIM1_BRK_IRQHandler         /* IRQ 43: TIM1 Break               */
    .word  TIM1_CC_IRQHandler          /* IRQ 44: TIM1 CC ← ignição!       */
    .word  TIM1_TRG_COM_IRQHandler     /* IRQ 45: TIM1 Trigger/Comm        */
    .word  TIM1_UP_IRQHandler          /* IRQ 46: TIM1 Update              */
    .word  TIM2_IRQHandler             /* IRQ 47: TIM2                     */
    .word  TIM5_IRQHandler             /* IRQ 48: TIM5 ← CKP!             */
    .word  TIM6_IRQHandler             /* IRQ 49: TIM6 (ADC trigger)       */
    .word  TIM7_IRQHandler             /* IRQ 50: TIM7                     */
    .word  I2C1_EV_IRQHandler          /* IRQ 51: I2C1 Event               */
    .word  I2C1_ER_IRQHandler          /* IRQ 52: I2C1 Error               */
    .word  I2C2_EV_IRQHandler          /* IRQ 53: I2C2 Event               */
    .word  I2C2_ER_IRQHandler          /* IRQ 54: I2C2 Error               */
    .word  SPI1_IRQHandler             /* IRQ 55: SPI1                     */
    .word  SPI2_IRQHandler             /* IRQ 56: SPI2                     */
    .word  SPI3_IRQHandler             /* IRQ 57: SPI3                     */
    .word  USART1_IRQHandler           /* IRQ 58: USART1                   */
    .word  USART2_IRQHandler           /* IRQ 59: USART2                   */
    .word  USART3_IRQHandler           /* IRQ 60: USART3 ← TunerStudio     */
    .word  UART4_IRQHandler            /* IRQ 61: UART4                    */
    .word  UART5_IRQHandler            /* IRQ 62: UART5                    */
    .word  UART7_IRQHandler            /* IRQ 63: UART7                    */
    .word  UART8_IRQHandler            /* IRQ 64: UART8                    */
    .word  I2C3_EV_IRQHandler          /* IRQ 65: I2C3 Event               */
    .word  I2C3_ER_IRQHandler          /* IRQ 66: I2C3 Error               */
    .word  OTG_FS_IRQHandler           /* IRQ 67: OTG FS                   */
    .word  ETH_IRQHandler              /* IRQ 68: Ethernet                 */
    .word  CORDIC_IRQHandler           /* IRQ 69: CORDIC                   */
    .word  UART9_IRQHandler            /* IRQ 70: UART9                    */
    .word  USART6_IRQHandler           /* IRQ 71: USART6                   */
    .word  USART10_IRQHandler          /* IRQ 72: USART10                  */
    .word  USART11_IRQHandler          /* IRQ 73: USART11                  */
    .word  UART12_IRQHandler           /* IRQ 74: UART12                   */
    .word  I3C1_EV_IRQHandler          /* IRQ 75: I3C1 Event               */
    .word  I3C1_ER_IRQHandler          /* IRQ 76: I3C1 Error               */
    .word  DTS_IRQHandler              /* IRQ 77: DTS (Die Temp Sensor)    */
    .word  MCE1_IRQHandler             /* IRQ 78: MCE1                     */
    .word  MCE2_IRQHandler             /* IRQ 79: MCE2                     */
    .word  MCE3_IRQHandler             /* IRQ 80: MCE3                     */
    .word  LPTIM1_IRQHandler           /* IRQ 81: LPTIM1                   */
    .word  LPTIM2_IRQHandler           /* IRQ 82: LPTIM2                   */
    .word  LPTIM3_IRQHandler           /* IRQ 83: LPTIM3                   */
    .word  LPTIM4_IRQHandler           /* IRQ 84: LPTIM4                   */
    .word  LPTIM5_IRQHandler           /* IRQ 85: LPTIM5                   */
    .word  LPTIM6_IRQHandler           /* IRQ 86: LPTIM6                   */
    .word  VREFBUF_IRQHandler          /* IRQ 87: VREFBUF                  */
    .word  I2C4_EV_IRQHandler          /* IRQ 88: I2C4 Event               */
    .word  I2C4_ER_IRQHandler          /* IRQ 89: I2C4 Error               */
    .word  SPI4_IRQHandler             /* IRQ 90: SPI4                     */
    .word  SPI5_IRQHandler             /* IRQ 91: SPI5                     */
    .word  SPI6_IRQHandler             /* IRQ 92: SPI6                     */
    .word  SAI1_IRQHandler             /* IRQ 93: SAI1                     */
    .word  SAI2_IRQHandler             /* IRQ 94: SAI2                     */
    .word  TIM3_IRQHandler             /* IRQ 95: TIM3 ← PWM IACV/waste    */
    .word  TIM4_IRQHandler             /* IRQ 96: TIM4 ← PWM VVT           */
    .word  TIM8_BRK_IRQHandler         /* IRQ 97: TIM8 Break               */
    .word  TIM8_UP_IRQHandler          /* IRQ 98: TIM8 Update              */
    .word  TIM8_TRG_COM_IRQHandler     /* IRQ 99: TIM8 Trigger/Comm        */
    .word  TIM8_CC_IRQHandler          /* IRQ 100: TIM8 CC                 */
    .word  TIM15_IRQHandler            /* IRQ 101: TIM15                   */
    .word  TIM16_IRQHandler            /* IRQ 102: TIM16                   */
    .word  TIM17_IRQHandler            /* IRQ 103: TIM17                   */
    .word  Default_Handler             /* IRQ 104: reservado               */
    .word  Default_Handler             /* IRQ 105: reservado               */
    .word  Default_Handler             /* IRQ 106: reservado               */
    .word  Default_Handler             /* IRQ 107: reservado               */
    .word  Default_Handler             /* IRQ 108: reservado               */
    .word  HASH_IRQHandler             /* IRQ 109: HASH                    */
    .word  Default_Handler             /* IRQ 110: reservado               */
    .word  CRYP_IRQHandler             /* IRQ 111: CRYP                    */
    .word  PKA_IRQHandler              /* IRQ 112: PKA                     */
    .word  Default_Handler             /* IRQ 113: reservado               */
    .word  Default_Handler             /* IRQ 114: reservado               */
    .word  Default_Handler             /* IRQ 115: reservado               */
    .word  Default_Handler             /* IRQ 116: reservado               */
    .word  Default_Handler             /* IRQ 117: reservado               */
    .word  Default_Handler             /* IRQ 118: reservado               */
    .word  Default_Handler             /* IRQ 119: reservado               */
    .word  Default_Handler             /* IRQ 120: reservado               */
    .word  Default_Handler             /* IRQ 121: reservado               */
    .word  Default_Handler             /* IRQ 122: reservado               */
    .word  Default_Handler             /* IRQ 123: reservado               */
    .word  Default_Handler             /* IRQ 124: reservado               */
    .word  Default_Handler             /* IRQ 125: reservado               */
    .word  Default_Handler             /* IRQ 126: reservado               */
    .word  Default_Handler             /* IRQ 127: reservado               */
    .word  Default_Handler             /* IRQ 128: reservado               */
    .word  Default_Handler             /* IRQ 129: reservado               */
    .word  Default_Handler             /* IRQ 130: reservado               */
    .word  Default_Handler             /* IRQ 131: reservado               */
    .word  Default_Handler             /* IRQ 132: reservado               */
    .word  Default_Handler             /* IRQ 133: reservado               */
    .word  Default_Handler             /* IRQ 134: reservado               */
    .word  Default_Handler             /* IRQ 135: reservado               */
    .word  Default_Handler             /* IRQ 136: reservado               */
    .word  Default_Handler             /* IRQ 137: reservado               */
    .word  Default_Handler             /* IRQ 138: reservado               */
    .word  Default_Handler             /* IRQ 139: reservado               */
    .word  Default_Handler             /* IRQ 140: reservado               */
    .word  Default_Handler             /* IRQ 141: reservado               */
    .word  Default_Handler             /* IRQ 142: reservado               */
    .word  Default_Handler             /* IRQ 143: reservado               */
    .word  Default_Handler             /* IRQ 144: reservado               */
    .word  Default_Handler             /* IRQ 145: reservado               */
    .word  Default_Handler             /* IRQ 146: reservado               */
    .word  Default_Handler             /* IRQ 147: reservado               */

    .size  g_pfnVectors, .-g_pfnVectors

/* ── Reset_Handler ────────────────────────────────────────────────────────── */
/* Executado imediatamente após reset. Inicializa C runtime antes de main().   */

    .section .text.Reset_Handler
    .weak  Reset_Handler
    .type  Reset_Handler, %function

Reset_Handler:
    /* 1. Zerar seção .bss (variáveis globais não inicializadas) */
    ldr   r0, =_sbss
    ldr   r1, =_ebss
    movs  r2, #0
.zero_bss:
    cmp   r0, r1
    bge   .copy_data
    str   r2, [r0], #4
    b     .zero_bss

    /* 2. Copiar .data de Flash (LMA = _sidata) para SRAM (VMA = _sdata) */
.copy_data:
    ldr   r0, =_sidata      /* fonte na Flash */
    ldr   r1, =_sdata       /* destino na SRAM */
    ldr   r2, =_edata
.copy_data_loop:
    cmp   r1, r2
    bge   .call_main
    ldr   r3, [r0], #4
    str   r3, [r1], #4
    b     .copy_data_loop

    /* 3. Chamar main() — não deve retornar; se retornar, loop infinito */
.call_main:
    bl    main
    b     .

    .size  Reset_Handler, .-Reset_Handler

/* ── Default_Handler ─────────────────────────────────────────────────────── */
/* Todas as ISRs não implementadas caem aqui. Loop infinito permite que o IWDG */
/* (100 ms) resete o sistema — mais seguro do que retornar de uma ISR inesperada. */

    .section .text.Default_Handler, "ax", %progbits

Default_Handler:
    b     Default_Handler

    .size  Default_Handler, .-Default_Handler

/* ── Weak aliases ─────────────────────────────────────────────────────────── */
/* Handlers marcados como .weak podem ser sobrescritos por implementações em   */
/* arquivos .cpp/.c sem precisar modificar este arquivo de startup.            */

    /* ARM core exceptions */
    .weak  NMI_Handler
    .thumb_set NMI_Handler, Default_Handler

    .weak  HardFault_Handler
    .thumb_set HardFault_Handler, Default_Handler

    .weak  MemManage_Handler
    .thumb_set MemManage_Handler, Default_Handler

    .weak  BusFault_Handler
    .thumb_set BusFault_Handler, Default_Handler

    .weak  UsageFault_Handler
    .thumb_set UsageFault_Handler, Default_Handler

    .weak  SecureFault_Handler
    .thumb_set SecureFault_Handler, Default_Handler

    .weak  SVC_Handler
    .thumb_set SVC_Handler, Default_Handler

    .weak  DebugMon_Handler
    .thumb_set DebugMon_Handler, Default_Handler

    .weak  PendSV_Handler
    .thumb_set PendSV_Handler, Default_Handler

    /* SysTick_Handler: implementado em system.cpp — NÃO weak aqui */
    /* para garantir que o linker use a implementação real             */
    .weak  SysTick_Handler
    .thumb_set SysTick_Handler, Default_Handler

    /* STM32H562 IRQ handlers usados pelo firmware */
    .weak  ADC1_2_IRQHandler
    .thumb_set ADC1_2_IRQHandler, Default_Handler

    .weak  TIM1_CC_IRQHandler
    .thumb_set TIM1_CC_IRQHandler, Default_Handler

    .weak  TIM5_IRQHandler
    .thumb_set TIM5_IRQHandler, Default_Handler

    .weak  USART3_IRQHandler
    .thumb_set USART3_IRQHandler, Default_Handler

    .weak  FDCAN1_IT0_IRQHandler
    .thumb_set FDCAN1_IT0_IRQHandler, Default_Handler

    .weak  FDCAN1_IT1_IRQHandler
    .thumb_set FDCAN1_IT1_IRQHandler, Default_Handler

    /* Handlers restantes (não usados pelo firmware, mas necessários na tabela) */
    .weak  WWDG_IRQHandler
    .thumb_set WWDG_IRQHandler, Default_Handler
    .weak  PVD_AVD_IRQHandler
    .thumb_set PVD_AVD_IRQHandler, Default_Handler
    .weak  RTC_IRQHandler
    .thumb_set RTC_IRQHandler, Default_Handler
    .weak  RTC_S_IRQHandler
    .thumb_set RTC_S_IRQHandler, Default_Handler
    .weak  TAMP_IRQHandler
    .thumb_set TAMP_IRQHandler, Default_Handler
    .weak  RAMCFG_IRQHandler
    .thumb_set RAMCFG_IRQHandler, Default_Handler
    .weak  FLASH_IRQHandler
    .thumb_set FLASH_IRQHandler, Default_Handler
    .weak  FLASH_S_IRQHandler
    .thumb_set FLASH_S_IRQHandler, Default_Handler
    .weak  GTZC_IRQHandler
    .thumb_set GTZC_IRQHandler, Default_Handler
    .weak  RCC_IRQHandler
    .thumb_set RCC_IRQHandler, Default_Handler
    .weak  RCC_S_IRQHandler
    .thumb_set RCC_S_IRQHandler, Default_Handler
    .weak  EXTI0_IRQHandler
    .thumb_set EXTI0_IRQHandler, Default_Handler
    .weak  EXTI1_IRQHandler
    .thumb_set EXTI1_IRQHandler, Default_Handler
    .weak  EXTI2_IRQHandler
    .thumb_set EXTI2_IRQHandler, Default_Handler
    .weak  EXTI3_IRQHandler
    .thumb_set EXTI3_IRQHandler, Default_Handler
    .weak  EXTI4_IRQHandler
    .thumb_set EXTI4_IRQHandler, Default_Handler
    .weak  EXTI5_IRQHandler
    .thumb_set EXTI5_IRQHandler, Default_Handler
    .weak  EXTI6_IRQHandler
    .thumb_set EXTI6_IRQHandler, Default_Handler
    .weak  EXTI7_IRQHandler
    .thumb_set EXTI7_IRQHandler, Default_Handler
    .weak  EXTI8_IRQHandler
    .thumb_set EXTI8_IRQHandler, Default_Handler
    .weak  EXTI9_IRQHandler
    .thumb_set EXTI9_IRQHandler, Default_Handler
    .weak  EXTI10_IRQHandler
    .thumb_set EXTI10_IRQHandler, Default_Handler
    .weak  EXTI11_IRQHandler
    .thumb_set EXTI11_IRQHandler, Default_Handler
    .weak  EXTI12_IRQHandler
    .thumb_set EXTI12_IRQHandler, Default_Handler
    .weak  EXTI13_IRQHandler
    .thumb_set EXTI13_IRQHandler, Default_Handler
    .weak  EXTI14_IRQHandler
    .thumb_set EXTI14_IRQHandler, Default_Handler
    .weak  EXTI15_IRQHandler
    .thumb_set EXTI15_IRQHandler, Default_Handler
    .weak  GPDMA1_Channel0_IRQHandler
    .thumb_set GPDMA1_Channel0_IRQHandler, Default_Handler
    .weak  GPDMA1_Channel1_IRQHandler
    .thumb_set GPDMA1_Channel1_IRQHandler, Default_Handler
    .weak  GPDMA1_Channel2_IRQHandler
    .thumb_set GPDMA1_Channel2_IRQHandler, Default_Handler
    .weak  GPDMA1_Channel3_IRQHandler
    .thumb_set GPDMA1_Channel3_IRQHandler, Default_Handler
    .weak  GPDMA1_Channel4_IRQHandler
    .thumb_set GPDMA1_Channel4_IRQHandler, Default_Handler
    .weak  GPDMA1_Channel5_IRQHandler
    .thumb_set GPDMA1_Channel5_IRQHandler, Default_Handler
    .weak  GPDMA1_Channel6_IRQHandler
    .thumb_set GPDMA1_Channel6_IRQHandler, Default_Handler
    .weak  GPDMA1_Channel7_IRQHandler
    .thumb_set GPDMA1_Channel7_IRQHandler, Default_Handler
    .weak  IWDG_IRQHandler
    .thumb_set IWDG_IRQHandler, Default_Handler
    .weak  SAES_IRQHandler
    .thumb_set SAES_IRQHandler, Default_Handler
    .weak  DAC1_IRQHandler
    .thumb_set DAC1_IRQHandler, Default_Handler
    .weak  FDCAN2_IT0_IRQHandler
    .thumb_set FDCAN2_IT0_IRQHandler, Default_Handler
    .weak  FDCAN2_IT1_IRQHandler
    .thumb_set FDCAN2_IT1_IRQHandler, Default_Handler
    .weak  TIM1_BRK_IRQHandler
    .thumb_set TIM1_BRK_IRQHandler, Default_Handler
    .weak  TIM1_TRG_COM_IRQHandler
    .thumb_set TIM1_TRG_COM_IRQHandler, Default_Handler
    .weak  TIM1_UP_IRQHandler
    .thumb_set TIM1_UP_IRQHandler, Default_Handler
    .weak  TIM2_IRQHandler
    .thumb_set TIM2_IRQHandler, Default_Handler
    .weak  TIM6_IRQHandler
    .thumb_set TIM6_IRQHandler, Default_Handler
    .weak  TIM7_IRQHandler
    .thumb_set TIM7_IRQHandler, Default_Handler
    .weak  I2C1_EV_IRQHandler
    .thumb_set I2C1_EV_IRQHandler, Default_Handler
    .weak  I2C1_ER_IRQHandler
    .thumb_set I2C1_ER_IRQHandler, Default_Handler
    .weak  I2C2_EV_IRQHandler
    .thumb_set I2C2_EV_IRQHandler, Default_Handler
    .weak  I2C2_ER_IRQHandler
    .thumb_set I2C2_ER_IRQHandler, Default_Handler
    .weak  SPI1_IRQHandler
    .thumb_set SPI1_IRQHandler, Default_Handler
    .weak  SPI2_IRQHandler
    .thumb_set SPI2_IRQHandler, Default_Handler
    .weak  SPI3_IRQHandler
    .thumb_set SPI3_IRQHandler, Default_Handler
    .weak  USART1_IRQHandler
    .thumb_set USART1_IRQHandler, Default_Handler
    .weak  USART2_IRQHandler
    .thumb_set USART2_IRQHandler, Default_Handler
    .weak  UART4_IRQHandler
    .thumb_set UART4_IRQHandler, Default_Handler
    .weak  UART5_IRQHandler
    .thumb_set UART5_IRQHandler, Default_Handler
    .weak  UART7_IRQHandler
    .thumb_set UART7_IRQHandler, Default_Handler
    .weak  UART8_IRQHandler
    .thumb_set UART8_IRQHandler, Default_Handler
    .weak  I2C3_EV_IRQHandler
    .thumb_set I2C3_EV_IRQHandler, Default_Handler
    .weak  I2C3_ER_IRQHandler
    .thumb_set I2C3_ER_IRQHandler, Default_Handler
    .weak  OTG_FS_IRQHandler
    .thumb_set OTG_FS_IRQHandler, Default_Handler
    .weak  ETH_IRQHandler
    .thumb_set ETH_IRQHandler, Default_Handler
    .weak  CORDIC_IRQHandler
    .thumb_set CORDIC_IRQHandler, Default_Handler
    .weak  UART9_IRQHandler
    .thumb_set UART9_IRQHandler, Default_Handler
    .weak  USART6_IRQHandler
    .thumb_set USART6_IRQHandler, Default_Handler
    .weak  USART10_IRQHandler
    .thumb_set USART10_IRQHandler, Default_Handler
    .weak  USART11_IRQHandler
    .thumb_set USART11_IRQHandler, Default_Handler
    .weak  UART12_IRQHandler
    .thumb_set UART12_IRQHandler, Default_Handler
    .weak  I3C1_EV_IRQHandler
    .thumb_set I3C1_EV_IRQHandler, Default_Handler
    .weak  I3C1_ER_IRQHandler
    .thumb_set I3C1_ER_IRQHandler, Default_Handler
    .weak  DTS_IRQHandler
    .thumb_set DTS_IRQHandler, Default_Handler
    .weak  MCE1_IRQHandler
    .thumb_set MCE1_IRQHandler, Default_Handler
    .weak  MCE2_IRQHandler
    .thumb_set MCE2_IRQHandler, Default_Handler
    .weak  MCE3_IRQHandler
    .thumb_set MCE3_IRQHandler, Default_Handler
    .weak  LPTIM1_IRQHandler
    .thumb_set LPTIM1_IRQHandler, Default_Handler
    .weak  LPTIM2_IRQHandler
    .thumb_set LPTIM2_IRQHandler, Default_Handler
    .weak  LPTIM3_IRQHandler
    .thumb_set LPTIM3_IRQHandler, Default_Handler
    .weak  LPTIM4_IRQHandler
    .thumb_set LPTIM4_IRQHandler, Default_Handler
    .weak  LPTIM5_IRQHandler
    .thumb_set LPTIM5_IRQHandler, Default_Handler
    .weak  LPTIM6_IRQHandler
    .thumb_set LPTIM6_IRQHandler, Default_Handler
    .weak  VREFBUF_IRQHandler
    .thumb_set VREFBUF_IRQHandler, Default_Handler
    .weak  I2C4_EV_IRQHandler
    .thumb_set I2C4_EV_IRQHandler, Default_Handler
    .weak  I2C4_ER_IRQHandler
    .thumb_set I2C4_ER_IRQHandler, Default_Handler
    .weak  SPI4_IRQHandler
    .thumb_set SPI4_IRQHandler, Default_Handler
    .weak  SPI5_IRQHandler
    .thumb_set SPI5_IRQHandler, Default_Handler
    .weak  SPI6_IRQHandler
    .thumb_set SPI6_IRQHandler, Default_Handler
    .weak  SAI1_IRQHandler
    .thumb_set SAI1_IRQHandler, Default_Handler
    .weak  SAI2_IRQHandler
    .thumb_set SAI2_IRQHandler, Default_Handler
    .weak  TIM3_IRQHandler
    .thumb_set TIM3_IRQHandler, Default_Handler
    .weak  TIM4_IRQHandler
    .thumb_set TIM4_IRQHandler, Default_Handler
    .weak  TIM8_BRK_IRQHandler
    .thumb_set TIM8_BRK_IRQHandler, Default_Handler
    .weak  TIM8_UP_IRQHandler
    .thumb_set TIM8_UP_IRQHandler, Default_Handler
    .weak  TIM8_TRG_COM_IRQHandler
    .thumb_set TIM8_TRG_COM_IRQHandler, Default_Handler
    .weak  TIM8_CC_IRQHandler
    .thumb_set TIM8_CC_IRQHandler, Default_Handler
    .weak  TIM15_IRQHandler
    .thumb_set TIM15_IRQHandler, Default_Handler
    .weak  TIM16_IRQHandler
    .thumb_set TIM16_IRQHandler, Default_Handler
    .weak  TIM17_IRQHandler
    .thumb_set TIM17_IRQHandler, Default_Handler
    .weak  HASH_IRQHandler
    .thumb_set HASH_IRQHandler, Default_Handler
    .weak  CRYP_IRQHandler
    .thumb_set CRYP_IRQHandler, Default_Handler
    .weak  PKA_IRQHandler
    .thumb_set PKA_IRQHandler, Default_Handler
