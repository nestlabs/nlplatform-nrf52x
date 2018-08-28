/*
 *
 *    Copyright (c) 2012-2018 Nest Labs, Inc.
 *    All rights reserved.
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *        http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

/*
 *    Description:
 *      This file defines quantities useful for setting up the C
 *      runtime. This includes
 *
 *      - Symbols describing the start/end/size of sections in RAM.
 *      - Interrupt and fault handlers.
 *      - A macro specifying the memory order of interrupt and fault handlers.
 */

#ifndef _CSTARTUP_H_INCLUDED__
#define _CSTARTUP_H_INCLUDED__

extern void Reset_Handler(void);
extern void NMI_Handler(void);
extern void HardFault_Handler(void);
extern void MemoryManagement_Handler(void);
extern void BusFault_Handler(void);
extern void UsageFault_Handler(void);
extern void HardFault_Handler(void);
extern void SVC_Handler(void);
extern void DebugMon_Handler(void);
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);
extern void POWER_CLOCK_IRQHandler(void);
extern void Radio_IRQHandler(void);
extern void UARTE0_IRQHandler(void);
extern void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler(void);
extern void SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler(void);
extern void NFCT_IRQHandler(void);
extern void GPIOTE_IRQHandler(void);
extern void SAADC_IRQHandler(void);
extern void TIMER0_IRQHandler(void);
extern void TIMER1_IRQHandler(void);
extern void TIMER2_IRQHandler(void);
extern void RTC0_IRQHandler(void);
extern void TEMP_IRQHandler(void);
extern void RNG_IRQHandler(void);
extern void ECB_IRQHandler(void);
extern void CCM_AAR_IRQHandler(void);
extern void WDT_IRQHandler(void);
extern void RTC1_IRQHandler(void);
extern void QDEC_IRQHandler(void);
extern void COMP_LPCOMP_IRQHandler(void);
extern void SWI0_EGU0_IRQHandler(void);
extern void SWI1_EGU1_IRQHandler(void);
extern void SWI2_EGU2_IRQHandler(void);
extern void SWI3_EGU3_IRQHandler(void);
extern void SWI4_EGU4_IRQHandler(void);
extern void SWI5_EGU5_IRQHandler(void);
extern void TIMER3_IRQHandler(void);
extern void TIMER4_IRQHandler(void);
extern void PWM0_IRQHandler(void);
extern void PDM_IRQHandler(void);
extern void MWU_IRQHandler(void);
extern void PWM1_IRQHandler(void);
extern void PWM2_IRQHandler(void);
extern void SPIM2_SPIS2_SPI2_IRQHandler(void);
extern void RTC2_IRQHandler(void);
extern void I2S_IRQHandler(void);
extern void FPU_IRQHandler(void);
extern void USBD_IRQHandler(void);
extern void UARTE1_IRQHandler(void);
extern void QSPI_IRQHandler(void);
extern void CRYPTOCELL_IRQHandler(void);
extern void SPIM3_IRQHandler(void);
extern void PWM3_IRQHandler(void);

extern void main(void);

extern uint32_t const __BSS__start;
extern uint32_t const __BSS__size;
extern uint32_t const __CSTACK__end;
extern uint32_t const __DATA__start;
extern uint32_t const __DATA__size;
extern uint32_t const __DATA_INIT__start;
extern uint32_t const __DATA_INIT__size;
extern uint32_t const __RAM__end;
extern uint32_t const __RAMFUNC__start;
extern uint32_t const __RAMFUNC__size;
extern uint32_t const __RAMFUNC_INIT__size;
extern uint32_t const __RAMFUNC_INIT__start;
extern uint32_t const __SIGNATURE__start;

#define VECTOR_TABLE_ENTRIES \
    (uint32_t)&__CSTACK__end, \
    Reset_Handler, \
    NMI_Handler, \
    HardFault_Handler, \
    MemoryManagement_Handler, \
    BusFault_Handler, \
    UsageFault_Handler, \
    HardFault_Handler, \
    HardFault_Handler, \
    HardFault_Handler, \
    HardFault_Handler, \
    SVC_Handler, \
    DebugMon_Handler, \
    HardFault_Handler, \
    PendSV_Handler, \
    SysTick_Handler, \
                            \
    POWER_CLOCK_IRQHandler,                       /* NVIC interrupt 0 */ \
    Radio_IRQHandler,                             /* NVIC interrupt 1 */ \
    UARTE0_IRQHandler,                            /* NVIC interrupt 2 */ \
    SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler, /* NVIC interrupt 3 */ \
    SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler, /* NVIC interrupt 4 */ \
    NFCT_IRQHandler,                              /* NVIC interrupt 5 */ \
    GPIOTE_IRQHandler,                            /* NVIC interrupt 6 */ \
    SAADC_IRQHandler,                             /* NVIC interrupt 7 */ \
    TIMER0_IRQHandler,                            /* NVIC interrupt 8 */ \
    TIMER1_IRQHandler,                            /* NVIC interrupt 9 */ \
    TIMER2_IRQHandler,                            /* NVIC interrupt 10 */ \
    RTC0_IRQHandler,                              /* NVIC interrupt 11 */ \
    TEMP_IRQHandler,                              /* NVIC interrupt 12 */ \
    RNG_IRQHandler,                               /* NVIC interrupt 13 */ \
    ECB_IRQHandler,                               /* NVIC interrupt 14 */ \
    CCM_AAR_IRQHandler,                           /* NVIC interrupt 15 */ \
    WDT_IRQHandler,                               /* NVIC interrupt 16 */ \
    RTC1_IRQHandler,                              /* NVIC interrupt 17 */ \
    QDEC_IRQHandler,                              /* NVIC interrupt 18 */ \
    COMP_LPCOMP_IRQHandler,                       /* NVIC interrupt 19 */ \
    SWI0_EGU0_IRQHandler,                         /* NVIC interrupt 20 */ \
    SWI1_EGU1_IRQHandler,                         /* NVIC interrupt 21 */ \
    SWI2_EGU2_IRQHandler,                         /* NVIC interrupt 22 */ \
    SWI3_EGU3_IRQHandler,                         /* NVIC interrupt 23 */ \
    SWI4_EGU4_IRQHandler,                         /* NVIC interrupt 24 */ \
    SWI5_EGU5_IRQHandler,                         /* NVIC interrupt 25 */ \
    TIMER3_IRQHandler,                            /* NVIC interrupt 26 */ \
    TIMER4_IRQHandler,                            /* NVIC interrupt 27 */ \
    PWM0_IRQHandler,                              /* NVIC interrupt 28 */ \
    PDM_IRQHandler,                               /* NVIC interrupt 29 */ \
    HardFault_Handler,                            /* NVIC interrupt 30 */ \
    HardFault_Handler,                            /* NVIC interrupt 31 */ \
    MWU_IRQHandler,                               /* NVIC interrupt 32 */ \
    PWM1_IRQHandler,                              /* NVIC interrupt 33 */ \
    PWM2_IRQHandler,                              /* NVIC interrupt 34 */ \
    SPIM2_SPIS2_SPI2_IRQHandler,                  /* NVIC interrupt 35 */ \
    RTC2_IRQHandler,                              /* NVIC interrupt 36 */ \
    I2S_IRQHandler,                               /* NVIC interrupt 37 */ \
    FPU_IRQHandler,                               /* NVIC interrupt 38 */ \
    USBD_IRQHandler,                              /* NVIC interrupt 39 */ \
    UARTE1_IRQHandler,                            /* NVIC interrupt 40 */ \
    QSPI_IRQHandler,                              /* NVIC interrupt 41 */ \
    CRYPTOCELL_IRQHandler,                        /* NVIC interrupt 42 */ \
    SPIM3_IRQHandler,                             /* NVIC interrupt 43 */ \
    HardFault_Handler,                            /* NVIC interrupt 44 */ \
    PWM3_IRQHandler                               /* NVIC interrupt 45 */

#endif /* __CSTARTUP_H_INCLUDED__ */
