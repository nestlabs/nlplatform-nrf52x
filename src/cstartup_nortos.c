/*
 *
 *    Copyright (c) 2017-2018 Nest Labs, Inc.
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
 *      This is the bootloader-specific counterpart to cstartup.c.
 */
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <nlassert.h>
#include <nlplatform.h>
#include <nlplatform_nrf52x/cstartup.h>

#include <FreeRTOSConfig.h>

#if defined(WATCHDOG_IRQ_NUM) && defined(WATCHDOG_IRQ_PRIORITY)
nlSTATIC_ASSERT(WATCHDOG_IRQ_PRIORITY <= ATOMIC_PRIORITY_LEVEL);
#endif

uint32_t nortos_main_stack[CSTACK_SIZE_BOOTLOADER] __attribute((section(".cstack")));
uint8_t cstack_guard[CSTACK_GUARD_SIZE_BYTES] __attribute((section(".guard_region")));

static const vector_table_t nortos_vector_table __attribute__((used)) __attribute((section(".bat"))) =
{
    VECTOR_TABLE_ENTRIES
};

static const vector_table_t app_vector_table __attribute__((used)) __attribute((section(".intvec"))) =
{
    VECTOR_TABLE_ENTRIES
};

// Reset handler used by bootloader
void BootloaderEntryHandler(void);
void BootloaderEntryHandler(void)
{
    /* Turn on power to all RAM sections. Since RAM power registers are retained
     * across soft resets, it's more conservative to always power on all RAM in
     * the bootloader than assume that the bootloader's RAM usage is always strictly
     * less than the RAM usage of any previous running app. Also, it simplifies the
     * code in the app so it only has to worry about turning off RAM sections it doesn't
     * need rather than also worrying about turning on power to RAM sections that might
     * have been left off by another app (e.g. transitioning back and forth between
     * AUPD and the regular app).
     */
    for (unsigned ram_ahb_slave = 0; ram_ahb_slave < ARRAY_SIZE(NRF_POWER->RAM); ram_ahb_slave++)
    {
        /* Most AHB slaves only have 2 sections and only the RAM[8] has 6 sections,
         * but it's okay to write more bits (extras will just be ignored)
         */
        NRF_POWER->RAM[ram_ahb_slave].POWERSET = (POWER_RAM_POWERSET_S5POWER_Msk |
                                                  POWER_RAM_POWERSET_S4POWER_Msk |
                                                  POWER_RAM_POWERSET_S3POWER_Msk |
                                                  POWER_RAM_POWERSET_S2POWER_Msk |
                                                  POWER_RAM_POWERSET_S1POWER_Msk |
                                                  POWER_RAM_POWERSET_S0POWER_Msk);
    }

    nlplatform_cstartup_init();

    /* Workaround for Errata issue 115: RAM contents not being retained properly in Idle or Off mode.
     * Increases RAM retention power consumption from 20nA to 30nA
     * TODO: review and remove when new silicon arrives
     */
    *(volatile uint32_t*)0x40000ee4 = ((*(volatile uint32_t*)0x40000ee4 & 0x00000070) | // keep bits 6:4
                                       0x0000000f); // replace bits 3:0 with 0xf

    /* Workaround for Errata 136 "System: Bits in RESETREAS are set when they should not be"
     * TODO: review and remove when new silicon arrives
     */
    if (NRF_POWER->RESETREAS & POWER_RESETREAS_RESETPIN_Msk){
        NRF_POWER->RESETREAS =  ~POWER_RESETREAS_RESETPIN_Msk;
    }

    /* TODO: Enable FPU if we decide we want to use it. Consumes power so leave it disabled for now.
     * NOTE: FreeRTOS port.c for M4F already enables FPU so this is really only needed
     *       if bootloader or AUPD decide to use FPU
     *    SCB->CPACR |= (3UL << 20) | (3UL << 22)
     */

    /* Configure all NVIC priorities to default level two lower than the ATOMIC_PRIORITY_LEVEL
     * defined in FreeRTOSConfig.h. This allows (if needed) some interrupts to be configured
     * to be higher priority later if needed yet still not interfere with any code that
     * requires NVIC interrupts to be masked off.
     * Most of our nlplatform code does not expect nested interrupts (our stacks are too
     * small for that) so if there are interrupts at different priorities, we need to
     * make sure the nesting case is well tested.
     */
    for (IRQn_Type i = POWER_CLOCK_IRQn; i <= PWM3_IRQn; i++)
    {
        NVIC_SetPriority(i, ATOMIC_PRIORITY_LEVEL + 2);
    }
#if defined(WATCHDOG_IRQ_NUM) && defined( WATCHDOG_IRQ_PRIORITY)
    NVIC_SetPriority(WATCHDOG_IRQ_NUM, WATCHDOG_IRQ_PRIORITY);
#endif

    /* Copy initialized data from FLASH to RAM, if applicable. */
    memcpy((void*)&__DATA__start, (void*)&__DATA_INIT__start, (unsigned)&__DATA_INIT__size);
    /* Zero the bss section - might be faster to use uint32_t stores */
    memset((void*)&__BSS__start, 0, (unsigned)&__BSS__size);

    /* Jump to main */
    main();
}

// Reset handler used by AUPD
void AppEntryHandler(void);
void AppEntryHandler(void)
{
    /* We don't turn off power to unused RAM in AUPD, since the RAM usage is much
     * different than the app and we could lose information the app hoped to retain
     * between updates/resets. Bootloader should have powered on all RAM sections.
     *
     * For non-release builds, to make it easier to iterate development of AUPD
     * when the bootloader isn't run, always enable all RAM.
     */
#ifndef BUILD_CONFIG_RELEASE
    for (unsigned ram_ahb_slave = 0; ram_ahb_slave < ARRAY_SIZE(NRF_POWER->RAM); ram_ahb_slave++)
    {
        /* Most AHB slaves only have 2 sections and only the RAM[8] has 6 sections,
         * but it's okay to write more bits (extras will just be ignored)
         */
        NRF_POWER->RAM[ram_ahb_slave].POWERSET = (POWER_RAM_POWERSET_S5POWER_Msk |
                                                  POWER_RAM_POWERSET_S4POWER_Msk |
                                                  POWER_RAM_POWERSET_S3POWER_Msk |
                                                  POWER_RAM_POWERSET_S2POWER_Msk |
                                                  POWER_RAM_POWERSET_S1POWER_Msk |
                                                  POWER_RAM_POWERSET_S0POWER_Msk);
    }
#endif

    /* Switch to app vector table */
    SCB->VTOR = (uint32_t)&app_vector_table;

    nlplatform_cstartup_init();

    /* Zero the bss section - might be faster to use uint32_t stores */
    memset((void*)&__BSS__start, 0, (unsigned)&__BSS__size);

    /* Jump to main */
    main();
}
