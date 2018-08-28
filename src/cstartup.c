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
*      This provides functions for setting up the C-runtime, delegating to the
*      hardware-agnostic runtime setup, and jumping to main().
*/
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <nlmacros.h>

#include <nlplatform.h>
#include <nlplatform_nrf52x/cstartup.h>

void AppEntryHandler(void);

uint8_t nortos_main_stack[CSTACK_SIZE_BYTES] __attribute((section(".cstack")));
uint8_t cstack_guard[CSTACK_GUARD_SIZE_BYTES] __attribute((section(".guard_region")));

static const app_image_header_t app_image_header __attribute__((used)) __attribute((section(".aat"))) =
{
    (uint32_t)&__CSTACK__end,
    AppEntryHandler,
    (void*)&__SIGNATURE__start
};

static const vector_table_t app_vector_table __attribute__((used)) __attribute((section(".intvec"))) =
{
    VECTOR_TABLE_ENTRIES
};

#define MAX_RAM_OFFSET_COVERED_BY_4KB_SECTIONS 65536
#define RAM_SECTION_SIZE_4KB 4096UL
#define RAM_SIZE_SMALL_SLAVE 8192UL
#define RAM_SECTION_SIZE_32KB 32768UL

static void disable_unused_RAM(void)
{
    /* The 256KB of RAM is divided into sections of different sizes and
     * grouped under 9 AHB slaves.
     *
     * RAM AHB slave 0-7 have two 4KB sections each
     * (i.e. first 64KB of RAM divided into sixteen 4KB sections).
     * RAM AHB slave 8 has six 32KB sections.
     *
     * The end of RAM we need to keep powered should be indicated
     * by the linker symbol __RAM__end.
     *
     * Since RAM power registers retain state across SW resets, we
     * expect the bootloader to have enabled all RAM sections and
     * the app only calls this to turn off unused RAM sections.
     */
    uint32_t ram_retain_end_offset = (uint32_t)&__RAM__end - NL_SOC_RAM_BASE_ADDRESS;
    POWER_RAM_Type *ram_power_reg;

    // Find AHB slave control register for first section we will be turning off
    if (ram_retain_end_offset < MAX_RAM_OFFSET_COVERED_BY_4KB_SECTIONS)
    {
        // Round up to nearest 4KB section
        ram_retain_end_offset = ROUNDUP(ram_retain_end_offset, RAM_SECTION_SIZE_4KB);

        // find the AHB slave to start at
        ram_power_reg = &NRF_POWER->RAM[ram_retain_end_offset / RAM_SIZE_SMALL_SLAVE];

        // see if the offset is the second section of the AHB slave
        if (ram_retain_end_offset % RAM_SIZE_SMALL_SLAVE)
        {
            // disable just the second section and move to next slave
            ram_power_reg->POWERCLR = POWER_RAM_POWERCLR_S1POWER_Msk;
            ram_power_reg++;
        }

        // Turn off sections within the remaining first 8 AHB slaves
        while (ram_power_reg < &NRF_POWER->RAM[8])
        {
            // disable both sections of this AHB slave
            ram_power_reg->POWERCLR = POWER_RAM_POWERCLR_S1POWER_Msk | POWER_RAM_POWERCLR_S0POWER_Msk;
            ram_power_reg++;
        }
        // Turn off all six sections in AHB slave 8
        ram_power_reg->POWERCLR = (POWER_RAM_POWERCLR_S5POWER_Msk |
                                   POWER_RAM_POWERCLR_S4POWER_Msk |
                                   POWER_RAM_POWERCLR_S3POWER_Msk |
                                   POWER_RAM_POWERCLR_S2POWER_Msk |
                                   POWER_RAM_POWERCLR_S1POWER_Msk |
                                   POWER_RAM_POWERCLR_S0POWER_Msk);
    }
    else
    {
        // The ram_retain_end_offset is within the sections controlled
        // by RAM AHB slave 8, covering the last 192KB of RAM.
        uint32_t retain_mask;

        // Round up to nearest 32KB section
        ram_retain_end_offset = ROUNDUP(ram_retain_end_offset, RAM_SECTION_SIZE_32KB);

        // Convert the value into a mask of the bits retained.
        retain_mask = (1 << ((ram_retain_end_offset - MAX_RAM_OFFSET_COVERED_BY_4KB_SECTIONS) / RAM_SECTION_SIZE_32KB)) - 1;

        // Turn off the sections not retained
        NRF_POWER->RAM[8].POWERCLR = ~retain_mask;
    }
}

// Reset vector/entry used by FreeRTOS apps (not bootloader or AUPD, which don't run FreeRTOS)
void AppEntryHandler(void)
{
    /* For non-release builds, to make it easier to iterate development of
     * the app when the bootloader isn't run, always enable all RAM.
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

    /* Shutoff unused RAM sections to save power */
    disable_unused_RAM();

    /* Switch to app vector table */
    SCB->VTOR = (uint32_t)&app_vector_table;

    nlplatform_cstartup_init();

    /* Workaround for Errata 36 "CLOCK: Some registers are not reset when expected" */
    NRF_CLOCK->EVENTS_DONE = 0;
    NRF_CLOCK->EVENTS_CTTO = 0;
    NRF_CLOCK->CTIV = 0;

    /* Workaround for Errata 66 "TEMP: Linearity specification not met with default settings" */
    NRF_TEMP->A0 = NRF_FICR->TEMP.A0;
    NRF_TEMP->A1 = NRF_FICR->TEMP.A1;
    NRF_TEMP->A2 = NRF_FICR->TEMP.A2;
    NRF_TEMP->A3 = NRF_FICR->TEMP.A3;
    NRF_TEMP->A4 = NRF_FICR->TEMP.A4;
    NRF_TEMP->A5 = NRF_FICR->TEMP.A5;
    NRF_TEMP->B0 = NRF_FICR->TEMP.B0;
    NRF_TEMP->B1 = NRF_FICR->TEMP.B1;
    NRF_TEMP->B2 = NRF_FICR->TEMP.B2;
    NRF_TEMP->B3 = NRF_FICR->TEMP.B3;
    NRF_TEMP->B4 = NRF_FICR->TEMP.B4;
    NRF_TEMP->B5 = NRF_FICR->TEMP.B5;
    NRF_TEMP->T0 = NRF_FICR->TEMP.T0;
    NRF_TEMP->T1 = NRF_FICR->TEMP.T1;
    NRF_TEMP->T2 = NRF_FICR->TEMP.T2;
    NRF_TEMP->T3 = NRF_FICR->TEMP.T3;
    NRF_TEMP->T4 = NRF_FICR->TEMP.T4;

    /* Copy initialized data from FLASH to RAM */
    assert ((unsigned)&__DATA__size == (unsigned)&__DATA_INIT__size);
    memcpy((void*)&__DATA__start, (void*)&__DATA_INIT__start, (unsigned)&__DATA__size);
    /* Zero the bss section - might be faster to use uint32_t stores */
    memset((void*)&__BSS__start, 0, (unsigned)&__BSS__size);
    /* Copy ramfunc code from FLASH to RAM */
    assert ((unsigned)&__RAMFUNC__size == (unsigned)&__RAMFUNC_INIT__size);
    memcpy((void*)&__RAMFUNC__start, (void*)&__RAMFUNC_INIT__start, (unsigned)&__RAMFUNC__size);

    /* Todo: setup ACL, ? */

    /* Jump to main */
    main();
}

