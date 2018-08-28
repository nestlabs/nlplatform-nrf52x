/*
 *
 *    Copyright (c) 2015-2018 Nest Labs, Inc.
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
 *      This file...
 *
 */

#include <stdio.h>
#include <stdlib.h>

#define NLPLATFORM_INTERRUPT_FUNCTIONS
#define CALLING_NL_PRODUCT_INIT
#include <nlplatform.h>
#undef CALLING_NL_PRODUCT_INIT
#undef NLPLATFORM_INTERRUPT_FUNCTIONS

#include <nlplatform/nlfault.h>
#include <nlplatform/nlflash.h>
#include <nlplatform/nlflash_spi.h>
#include <nlplatform/nlfs.h>
#include <nlplatform/nlgpio.h>
#include <nlplatform/nlmpu.h>
#include <nlplatform/nltimer.h>
#include <nlplatform/nluart.h>
#include <nlplatform/nlwatchdog.h>
#include <nlplatform/nlwatchpoint.h>

#if defined(BUILD_FEATURE_NL_PROFILE) && !defined(NL_NO_RTOS)
#include <nlplatform/nlprofile.h>
#endif

#include <nlplatform_nrf52x/nrf52x_gpio.h>
#include <nlplatform_nrf52x/nlflash_nrf52x.h>
#if defined(BUILD_FEATURE_SW_TIMER) && !defined(NL_NO_RTOS)
#include <nlplatform/nlswtimer.h>
#include <nlertime.h>
#endif

#include <FreeRTOS.h>
#include <task.h>
#include <portmacro.h>

#include <nlassert.h>

#if defined(BUILD_FEATURE_SW_TIMER) && !defined(NL_NO_RTOS)
static uint32_t blk_sleep_tmr_cb(nl_swtimer_t *timer, void *arg);
static struct block_sleep_tmr_s
{
    nl_swtimer_t timer;
    nl_time_native_t end;
} s_blk_sleep_tmr;
#endif

// Assert that FreeRTOS ATOMIC_PRIORITY_LEVEL is same as PRODUCT_INTERRUPT_DISABLE_BASE_PRIORITY
#ifdef PRODUCT_INTERRUPT_DISABLE_BASE_PRIORITY
nlSTATIC_ASSERT(ATOMIC_PRIORITY_LEVEL == PRODUCT_INTERRUPT_DISABLE_BASE_PRIORITY);
nlSTATIC_ASSERT(configMAX_SYSCALL_INTERRUPT_PRIORITY == PRODUCT_INTERRUPT_DISABLE_BASE_PRIORITY_VALUE);
#endif

/* Allow product to define the RTC PRESCALER value, which determines the FreeRTOS
 * tick frequency. If not defined, we default to a 4MS (really 3.9978ms) tick
 */
#ifndef NRF_RTC_PRESCALER_VALUE
#define NRF_RTC_PRESCALER_VALUE 130 /* Gives a frequency of 250.1374046Hz, 3.9978ms tick */
#endif

static uint32_t sCachedRebootCause;

static int sBlockSleepCount = 0;

/* nrf52x has 4 watchpoint comparators.  We use the last one for
 * guarding task stacks.
 */
#define TASK_STACK_WATCHPOINT_INDEX 3

/* guard 256 bytes at 0x0. could guard more, but I think this
 * is sufficient to catch most NULL ptr use cases. this
 * covers the vector table.
 */
#define ADDRESS_ZERO_GUARD_SIZE 256

/* MPU defines, don't seem to be in CMSIS */
#define MPU_RASR_AP_NO_ACCESS 0
#define MPU_RASR_AP_READ_ONLY 7
#define MPU_RASR_TEX_NORMAL   1


static void guard_cstack_and_address_0(void)
{
    // We generally prefer to use the watchpoint block to catch stack
    // overflow instead of the MPU because the MPU would prevent
    // all writes, including the pushing of fault context (if we want
    // to catch interrupt handlers causing stack overflows, we have to
    // enable the read-only access in privileged mode as well as unprivileged
    // mode). The watchpoint catches writes after the fact, and still allows
    // the pushing of context by the HW on fault, so we can get a full
    // context and backtrace of the faulting instruction.

    // So a watchpoint is used for guarding the current task stack, which
    // we update on every context switch. However, there are very few
    // watchpoints (4 on a typical cortex-m3 and m4) so we don't want
    // to use more than one. We use the MPU to guard the CSTACK, which
    // should be much less likely to overflow. We set up the protection
    // once at boot.
    extern uint32_t const __GUARD_REGION__start;
    extern uint32_t const __GUARD_REGION__size;

    // this function is called before .bss and .data are initialized,
    // so must be careful about any globals that may be used by
    // any functions called. the only global known to be accessed
    // is int_lock_count, when nl_mpu_request_region() uses
    // nlplatform_interrupt_disable() and nlplatform_interrupt_enable().
    // set int_lock_count to 0 manually here.
    int_lock_count = 0;

    nl_mpu_init();
    nl_mpu_request_region((unsigned)&__GUARD_REGION__start, (unsigned)&__GUARD_REGION__size,
                          MPU_RASR_XN_Msk |
                          MPU_RASR_AP_READ_ONLY << MPU_RASR_AP_Pos | // privileged and unprivileged read-only, so backtrace attempts don't fault when reading the area
                          MPU_RASR_TEX_NORMAL << MPU_RASR_TEX_Pos); // normal memory, outer and inner non-cacheable, not shared
#ifdef NL_BOOTLOADER
    if (SCB->VTOR == 0x0)
    {
        // for the bootloader, set address 0 guard to read only
        // so fault handling still works
        nl_mpu_request_region(0x0, ADDRESS_ZERO_GUARD_SIZE,
                              MPU_RASR_XN_Msk |
                              MPU_RASR_AP_READ_ONLY << MPU_RASR_AP_Pos | // privileged and unprivileged read-only, so fault handling still works
                              MPU_RASR_TEX_NORMAL << MPU_RASR_TEX_Pos); // normal memory, outer and inner non-cacheable, not shared

    }
    else
    {
        // for aupd, we've already switched vector table so set the
        // the address 0 guard to no access. note that if aupd
        // wants to update the bootloader, it needs to disable
        // the mpu, and then reenable it afterwards.
        nl_mpu_request_region(0x0, ADDRESS_ZERO_GUARD_SIZE,
                              MPU_RASR_XN_Msk |
                              MPU_RASR_AP_NO_ACCESS << MPU_RASR_AP_Pos | // privileged and unprivileged no access
                              MPU_RASR_TEX_NORMAL << MPU_RASR_TEX_Pos); // normal memory, outer and inner non-cacheable, not shared
    }
#else
    // for regular apps, set the address 0 guard to no access
    // to catch NULL ptr usage
    nl_mpu_request_region(0x0, ADDRESS_ZERO_GUARD_SIZE,
                          MPU_RASR_XN_Msk |
                          MPU_RASR_AP_NO_ACCESS << MPU_RASR_AP_Pos | // privileged and unprivileged no access
                          MPU_RASR_TEX_NORMAL << MPU_RASR_TEX_Pos); // normal memory, outer and inner non-cacheable, not shared
#endif
    nl_mpu_enable(true, true, true);
}

/* There are three variants of nlplatform_init().  One for the bootloader,
 * one for aupd, and one app/shell.
 *
 * The bootloader calls nlplatform_init on every boot, and so do the apps
 * (aupd and full app).  To make boot time faster, we want to avoid
 * repeating any initialization in the app that was done already by
 * the bootloader.
 *
 * aupd differs from the full app because it has no FreeRTOS, making it
 * more like the bootloader in this regard.  We compile two versions of
 * nlplatform_soc, one with FreeRTOS and one without FreeRTOS.  However,
 * since aupd and the bootloader both use the non-FreeRTOS version, but
 * we want them to do different things at boot time, we use the linker
 * script to select which function to use (rather than building three
 * library variants).
 */
#ifdef NL_BOOTLOADER

/* used by the bootloader */
void nlplatform_init(void)
{
    sCachedRebootCause = NRF_POWER->RESETREAS;
    // don't clear reset reason so app can read it too

#ifdef BUILD_FEATURE_WATCHPOINT_STACK_GUARD
    // disable watchpoint used for stack guarding so any stack init routines
    // don't trip the watchpoint during early boot
    nl_watchpoint_disable(TASK_STACK_WATCHPOINT_INDEX);
#endif

    /* start LF clock */
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    /* For radio to work, need to use configure LFCLKSRC */
#ifdef PRODUCT_NRF_LFCLKSRC
    /* use product defined clock source */
    NRF_CLOCK->LFCLKSRC = PRODUCT_NRF_LFCLKSRC;
#else
    /* default clock source is normal xtal operation */
    NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
#endif
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    /* Wait for LFCLK to start */
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0) {}

    nltimer_init();

    nltimer_request(NLTIMER_DELAY);

    nlgpio_init();

    // not needed in the bootloader
    //nluart_init();

    nlwatchdog_init();

    nlplatform_crypto_init();

    /* set SEVONPEND so interrupts wake from WFE */
    SCB->SCR |= SCB_SCR_SEVONPEND_Msk;
}

void nlplatform_init_aupd(void);
void nlplatform_init_aupd(void)
{
    sCachedRebootCause = NRF_POWER->RESETREAS;
    // clear reset reasons so next read of the register doesn't have stale data
    NRF_POWER->RESETREAS = 0xffffffff;

#ifdef BUILD_FEATURE_WATCHPOINT_STACK_GUARD
    // use the last watchpoint to guard the first 32-bytes of the AUPD CSTACK
    extern uint32_t const __CSTACK__start;
    nl_watchpoint_enable(TASK_STACK_WATCHPOINT_INDEX, (uint32_t)&__CSTACK__start, WATCHPOINT_ON_WRITE, 0x5);
#endif

    nltimer_init();

    // Though bootloader did watchdog_init(), we have to run
    // again in the app since we need to check debugger state.
    nlwatchdog_init();

    nltimer_request(NLTIMER_DELAY);

    /* everything else was initialized by bootloader */
}

void nlplatform_cstartup_init(void)
{
#ifdef PRODUCT_NRF_DCDCEN
    /* use product defined regulator configuration */
    NRF_POWER->DCDCEN = PRODUCT_NRF_DCDCEN;
#else
    // enable DCDC buck, which is more efficient than the LDO that we boot with.
    // assumes external LC components are present to support DCDC.
    NRF_POWER->DCDCEN = POWER_DCDCEN_DCDCEN_Enabled << POWER_DCDCEN_DCDCEN_Pos;
#endif

    // enable instruction cache, profiling disabled
    NRF_NVMC->ICACHECNF = NVMC_ICACHECNF_CACHEEN_Msk;

    // enable ITM/DWT and debug monitor, which is used by watchpoint module,
    // nlwatchdog, and swo_vuart
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk | CoreDebug_DEMCR_MON_EN_Msk;

    guard_cstack_and_address_0();

#ifdef BUILD_FEATURE_WATCHPOINT_STACK_GUARD
    // disable watchpoint used for stack guarding so any stack init routines
    // don't trip the watchpoint during early boot
    nl_watchpoint_disable(TASK_STACK_WATCHPOINT_INDEX);
#endif
}

#else /* NL_BOOTLOADER */

#if defined(BUILD_FEATURE_WATCHDOG) && defined(BUILD_FEATURE_SW_TIMER)
static nl_swtimer_t watchdog_late_timer;
static uint32_t late_watchdog_timer_expiry(nl_swtimer_t *timer, void *arg)
{
    // No need to do anything, the wakeup alone should be enough to
    // expire the watchdog
    return 0;
}
#endif

/* used by apps */
void nlplatform_init(void)
{
    sCachedRebootCause = NRF_POWER->RESETREAS;
    // clear reset reasons so next read of the register doesn't have stale data
    NRF_POWER->RESETREAS = 0xffffffff;

    /* seed rand() */
    srand(nlplatform_get_random_number());

#ifdef BUILD_FEATURE_NO_INIT_FROM_BOOTLOADER
    /* If using the Nest bootloader then gpio init from
     * bootloader is sufficient, no need to repeat here. */
    nlgpio_init();
#endif

    // Though bootloader did watchdog_init(), we have to run
    // again in the app since we need to check debugger state.
    nlwatchdog_init();

    nltimer_init();

    // Configure timer we use for delays to be 32-bit, free running
    // and auto-repeating when it hits max uint32.
    // We turn it on and off as needed in nlplatform_delay_us().
    nltimer_request(NLTIMER_DELAY);
    nltimer_set(NLTIMER_DELAY, 0xffffffff, NULL, NULL, true);

    nluart_init(); /* needed to allocate semaphores per controller */

#if defined(BUILD_FEATURE_WATCHDOG) && defined(BUILD_FEATURE_SW_TIMER)
    nl_swtimer_init(&watchdog_late_timer, late_watchdog_timer_expiry, NULL);
#endif

#if defined(BUILD_FEATURE_SW_TIMER)
    nl_swtimer_init(&s_blk_sleep_tmr.timer, blk_sleep_tmr_cb, NULL);
#endif

    nlplatform_crypto_init();
}

void nlplatform_cstartup_init(void)
{
    // re-initialize the MPU and request a region to guard the CSTACK.
    // our GUARD_REGION might be different than the one in bootloader.
    guard_cstack_and_address_0();

    // watchpoint should already be disabled by bootloader so it won't
    // interfere with .data or .bss initialization
}

#endif /* NL_BOOTLOADER */

void nlplatform_reset(nl_reset_reason_t reset_reason)
{
#ifdef BUILD_FEATURE_RESET_INFO
    nl_reset_info_prepare_reset(reset_reason, NULL);
#endif

    // Run any product-specific reset hooks
    nlproduct_prepare_reset(reset_reason);

    // clear HW resetreas so it doesn't have stale info
    NRF_POWER->RESETREAS = 0xffffffff;

    NVIC_SystemReset();
}

int nlplatform_get_unique_id(const uint8_t **uid, size_t *len)
{
    *uid = (uint8_t *)&NRF_FICR->DEVICEID[0];
    *len = 8;

    return 0;
}

#if !defined(NL_BOOTLOADER)
void nlplatform_print_reset_cause(void)
{
    uint32_t uiX, uiY, uiZ;
    /**
     * Scan through the reset events and print out descriptors for
     * any of them which are set (on some hardware it is possible to
     * get multiple reset events, not sure about the EM35x, but may as
     * well check them all).
     *
     * The WatchDog is a special case where we'll also print out the
     * last flag state in order to determine which tasks (if any)
     * instigated the reset.
     */
    uiY = 0;
    uiZ = sCachedRebootCause;
    printf("Reset Event(s) : ");
    for (uiX = 0; uiX < ARRAY_SIZE(reset_evMasks); uiX++)
    {
        if (uiZ & reset_evMasks[uiX])
        {
            if (uiY > 0)
            {
                printf(", ");
            }
            uiY++;

            printf("%s", reset_evDescs[uiX]);
            if (POWER_RESETREAS_DOG_Msk == reset_evMasks[uiX])
            {
                printf(" (Flags $%08lx)", nl_watchDog_lastFlags);
            }
        }
    }
    printf("\n");
}

void nlplatform_print_wakeup_cause(void)
{
    // We're not using System OFF mode, just System ON low power,
    // and there's no special HW support for determining what
    // event/interrupt caused us to exit idle.  We don't try
    // to determine the wakeup cause
    printf("Wakeup cause unknown\n");
}
#endif /* !defined(NL_BOOTLOADER) */

static uint32_t rand32(void)
{
    uint32_t result;
    uint8_t *p8 = (uint8_t*)&result;
    uint8_t *p8end = (uint8_t*)(&result + 1);

    /* Enable bias correction.  Slower result but more uniformly distributed. */
    NRF_RNG->CONFIG = RNG_CONFIG_DERCEN_Enabled;
    /* Start */
    NRF_RNG->TASKS_START = 1;
    /* Collect 4 samples */
    while (p8 < p8end)
    {
        /* Clear VALRDY event status */
        NRF_RNG->EVENTS_VALRDY = 0;
        /* Wait for value to be ready */
        while (NRF_RNG->EVENTS_VALRDY == 0);
        /* Add to result */
        *p8++ = NRF_RNG->VALUE;
    }
    /* Stop to save power */
    NRF_RNG->TASKS_STOP = 1;
    return result;
}

uint32_t nlplatform_get_random_number(void)
{
#ifdef NL_BOOTLOADER
    return rand32();
#else
    uint32_t result;
    // disable interrupts while collecting random number to make it thread safe
    nlplatform_interrupt_disable();
    result = rand32();
    nlplatform_interrupt_enable();
    return result;
#endif
}

int nlplatform_get_entropy(unsigned char *outEntropy, size_t inSize)
{
    unsigned char *entropy;
    uint32_t      idx;
    uint32_t      rval;

    do
    {
        rval = (int)nlplatform_get_random_number();

        entropy = (unsigned char *)&rval;

        for (idx = 0; (idx < sizeof(int)) && (inSize > 0); idx++, inSize--)
            *outEntropy++ = *entropy++;
    }
    while (inSize > 0);

    return 0;
}

#ifdef NL_BOOTLOADER

/* No RTOS version doesn't need the complexity of thread-safety
 * so keep the code smaller and simpler.
 */
void nlplatform_delay_ms(unsigned delay_ms)
{
    // long delays might indicate a problem so assert to catch these.
    // also since HW timer is in microseconds and max 32-bit, this
    // catches value being too large.
    assert(delay_ms < 1000);
    nlplatform_delay_us(delay_ms * 1000);
}

void nlplatform_delay_us(unsigned delay_us)
{
    // long delays might indicate a problem so assert to catch these
    assert(delay_us < 1000000);
    // The Nordic reference code just delays by looping
    // with NOPS.  It says it's not very accurate since instruction
    // execution rate can vary based on clock and whether the code
    // is running from RAM, FLASH, or instruction cached FLASH.
    //
    // For now, we use a HW TIMER to do the delay, and then turn
    // it off to save power.

    // Configure for one-shot, auto-stop, no callback (we'll
    // just poll)
    nltimer_set(NLTIMER_DELAY, delay_us, NULL, NULL, false);
    nltimer_start(NLTIMER_DELAY);
    while (nltimer_active(NLTIMER_DELAY)) {};
    // Although stopped, call nltimer_stop() to shutdown and save power
    nltimer_stop(NLTIMER_DELAY);
}

#else /* NL_BOOTLOADER */

void nlplatform_delay_ms(unsigned delay_ms)
{
    while (delay_ms > 0) {
        nlplatform_delay_us(1000);
        delay_ms--;
    }
}

void nlplatform_delay_us(unsigned delay_us)
{
    // long delays might indicate a problem so assert to catch these
    assert(delay_us < 1000000);
    // The Nordic reference code just delays by looping
    // with NOPS.  It says it's not very accurate since instruction
    // execution rate can vary based on clock and whether the code
    // is running from RAM, FLASH, or instruction cached FLASH.
    //
    // For now, we use a HW TIMER to do the delay, and then turn
    // it off to save power.

    static uint8_t s_active_delays;
    uint32_t time_at_start;

    // increment the s_active_delays counter inside a critical section
    nlplatform_interrupt_disable();
    if (s_active_delays++ == 0)
    {
        // Start the HW timer
        time_at_start = 0;
        nltimer_start(NLTIMER_DELAY);
    }
    else
    {
        // Timer is already running, record start time
        time_at_start = nltimer_elapsed(NLTIMER_DELAY);
    }
    // while we wait, enable interrupts so we don't keep interrupts
    // off too long.
    nlplatform_interrupt_enable();

    // loop until the elapsed time reaches the target. HW timer
    // should wrap.
    while ((nltimer_elapsed(NLTIMER_DELAY) - time_at_start) < delay_us) {};

    // decrement the s_active_delays counter inside a critical section
    nlplatform_interrupt_disable();
    if (--s_active_delays == 0)
    {
        // we can shut off the timer now
        nltimer_stop(NLTIMER_DELAY);
    }
    nlplatform_interrupt_enable();
}

#endif /* NL_BOOTLOADER */

uint32_t nlplatform_get_last_reset_cause(void)
{
    return sCachedRebootCause;
}

nl_reset_reason_t nlplatform_get_reset_reason(void)
{
    static nl_reset_reason_t sCachedResetReason = NL_RESET_REASON_UNSPECIFIED;

    if (sCachedResetReason == NL_RESET_REASON_UNSPECIFIED)
    {
#ifdef BUILD_FEATURE_RESET_INFO
        nl_reset_reason_t reason;
#endif

        sCachedResetReason = NL_RESET_REASON_UNKNOWN;

        if (sCachedRebootCause & POWER_RESETREAS_RESETPIN_Msk)
        {
            // TODO: Uncomment this line once this constant is
            // introduced into nlplatform
            //sCachedResetReason = NL_RESET_REASON_EXTERNAL;
        }

#ifdef BUILD_FEATURE_RESET_INFO
        else if ((reason = nl_reset_info_get_reset_reason()) != NL_RESET_REASON_UNKNOWN)
        {
            sCachedResetReason = reason;
        }
        else if (IS_VALID_FAULT_REASON(g_reset_info.fault_info.reason))
        {
            sCachedResetReason = g_reset_info.fault_info.reason;
        }
#endif

        else if (sCachedRebootCause & POWER_RESETREAS_LOCKUP_Msk)
        {
            sCachedResetReason = NL_RESET_REASON_HARD_FAULT;
        }
        else if ((sCachedRebootCause & POWER_RESETREAS_OFF_Msk)    ||
                 (sCachedRebootCause & POWER_RESETREAS_LPCOMP_Msk) ||
                 (sCachedRebootCause & POWER_RESETREAS_DIF_Msk)    ||
                 (sCachedRebootCause & POWER_RESETREAS_NFC_Msk)    ||
                 (sCachedRebootCause & POWER_RESETREAS_VBUS_Msk))
        {
            sCachedResetReason = NL_RESET_REASON_UNKNOWN;
        }
        else if (sCachedRebootCause & POWER_RESETREAS_DOG_Msk)
        {
            sCachedResetReason = NL_RESET_REASON_WATCHDOG;
        }
        else if (sCachedRebootCause & POWER_RESETREAS_SREQ_Msk)
        {
            sCachedResetReason = NL_RESET_REASON_SW_REQUESTED;
        }
        else
        {
            // TODO: Uncomment this line once this constant is
            // introduced into nlplatform
            //sCachedResetReason = NL_RESET_REASON_POWER_ON;
        }
    }

    return sCachedResetReason;
}

void nlplatform_reset_info_init_done(void)
{
}

void nlplatform_quiesce_on_fault(void)
{
#if !defined(NL_BOOTLOADER)
    /**
     * If we're NOT targeting a bootloader build the UART will be in
     * asynchronous mode and requires a switch into FORCED SYNC MODE
     * prior to any further activity (i.e. printing) in the fault handler.
     */
    nluart_force_sync(CONSOLE_UART_ID);

    /**
     * Suspend task switching so that NLER logging calls do not try to
     * run scheduler.
     */
    vTaskSuspendAll();
#endif
#if (defined(NL_DISABLE_WATCHDOG_OPERATION) || defined(NL_WATCHDOG_REQUIRE_ENROLLMENT))
    /**
     * If compiled to not enable watchdog by default, enable it here so that we don't
     * hang in fault handler.
     */
    nlwatchdog_set_enable(true);
#endif
}

// Define a scratch buffer to be used for things like env
// Make sure it's word aligned.
uint8_t flash_activity_buffer[FLASH_BUFFER_SIZE] __attribute__((aligned(4))) __attribute__((used)) = {0};

// In diags mode or if explicitly allowing sysenv writes, define a large temp buffer to be used by
// env-dump, flash-exttest, sysenv writes, and env-unittest.
#if defined(BUILD_CONFIG_DIAGNOSTICS) || defined(BUILD_FEATURE_SYSENV_WRITES_ALLOWED)
uint8_t diags_temp_buffer[DIAGS_TEMP_RAM_SIZE] __attribute__((aligned(4))) __attribute__((used)) = {0};
// also define a small buffer
uint8_t diags_small_temp_buffer[DIAGS_SMALL_TEMP_RAM_SIZE] __attribute__((aligned(4))) __attribute__((used)) = {0};
#endif

// used by nlflash.c
const nlflash_func_table_t g_flash_device_table[NL_NUM_FLASH_IDS] =
{
    // flash_nrf52840 internal flash
    {
        // the bootloader doesn't need these but AUPD does.
        // rather than compiling three variants of this library
        // (normal, bootloader, aupd), we only compile two
        // and leave these hooks in.
        .init = NULL,
        .request = NULL,
        .release = NULL,
        .flush = NULL,
        .read_id = NULL,
        .get_info = nlflash_nrf52x_get_info,
        .erase = nlflash_nrf52x_erase,
        .read = nlflash_nrf52x_read,
        .write = nlflash_nrf52x_write,
    },
#ifndef BUILD_FEATURE_NO_SPI_FLASH
    // flash_spi
    {
        .init = NULL,
        .request = nlflash_spi_request,
        .release = nlflash_spi_release,
        .flush = nlflash_spi_flush,
        .read_id = nlflash_spi_read_id,
        .get_info = nlflash_spi_get_info,
        .erase = nlflash_spi_erase,
        .read = nlflash_spi_read,
        .write = nlflash_spi_write,
    },
#endif
};

void nlplatform_block_sleep(bool block)
{
    nlplatform_interrupt_disable();
    if (block)
    {
#if defined(BUILD_FEATURE_NL_PROFILE) && !defined(NL_NO_RTOS)
        if (sBlockSleepCount == 0)
        {
            nl_profile_start(NL_PROFILE_DEEP_SLEEP_BLOCKED);
        }
#endif
        sBlockSleepCount++;
    }
    else
    {
        sBlockSleepCount--;
        nlASSERT(sBlockSleepCount >= 0);
#if defined(BUILD_FEATURE_NL_PROFILE) && !defined(NL_NO_RTOS)
        if (sBlockSleepCount == 0)
        {
            nl_profile_stop(NL_PROFILE_DEEP_SLEEP_BLOCKED);
        }
#endif
    }
    nlplatform_interrupt_enable();
}

bool nlplatform_is_sleep_blocked(void)
{
    return (sBlockSleepCount != 0);
}

#if defined(BUILD_FEATURE_SW_TIMER) && !defined(NL_NO_RTOS)
static uint32_t blk_sleep_tmr_cb(nl_swtimer_t *timer, void *arg)
{
    nlplatform_block_sleep(false);
    return 0;
}

void nlplatform_block_sleep_ms(unsigned ms)
{
    nl_time_native_t curr;
    nl_time_native_t block_time_ticks;
    nl_time_native_t end;

    block_time_ticks = nl_time_ms_to_delay_time_native(ms);

    /* Use a critical section so that the timer cannot expire nor system time
     * elapse while its determined whether a timer restart is needed. */
    nlplatform_interrupt_disable();

    curr = xTaskGetTickCount();
    end = curr + block_time_ticks;

    if (nl_swtimer_is_active(&s_blk_sleep_tmr.timer))
    {
        if (block_time_ticks > (s_blk_sleep_tmr.end - curr))
        {
            /* A longer duration of sleep-blocking than currently scheduled is
             * needed. */
            s_blk_sleep_tmr.end = end;
            nl_swtimer_cancel(&s_blk_sleep_tmr.timer);
            nl_swtimer_start(&s_blk_sleep_tmr.timer, ms);
        }

    }
    else
    {
        nlplatform_block_sleep(true);
        s_blk_sleep_tmr.end = end;
        nl_swtimer_start(&s_blk_sleep_tmr.timer, ms);
    }

    nlplatform_interrupt_enable();
}
#endif

/* For FreeRTOS V10.0.1, the base of the stack is
 * at offset 48 into the TCB:
 *     (gdb) p/d &((tskTCB *)0)->pxStack
 *     $16 = 48
 */
#define TCB_OFFSET_PX_STACK 48

#if !defined(NL_BOOTLOADER)
void nlplatform_soc_dump_context(void)
{
    printf("- SOC context: -\n");

#ifdef BUILD_FEATURE_WATCHPOINT_STACK_GUARD
    // Dump watchpoint state
    TaskHandle_t currentTaskHandle = xTaskGetCurrentTaskHandle();
    if (currentTaskHandle)
    {
        uint32_t stackBaseAddress = *(uint32_t*)((uint32_t)(currentTaskHandle) + TCB_OFFSET_PX_STACK);
        printf("CurrentTask = '%s', stackBase = 0x%x\n", pcTaskGetName(currentTaskHandle), stackBaseAddress);
    }

    printf("DWT->COMP3 = 0x%08x\n", DWT->COMP3);
    printf("DWT->MASK3 = 0x%08x\n", DWT->MASK3);
    printf("DWT->FUNCTION3 = 0x%08x\n", DWT->FUNCTION3);
#endif

    // Dump additional debug halt reason registers
    printf("DEMCR = 0x%x\n", CoreDebug->DEMCR);
}

void nlplatform_configure_stack_guard(void)
{
    TaskHandle_t currentTaskHandle = xTaskGetCurrentTaskHandle();
    uint32_t stackBaseAddress = *(uint32_t*)((uint32_t)(currentTaskHandle) + TCB_OFFSET_PX_STACK);
    (void)stackBaseAddress;
#ifdef BUILD_FEATURE_MPU_STACK_GUARD
    halInternalSetMPUStackGuardRegionStart(stackBaseAddress);
#endif
#ifdef BUILD_FEATURE_WATCHPOINT_STACK_GUARD
    // double check that the stackBaseAddress is 32-byte aligned, so the mask we
    // pass to nl_watchpoint_enable() actually works in guarding the first 32 bytes
    // of the task stack
    assert((stackBaseAddress % 32) == 0);

    // use the last watchpoint to guard the first 32-bytes of the current task stack
    nl_watchpoint_enable(TASK_STACK_WATCHPOINT_INDEX, stackBaseAddress, WATCHPOINT_ON_WRITE, 0x5);
#endif
}
#endif /* !defined(NL_BOOTLOADER) */


#if !defined(NL_BOOTLOADER)

/* Implementation of system tick and vPortSuppressTicksAndSleep() ported from Nordic SDK.
 * The FreeRTOS port uses the RTC and not SysTick as the tick source.
 * I believe this is because the RTC keeps running during SOC sleep
 * whereas SysTick does not.
 *
 * RTC is a 24-bit counter, which we're configuring to run at 250Hz (4ms tick).
 */

/* Count of extra ticks slept.  Particularly when stepping through there
 * sleep code in a debugger, the count of ticks slept can be much higher
 * than waht was requested.  FreeRTOS is unhappy if vTaskStepTick() is
 * given a count that is higher than it requested.  So, we keep track
 * of the extra ticks in this global and feed them back to the kernel
 * over time in subsequent tick irqs.
 */
static uint32_t extra_sleep_ticks;

extern void xPortSysTickHandler(void);

void nlrtc1_isr ( void );
void nlrtc1_isr ( void )
{
    NRF_RTC1->EVENTS_TICK = 0;
    NRF_RTC1->EVENTS_COMPARE[0] = 0;

    xPortSysTickHandler();

    /* Feed one extra sleep tick, if any.
     * Note we do not read the counter register and try to see
     * if any ticks were missed due to late/delayed RTC1_IRQ.
     * It's not expected to have interrupt latency over 4ms (the
     * default tick - some systems might have an even longer tick
     * like 10ms), and it's somewhat expensive to read the COUNTER
     * register due to need for synchronization between the CPU clock
     * and the lfclk that drives the RTC
     */
    if (extra_sleep_ticks > 0)
    {
        extra_sleep_ticks--;
        xPortSysTickHandler();
    }
}

/*
 * Setup the RTC to generate the tick interrupts at the required
 * frequency, overriding weak definition in freertos
 */
void vPortSetupTimerInterrupt( void );
void vPortSetupTimerInterrupt( void )
{
    /* Configure RTC1 to interrupt at the requested rate. */
    NRF_RTC1->PRESCALER = NRF_RTC_PRESCALER_VALUE;
    NRF_RTC1->INTENSET = (RTC_INTENSET_TICK_Set << RTC_INTENSET_TICK_Pos);
    NRF_RTC1->TASKS_CLEAR = 1;
    NRF_RTC1->TASKS_START = 1;

    NVIC_EnableIRQ(RTC1_IRQn);
}

// Maximum tick count at 24-bits
#define portNRF_RTC_MAXTICKS ((1 << 24) - 1)

// Set to 1 to get some info about events before and after WFE
#define DEBUG_SLEEP_EVENTS 0

static void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
{
    /*
     * Implementation note:
     *
     * To help debugging the option configUSE_TICKLESS_IDLE_SIMPLE_DEBUG was presented.
     * This option would make sure that even if program execution was stopped inside
     * this function no more than expected number of ticks would be skipped.
     *
     * Normally RTC works all the time even if firmware execution was stopped
     * and that may lead to skipping too much of ticks.
     */
    TickType_t enterTime;

    /* We don't support a delay of 1 because the HW counter is free running and
     * there could be a race where the counter is incremented after we read the
     * current value, causing the CC value we set to be invalid (would never
     * trigger an interrupt until counter rolled over).
     */
    assert(xExpectedIdleTime > 1);

    /* Make sure the SysTick reload value does not overflow the counter. */
    if ( xExpectedIdleTime > portNRF_RTC_MAXTICKS - configEXPECTED_IDLE_TIME_BEFORE_SLEEP )
    {
        xExpectedIdleTime = portNRF_RTC_MAXTICKS - configEXPECTED_IDLE_TIME_BEFORE_SLEEP;
    }
    /* Block all the interrupts globally */
#ifdef SOFTDEVICE_PRESENT
    do{
        uint8_t dummy = 0;
        uint32_t err_code = sd_nvic_critical_region_enter(&dummy);
        APP_ERROR_CHECK(err_code);
    }while(0);
#else
    nlplatform_interrupt_disable();
#endif

    enterTime = NRF_RTC1->COUNTER;

    if ( eTaskConfirmSleepModeStatus() != eAbortSleep )
    {
        TickType_t xModifiableIdleTime;
        TickType_t wakeupTime = (enterTime + xExpectedIdleTime) & portNRF_RTC_MAXTICKS;

        /* Stop tick events */
        NRF_RTC1->INTENCLR = RTC_INTENCLR_TICK_Clear;

        /* Configure compare interrupt for our wakeupTime */
        NRF_RTC1->CC[0] = wakeupTime;
        NRF_RTC1->EVENTS_COMPARE[0] = 0;
        NRF_RTC1->INTENSET = (RTC_INTENSET_COMPARE0_Set << RTC_INTENSET_COMPARE0_Pos);

        /* CryptoCell appears to be setting a pending interrupt bit in ISPR, even though we
         * don't enable it. Clear it to prevent it from keeping us from sleeping.
         */
        if (NVIC_GetPendingIRQ(CRYPTOCELL_IRQn))
        {
            NVIC_ClearPendingIRQ(CRYPTOCELL_IRQn);
        }

        /* Suspend uart */
        nluart_suspend();

#if DEBUG_SLEEP_EVENTS
        uint32_t pre_wfe_ispr0;
        uint32_t pre_wfe_ispr1;
        uint32_t post_wfe_ispr0;
        uint32_t post_wfe_ispr1;
        uint32_t wfe_count = 0;
#endif

        __DSB();

        /* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
         * set its parameter to 0 to indicate that its implementation contains
         * its own wait for interrupt or wait for event instruction, and so wfi
         * should not be executed again.  However, the original expected idle
         * time variable must remain unmodified, so a copy is taken. */
        xModifiableIdleTime = xExpectedIdleTime;
        configPRE_SLEEP_PROCESSING( xModifiableIdleTime );

#if DEBUG_SLEEP_EVENTS
        pre_wfe_ispr0 = NVIC->ISPR[0];
        pre_wfe_ispr1 = NVIC->ISPR[1];
#endif
        if ( xModifiableIdleTime > 0 )
        {
            bool enable_ITM_DWT_after_sleep = false;

            // disable MPU that was guarding main stack to save power
            nl_mpu_enable(false, false, false);

#ifdef BUILD_FEATURE_WATCHPOINT_STACK_GUARD
            // suspend task stack guard watchpoint to save power
            nl_watchpoint_disable(TASK_STACK_WATCHPOINT_INDEX);
#endif

            if (!(CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk))
            {
                /* no debugger connected, disable ITM/DWT to save power.
                 * note that other users of ITM/DWT are:
                 *  - watchpoint, which is the most common user, but
                 *    we just disabled it
                 *  - virtual uart, which should be okay to suspend
                 *    in sleep
                 *  - CYCCNT for watchdog when debugger is connected,
                 *    but we know debugger is not connected so CYCCNT
                 *    should not be in use
                 */
                if (CoreDebug->DEMCR & (CoreDebug_DEMCR_TRCENA_Msk | CoreDebug_DEMCR_MON_EN_Msk))
                {
                    enable_ITM_DWT_after_sleep = true;
                    CoreDebug->DEMCR &= ~(CoreDebug_DEMCR_TRCENA_Msk | CoreDebug_DEMCR_MON_EN_Msk);
                }
            }

            // note: instruction cache is automatically powered down in sleep

            do
            {
#if DEBUG_SLEEP_EVENTS
                wfe_count++;
#endif
                __WFE();
            } while (0 == (NVIC->ISPR[0] | NVIC->ISPR[1]));

            // enable ITM/DWT if we had disabled it during sleep
            if (enable_ITM_DWT_after_sleep)
            {
                CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk | CoreDebug_DEMCR_MON_EN_Msk;
            }

#ifdef BUILD_FEATURE_WATCHPOINT_STACK_GUARD
            // restore watchpoint that is guarding task stack
            nl_watchpoint_set_type(TASK_STACK_WATCHPOINT_INDEX, WATCHPOINT_ON_WRITE);
#endif

            // enable MPU guarding main stack
            nl_mpu_enable(true, true, true);
        }
#if DEBUG_SLEEP_EVENTS
        post_wfe_ispr0 = NVIC->ISPR[0];
        post_wfe_ispr1 = NVIC->ISPR[1];
#endif

        configPOST_SLEEP_PROCESSING( xExpectedIdleTime );
        NRF_RTC1->INTENCLR = (RTC_INTENCLR_COMPARE0_Clear << RTC_INTENCLR_COMPARE0_Pos);
        NRF_RTC1->EVENTS_COMPARE[0] = 0;

        /* resume_uart */
        nluart_resume();

#if DEBUG_SLEEP_EVENTS
        printf("%s: after WFE (count = %u), pre_wfe_ispr0 = 0x%08x, pre_wfe_ispr1 = 0x%08x, post_wfe_ispr0 = 0x%08x, post_wfe_ispr1 = 0x%08x\n",
            __func__, wfe_count, pre_wfe_ispr0, pre_wfe_ispr1, post_wfe_ispr0, post_wfe_ispr1);
#endif

        /* Reenable tick interrupt and update tick count after sleep */
        {
            TickType_t diff;
            TickType_t exitTime;

            NRF_RTC1->EVENTS_TICK = 0;
            NRF_RTC1->INTENSET = (RTC_INTENSET_TICK_Set << RTC_INTENSET_TICK_Pos);

            exitTime = NRF_RTC1->COUNTER;

            // TickType_t is 32-bit, but register is 24-bit, so handle
            // wrapping by & with max 24-bit value
            diff = (exitTime - enterTime) & portNRF_RTC_MAXTICKS;

            /* It is important that we clear pending here so that our
             * corrections are latest and in sync with tick_interrupt handler.
             * Otherwise we could immediately get a IRQ and add an extra tick
             * after sleep.
             */
            NVIC_ClearPendingIRQ(RTC1_IRQn);

#if DEBUG_SLEEP_EVENTS
            printf("%s: sleep resumed after %u ticks\n", __func__, diff);
#endif

            if (diff > 0)
            {
                /* FreeRTOS asserts if the actual sleep time was longer than
                 * requested. This can happen when using a debugger.
                 * Save the extra ticks in a global that we feed back
                 * to FreeRTOS in the RTC IRQHandler.
                 */
                if (diff > xExpectedIdleTime)
                {
                    extra_sleep_ticks += diff - (xExpectedIdleTime - 1);
                    diff = xExpectedIdleTime - 1;
                    /* Pend the nlrtc1_isr() immediately to start
                     * processing the extra ticks
                     */
                    NVIC_SetPendingIRQ(RTC1_IRQn);
                }
                vTaskStepTick(diff);
            }
        }
    }
#ifdef SOFTDEVICE_PRESENT
    uint32_t err_code = sd_nvic_critical_region_exit(0);
    APP_ERROR_CHECK(err_code);
#else
    nlplatform_interrupt_enable();
#endif
}

void nlplatform_suppress_ticks_and_sleep(uint32_t xExpectedIdleTime);
void nlplatform_suppress_ticks_and_sleep(uint32_t xExpectedIdleTime)
{
#ifdef BUILD_FEATURE_SW_TIMER_USES_RTOS_TICK
    bool timer_sleep_ok;
    TickType_t before_sleep_tick_count;
#endif

#ifdef BUILD_FEATURE_SW_TIMER_USES_RTOS_TICK
    timer_sleep_ok = nl_swtimer_pre_sleep(&before_sleep_tick_count, &xExpectedIdleTime);
    // FreeRTOS imposes a minimum tick time before sleep of configEXPECTED_IDLE_TIME_BEFORE_SLEEP
    // (defaults to 2, but could be higher), but nl_swtimer_pre_sleep()
    // might have changed xExpectedIdleTime to be less than that. We want to still
    // enforce the FreeRTOS configured minimum. It's expecially bad if the value
    // is 1 because there could be a race when computing the wakeup time if
    // the underlying HW counter increments right after we read the current
    // value because the setting for the wakeup interrupt might miss the
    // counter increment and we won't get the CC match until the counter
    // wraps!
    if (timer_sleep_ok && !nlplatform_is_sleep_blocked() && xExpectedIdleTime > configEXPECTED_IDLE_TIME_BEFORE_SLEEP)
    {
#endif

#if defined(BUILD_FEATURE_NL_PROFILE) && !defined(NL_NO_RTOS)
        nl_profile_start(NL_PROFILE_DEEP_SLEEP);
#endif
        vPortSuppressTicksAndSleep(xExpectedIdleTime);
#if defined(BUILD_FEATURE_NL_PROFILE) && !defined(NL_NO_RTOS)
        nl_profile_stop(NL_PROFILE_DEEP_SLEEP);
#endif

#ifdef BUILD_FEATURE_SW_TIMER_USES_RTOS_TICK
    }
    nl_swtimer_post_sleep(before_sleep_tick_count);
#endif
}
#endif // NL_BOOTLOADER

bool nlplatform_bootloader_is_locked(void)
{
    // a value of 0x00 for APPROTECT means the bootloader is locked
    return (NRF_UICR->APPROTECT == 0x00);
}

void nlplatform_acl_configure(nlacl_id_t acl_id, unsigned region_start_addr, unsigned region_size, unsigned permission)
{
    assert(acl_id <= MAX_ACL_ID);
    // assert that region start addr is aligned on flash page boundary
    assert((region_start_addr & (NRF52X_INTERNAL_FLASH_ERASE_SIZE - 1)) == 0);
    // assert that region_size is not zero and is a multiple of the flash page size
    assert((region_size > 0) && ((region_size & (NRF52X_INTERNAL_FLASH_ERASE_SIZE - 1)) == 0));
    // assert that permission is valid. since 0 is no protection, which is already the
    // default, it's probably an error to request that so don't allow no protection
    assert((permission == NL_ACL_PERMISSION_BLOCK_WRITE_ERASE) ||
           (permission == NL_ACL_PERMISSION_BLOCK_EXECUTE_READ) ||
           (permission == NL_ACL_PERMISSION_BLOCK_ALL));
    // all registers are write-once per boot so check that they are currently in reset state
    assert(NRF_ACL->ACL[acl_id].ADDR == 0);
    assert(NRF_ACL->ACL[acl_id].SIZE == 0);
    assert(NRF_ACL->ACL[acl_id].PERM == 0);

    // now do the actual writes to the configuration registers
    NRF_ACL->ACL[acl_id].ADDR = region_start_addr;
    NRF_ACL->ACL[acl_id].SIZE = region_size;
    NRF_ACL->ACL[acl_id].PERM = permission;

    // Do two dummy reads to so new permissions are enforced by the time function returns.
    // OPS says enforcement takes two clock cycles after last register is written.
    (void)NRF_ACL->ACL[acl_id].ADDR;
    (void)NRF_ACL->ACL[acl_id].ADDR;
}

// If APPROTECT is enabled, disable it by erasing UICR. We backup
// and restore to the UICR all other fields except the APPROTECT,
// and also set the CUSTOMER[UNLOCK_UICR_CUSTOMER_INDEX] to indicate
// we're now in a pesistent unlocked state (so that next call
// of nlplatform_external_debugger_access_disable() will do nothing)
void nlplatform_external_debugger_access_enable(void *scratch_buffer, size_t scratch_buffer_size)
{
    if (NRF_UICR->APPROTECT == UICR_APPROTECT_PALL_Enabled)
    {
        // enable external debugger access by erasing UICR and restoring
        // all contents of the UICR except set APPROTECT to disabled and
        // set CUSTOMER[UNLOCK_UICR_CUSTOMER_INDEX] to UNLOCK_VALUE_UNLOCK_TOKEN_RECEIVED
        int retval;
        size_t retlen;
        NRF_UICR_Type *uicr_backup_buf = (NRF_UICR_Type*)scratch_buffer;
        assert(scratch_buffer_size >= NRF52X_INTERNAL_FLASH_ERASE_SIZE);

        // copy current UICR to RAM buffer
        memcpy(uicr_backup_buf, NRF_UICR, NRF52X_INTERNAL_FLASH_ERASE_SIZE);

        // set APPROTECT in the backup to disabled
        uicr_backup_buf->APPROTECT = UICR_APPROTECT_PALL_Disabled;

        // set CUSTOMER[UNLOCK_UICR_CUSTOMER_INDEX] to UNLOCK_VALUE_UNLOCK_TOKEN_RECEIVED
        uicr_backup_buf->CUSTOMER[UNLOCK_UICR_CUSTOMER_INDEX] = UNLOCK_VALUE_UNLOCK_TOKEN_RECEIVED;

        // enable erase
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
        // erase UICR
        NRF_NVMC->ERASEUICR = NVMC_ERASEUICR_ERASEUICR_Erase;
        // wait for erase to finish
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
        // go back to read enable
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;

        // write UICR we saved/modified earlier
        retval = nlflash_write(NLFLASH_INTERNAL, NRF_UICR_BASE, NRF52X_INTERNAL_FLASH_ERASE_SIZE,
                               &retlen, (const uint8_t*)uicr_backup_buf, NULL);
        (void)retval;
        assert(retval == 0);
    }
}

// checks the APPROTECT register in UICR. If it is not enabled and the UICR
// customer register we use to indicate a device is in a persisent
// unlocked state is not set (i.e. device was sent an unlock token)
// then enable APPROTECT and reboot to enforce new APPROTECT setting
// (according to Nordic OPS manual, changes to UICR only take effect after a reset).
void nlplatform_external_debugger_access_disable(void)
{
    if ((NRF_UICR->APPROTECT != UICR_APPROTECT_PALL_Enabled) &&
        (NRF_UICR->CUSTOMER[UNLOCK_UICR_CUSTOMER_INDEX] == UNLOCK_VALUE_NO_UNLOCK_TOKEN_RECEIVED))
    {
        // APPROTECT not enabled and not set to unlock, enable it now by
        // setting APPROTECT to UICR_APPROTECT_PALL_Enabled

        // Enable write access to UICR/internal FLASH
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
        // Set APPROTECT to enable (which blocks out external debuggers)
        NRF_UICR->APPROTECT = UICR_APPROTECT_PALL_Enabled;
        // Wait for write to finish
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
        // Reset so new value is applied
        nlplatform_reset(NL_RESET_REASON_SW_REQUESTED);
        // Never gets here
    }
}

bool nlplatform_get_external_debugger_access(void)
{
    return NRF_UICR->APPROTECT != UICR_APPROTECT_PALL_Enabled;
}
