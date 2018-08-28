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
 *      Nordic NRF52840 WatchDog Driver.
 *
 */
#include <stdint.h>
#include <stdio.h>

#include <nlplatform.h>
#include <nlplatform/nlwatchdog.h>

#include <nrf52x/device/nrf52840.h>
#include <nrf52x/device/nrf52840_bitfields.h>

#include <FreeRTOS.h>
#include <task.h>

/*******************************************************************************
 * !!!!----                  IMPLEMENTATION DETAILS                  ----!!!!
 ******************************************************************************/
/*  Upon construction, the hardware WatchDog is automatically enabled if
 * NL_DISABLE_WATCHDOG_OPERATION is undefined and will trigger within a
 * fixed duration unless it is reset. Resetting or the WatchDog may be
 * accomplished in two ways:
 *
 *  -Calling WatchDog::refresh()
 *  -Using the SoftDog task tracking mechanism.
 *
 *  The latter of the two options above is the intended use case. At start,
 * each task which requires monitoring by the SoftDog must register itself
 * through a call to WatchDog::allocateId(), which (if availble) will return
 * an identification number for the task to use.
 *  The task may then enable and disable SoftDog tracking through a call to
 * WatchDog::enabletracking(bool inEnable, uint32_t uiD), and perform refresh
 * requests through calls to WatchDog::refreshFlag(uint32_t uiD). The hardware
 * WatchDog will only be reset when all tasks have either disabled tracking or
 * performed refresh requests.
 *
 *  A "usual" FreeRTOS task may utilize the SoftDog as shown below, where the
 * given task's monitoring is enabled at the start of one run iteration, and
 * then disabled at its end:
 *
 * uint32_t uiD;
 *
 * uiD = nl_wdt_allocate_id();
 * for(;;)
 * {
 *     nl_wdt_enable_tracking(true, uiD);
 *     ...
 *     nl_wdt_enable_tracking(false, uiD);
 * }
 *
 *  Note that either enabling or disabling a task will automatically perform
 * a refresh for said task's ID. So WatchDog::refreshFlag(uint32_t uiD) should
 * only be required for tasks which have iteration times expected to exceed 2s.
 *
 *  Two modes of operation are provided, exclusive and non-exclusive. In exclusive
 * mode (#define NL_WATCHDOG_REQUIRE_ENROLLMENT) the WatchDog will only enable itself
 * when at least one task has been registered for tracking. In non-exclusive mode,
 * the WatchDog will always be running regardless of whether a task is enrolled
 * or not. The latter case is useful for environments where all tasks are
 * expected to op-in to using the WatchDog.
 *  Internally, all monitored tasks just set and clear reserved entries in a
 * bitfield, and once all placation / reset bits have been cleared the hardware
 * WatchDog will actually be reset.
 *
 * ---- SLEEP SAFE MONITORING ----
 *  As the hardware WatchDog is frozen upon entry to DEEP SLEEP, it is desirable
 * to make sure that all tasks are "sleep safe," in that they're not currently
 * blocked waiting on another task or in any region of their execution iteration
 * other than the loop point.
 *  Due to the expected "usual" FreeRTOS task usage of the WatchDog supplied
 * above, we're considering a "sleep safe" condition to be where all tasks have
 * disabled their WatchDog tracking features. This situation may be checked
 * through a call to WatchDog::isSleepSafe().
 */

/*******************************************************************************
 * !!!!----                 MODULE SHARED / GLOBALS                  ----!!!!
 ******************************************************************************/
uint32_t __attribute__ ((section(".noinit")))   nl_watchDog_lastFlags;

static uint32_t sFlags;
static uint32_t sAllFlags;
static uint8_t sCurrentId;

#ifndef WATCHDOG_TIMEOUT_IN_SECONDS
#define WATCHDOG_TIMEOUT_IN_SECONDS 12
#endif

/**
 * @fn void nlwatchdog_test(void)
 * @details Test Watchdog functionality by first enabling it and then heading
 * off to nlwatchdog_stop(), hopefully followed by a reset.
 *----------------------------------------------------------------------------*/
void nlwatchdog_test(void)
{
    nlwatchdog_set_enable(true);
    // to help with timing how long a watchdog takes to trigger,
    // refresh right before we spin.
    nlwatchdog_refresh();
    nlwatchdog_stop();
}

#if defined(WATCHDOG_RTC_CTRL)

/* The NRF52X Watchdog HW block is very flawed and will not provide very useful
 * information when it trips so it's better to use one of the RTCs.
 * Watchdog HW block flaws include:
 *   - there is no way to pause/disable it once enabled
 *   - there is no way to change its configuration once enabled
 *   - a software reset doesn't reset the watchdog (so bootloader/early boot
 *     code needs to be aware that the watchdog might already be running)
 *   - the early warning interrupt mechanism is a regular interrupt and not a NMI
 *   - the early warning interrupt only allows for 2 clks at 32768Hz (about 60us)
 *     before the watchdog reset occurs, which isn't enough time to do much
 *   - RAM is reset by the watchdog interrupt, so even if the early warning
 *     interrupt could save all state to RAM, there's no guarantee that the RAM
 *     contents will be valid on next boot
 *
 * It's just better not to use the HW Watchdog block given its flaws. Instead,
 * we'll use a RTC, which is even lower power, and has almost none of the flaws
 * listed above. Two concerns when using a RTC timer for watchdog functionality:
 *   - We can't configure the RTC timer as a NMI, so runaway code that leaves
 *     interrupts disabled won't get caught by the RTC based watchdog functionality.
 *     We can mitigate this by changing the nlplatform_disable_interrupt() function to
 *     be like the FreeRTOS functions, where we change BASEPRI instead of PRIMASK.
 *     We set RTC timer interrupt priority to 0 (highest priority configurable interrupt)
 *     and outside of the range that is disabled by nlplatform_disable_interrupt(), and
 *     make sure no one sets PRIMASK directly. Then the only code that can block the
 *     RTC interrupt would be the HardFault handler, which has fixed priority of -1.
 *   - We can't disable RTC timer automatically when a debugger is connected.
 *     This could be mitigated by using openocd/gdb hooks to disable/enable the RTC
 *     timer used whenever we halt/resume the processor, but that requires everyone
 *     to have the right hooks setup. Instead we mitigate this case by using the
 *     WDT->CYCCNT to count the number of processor clocks since the last time the
 *     watchdog was refreshed. If the CYCCNT value isn't about the value expected
 *     for the given timeout, then we assume this is because the debugger had stopped
 *     the processor and so we self-refresh the RTC timer. We've tested that the
 *     CYCCNT does count in WFE, so sleep with debugger connected does trip the
 *     watchdog as expected.
 */

#define kMaxIds         32

static bool s_check_cycle_counter = false;

/* The minimum number of cycles we expected to have run since
 * last time we refreshed the RTC timer. It should be related
 * to the RTC timeout.
 * At 64MHz we expect roughly 64000000 cycles per second,
 * so MIN_CYCCNT_FOR_WATCHDOG_RESET should be
 * 64000000 * WATCHDOG_TIMEOUT_IN_SECONDS.
 * When tested, we found that less than 64000000 cycles were
 * run in 12 seconds, maybe due to inaccuracy of the HCLK
 * or some other reason. We'll use a more conservative
 * number like 63000000, which appears to work.
 *
 * Note that since CYCCNT register is 32-bits, this limits
 * the maximum value for WATCHDOG_TIMEOUT_IN_SECONDS that
 * we can verify using the CYCCNT to about 68 seconds.
 */
#define MIN_CYCLES_PER_SECOND 63000000
#if (WATCHDOG_TIMEOUT_IN_SECONDS > (0xffffffff/MIN_CYCLES_PER_SECOND))
#error WATCHDOG_TIMEOUT_IN_SECONDS too large to check with CYCCNT
#endif
#define MIN_CYCCNT_FOR_WATCHDOG_RESET (MIN_CYCLES_PER_SECOND * WATCHDOG_TIMEOUT_IN_SECONDS)

/*******************************************************************************
 * !!!!----             CONSTRUCTOR AND DESTRUCTOR                   ----!!!!
 ******************************************************************************/
void nlwatchdog_init(void)
{
    /* Here it would be good to assert that the DWT TRCENA and MON_EN have both
     * been enabled by nlplatform as this module depends on them. However,
     * MON_EN can be disabled by the debugger because it uses a different
     * mechanism for its breakpoints and watchpoints. That will cause the
     * assert to falsely trip. The ensuing confusion from such a false alarm
     * outweighs the benefit of the assert at this point. */

    /* Check if a debugger is connected. If it is, attempt to use the DWT
     * cycle counter to track how many instructions excecuted since
     * last watchdog refresh. We expect it to be quite high except
     * if debugger is being used.
     */
    if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk)
    {
        /* Assert that the CYCCNT is supported, otherwise this won't
         * work and debugging with the RTC implementation of watchdog
         * will be a very bad experience.
         */
        assert((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) == 0);

        // Enable CYCCNT and set global that we should be checking
        // CYCCNT whenever the RTC interrupt fires.
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        s_check_cycle_counter = true;
    }
#if (defined(NL_DISABLE_WATCHDOG_OPERATION) || (defined(NL_WATCHDOG_REQUIRE_ENROLLMENT) && !defined(NL_NO_RTOS)))
    nlwatchdog_set_enable(false);
#else
    nlwatchdog_set_enable(true);
#endif
}

/*******************************************************************************
 * !!!!----                  Local critical section                     ----!!!!
 ******************************************************************************/
// We cannot use nlplatform_interrupt_disable() to implement a critical section
// for this watchdog implementation since nlplatform_interrupt_disable() only
// disables interrupts at priorities lower than the priority being used for
// the RTC interrupt we're using. So instead, configure BASEPRI directly to
// block interrupt priorities at or below WATCHDOG_IRQ_PRIORITY if it's
// priority is non-zero. If WATCHDOG_IRQ_PRIORITY is 0, have to use
// PRIMASK because setting BASEPRI to 0 is equivalent to not masking.
#if WATCHDOG_IRQ_PRIORITY == 0
#define WATCHDOG_CRITICAL_SECTION_START() { uint32_t old_primask = __get_PRIMASK(); __set_PRIMASK(1)
#define WATCHDOG_CRITICAL_SECTION_END()   __set_PRIMASK(old_primask); }
#else
#define WATCHDOG_CRITICAL_SECTION_START() { uint32_t old_base_pri = __get_BASEPRI(); __set_BASEPRI(WATCHDOG_IRQ_PRIORITY << (8 - __NVIC_PRIO_BITS))
#define WATCHDOG_CRITICAL_SECTION_END()   __set_BASEPRI(old_base_pri); }
#endif // WATCHDOG_IRQ_PRIORITY == 0

/*******************************************************************************
 * !!!!----         DEVICE INITIALIZATION AND STATE CONTROL          ----!!!!
 ******************************************************************************/

/**
 * @fn void watchdog_internal_enable_locked(void)
 * @details Internal function to enable RTC used as the watchdog
 *----------------------------------------------------------------------------*/
static void watchdog_internal_enable_locked(void)
{
    NRF_RTC_Type *rtc_ctrl = WATCHDOG_RTC_CTRL;
    if (rtc_ctrl->INTENSET == 0)
    {
        /* Make sure RTC timer is stopped */
        rtc_ctrl->TASKS_STOP = 1;

        /* Set timeout. Since resolution we need is typically just seconds,
         * use the largest PRESCALER value. PRESCALER register is 12-bits,
         * and setting it to 0xfff gives a COUNTER frequency of
         *      32768/(PRESCALER+1) = 8Hz, or 125ms
         */
        rtc_ctrl->PRESCALER = RTC_PRESCALER_PRESCALER_Msk;
        rtc_ctrl->CC[0] = WATCHDOG_TIMEOUT_IN_SECONDS * 8;

        /* Enable RTC compare interrupt */
        rtc_ctrl->INTENSET = (RTC_INTENSET_COMPARE0_Set << RTC_INTENSET_COMPARE0_Pos);
        nlplatform_irq_enable(WATCHDOG_IRQ_NUM);

        /* Start the RTC */
        rtc_ctrl->TASKS_CLEAR = 1;
        rtc_ctrl->TASKS_START = 1;

        /* Clear cycle counter if in use */
        if (s_check_cycle_counter)
        {
            DWT->CYCCNT = 0;
        }
    }
}

/**
 * @fn void watchdog_internal_disable_locked(void)
 * @details Internal function to disable HW watchdog
 *----------------------------------------------------------------------------*/
static void watchdog_internal_disable_locked(void)
{
    NRF_RTC_Type *rtc_ctrl = WATCHDOG_RTC_CTRL;
    nlplatform_irq_disable(WATCHDOG_IRQ_NUM);
    rtc_ctrl->INTENCLR = (RTC_INTENCLR_COMPARE0_Clear << RTC_INTENCLR_COMPARE0_Pos);
    rtc_ctrl->TASKS_STOP = 1;
}

/**
 * @fn void nlwatchdog_set_enable(bool enable)
 * @details Enable or disable the hardware watchdog based upon enable.
 *----------------------------------------------------------------------------*/
void nlwatchdog_set_enable(bool enable)
{
    WATCHDOG_CRITICAL_SECTION_START();
    if (enable)
    {
        watchdog_internal_enable_locked();
    }
    else
    {
        watchdog_internal_disable_locked();
    }
    WATCHDOG_CRITICAL_SECTION_END();
}

/**
 * @fn bool nlwatchdog_is_enabled(void)
 * @details Returns the enabled state of the watchdog.
 *----------------------------------------------------------------------------*/
bool nlwatchdog_is_enabled(void)
{
    return (WATCHDOG_RTC_CTRL->INTENSET != 0);
}

/**
 * @fn void nlwatchdog_refresh(void)
 * @details Reset the RTC used for watchdog functionality.
 *----------------------------------------------------------------------------*/
void nlwatchdog_refresh(void)
{
    NRF_RTC_Type *rtc_ctrl = WATCHDOG_RTC_CTRL;
    rtc_ctrl->TASKS_CLEAR = 1;

    /* Clear cycle counter if in use */
    if (s_check_cycle_counter)
    {
        DWT->CYCCNT = 0;
    }
}

/**
 * @fn void nlwatchdog_stop(void)
 * @details Disable interrupts and lock the CPU until the watchdog (hopefully)
 * notices that it hasn't been pet lately and comes to reset us.
 *----------------------------------------------------------------------------*/
void nlwatchdog_stop(void)
{
    // block all other interrupts except our own priority
    nlplatform_interrupt_disable();
    for (; ; ) ;
}

/*
 * @fn bool nlwatchdog_ignore_pre_watchdog_isr(void)
 * @details Checks if debugger is connected and "active". Don't trigger the
 * watchdog if not many instructions have been run. This tries to make our
 * RTC timer based watchdog be less disruptive during debugging sessions
 * since we don't halt the RTC timer when the processor is halted in debug.
 *----------------------------------------------------------------------------*/
bool nlwatchdog_ignore_pre_watchdog_isr(void)
{
    bool ignore_watchdog_isr = false;

    if (s_check_cycle_counter)
    {
        /* Check if the cycle counter is sufficiently high that we believe
         * the debugger wasn't being actively used and should go ahead and
         * trigger the watchdog. Otherwise, refresh the RTC (but not cycle
         * counter) and try again next time the RTC interrupt occurs.
         */
        if (DWT->CYCCNT < MIN_CYCCNT_FOR_WATCHDOG_RESET)
        {
            /* Too few cycles since last refresh, maybe debugger
             * is being used. Reset RTC and try again.
             */
            NRF_RTC_Type *rtc_ctrl = WATCHDOG_RTC_CTRL;

            ignore_watchdog_isr = true;

            // clear irq so it will trigger again
            rtc_ctrl->EVENTS_COMPARE[0] = 0;
            nlplatform_irq_clear_pending(WATCHDOG_IRQ_NUM);

            // reset the timer
            rtc_ctrl->TASKS_CLEAR = 1;
        }
    }
    return ignore_watchdog_isr;
}


#else /* defined(WATCHDOG_RTC_CTRL) */

/* The NRF52X has 8 watchdog refresh enables, which we can use to
 * implement SoftDogs (per task tracking watchdog IDs).  Only
 * when all enabled refresh registers have their values set does
 * the HW watchdog reload it's CRV register.
 */
#define kMaxIds         8

/*******************************************************************************
 * !!!!----             CONSTRUCTOR AND DESTRUCTOR                   ----!!!!
 ******************************************************************************/
void nlwatchdog_init(void)
{
#if (!defined(NL_DISABLE_WATCHDOG_OPERATION) && !defined(NL_WATCHDOG_REQUIRE_ENROLLMENT))
    nlwatchdog_set_enable(true);
#endif
}

/*******************************************************************************
 * !!!!----                  Local critical section                     ----!!!!
 ******************************************************************************/
// When not using the RTC, local critical section can use the regular
// nlplatform_interrupt_disable/enable() calls.
#define WATCHDOG_CRITICAL_SECTION_START nlplatform_interrupt_disable
#define WATCHDOG_CRITICAL_SECTION_END   nlplatform_interrupt_enable

/*******************************************************************************
 * !!!!----         DEVICE INITIALIZATION AND STATE CONTROL          ----!!!!
 ******************************************************************************/
/**
 * @fn void watchdog_internal_enable_locked(void)
 * @details Internal function to enable HW watchdog
 *----------------------------------------------------------------------------*/
static void watchdog_internal_enable_locked(void)
{
    if (NRF_WDT->RUNSTATUS == 0)
    {
        /* Set timeout */
        NRF_WDT->CRV = (WATCHDOG_TIMEOUT_IN_SECONDS * 32768) - 1;
        /* Enable just one RR register.  This can't be changed
         * after the watchdog is enabled so not able to use the
         * other 7 RR registers given the dynamic nature of our
         * id allocation and enable/disabling.
         */
        NRF_WDT->RREN = 0x01;
        /* Configure so WDT halts if CPU is halted in debugger but not
         * if CPU is put to sleep.
         */
        NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos);
        /* Enable WDT timeout interrupt */
        NRF_WDT->INTENSET = WDT_INTENSET_TIMEOUT_Set;
        nlplatform_irq_enable(WDT_IRQn);

        /* Start the WDT */
        NRF_WDT->TASKS_START = 1;
    }
}

/**
 * @fn void watchdog_internal_disable_locked(void)
 * @details Internal function to disable HW watchdog
 *----------------------------------------------------------------------------*/
static void watchdog_internal_disable_locked(void)
{
    // nothing to do here, can't disable HW watchdog once enabled
}

/**
 * @fn void nlwatchdog_set_enable(bool enable)
 * @details Enable or disable the hardware watchdog based upon enable.
 *----------------------------------------------------------------------------*/
void nlwatchdog_set_enable(bool enable)
{
    WATCHDOG_CRITICAL_SECTION_START();
    if (enable)
    {
        watchdog_internal_enable_locked();
    }
    else
    {
        // can't disable on NRF52x, just pet it for now
        nlwatchdog_refresh();
    }
    WATCHDOG_CRITICAL_SECTION_END();
}

/**
 * @fn bool nlwatchdog_is_enabled(void)
 * @details Returns the enabled state of the watchdog.
 *----------------------------------------------------------------------------*/
bool nlwatchdog_is_enabled(void)
{
    return (NRF_WDT->RUNSTATUS != 0);
}

/**
 * @fn void nlwatchdog_refresh(void)
 * @details Placate the hardware watchdog.
 *----------------------------------------------------------------------------*/
void nlwatchdog_refresh(void)
{
    NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}

/**
 * @fn void nlwatchdog_stop(void)
 * @details Lock the CPU until the watchdog (hopefully)
 * notices that it hasn't been pet lately and comes to reset us.
 * Since WDT interrupt is not a NMI, must not block all interrupts.
 *----------------------------------------------------------------------------*/
void nlwatchdog_stop(void)
{
    for (; ; ) ;
}

#endif /* defined(WATCHDOG_RTC_CTRL) */

/*******************************************************************************
 * !!!!----                      ?SLEEP SAFE?                        ----!!!!
 ******************************************************************************/
 /**
 * @fn bool nlwatchdog_is_sleep_safe(void)
 * @details Returns whether or not the current SoftDog state is sleep-safe,
 * where sleep safe is considered to be a situation where all registered
 * tasks have disabled tracking.
 *----------------------------------------------------------------------------*/
bool nlwatchdog_is_sleep_safe(void)
{
    return (0 != sAllFlags) ? false : true;
}

/*******************************************************************************
 * !!!!----       TASK ENROLLMENT AND TRACKING (USER FACING)         ----!!!!
 ******************************************************************************/
/**
 * @fn int WatchDog::allocateId(void)
 * @details Allocate and return a new watchdog monitoring ID, which may be
 * used to track a particular task via enabletracking and refreshFlag. Will
 * return a negative value if no monitoring IDs are available for use.
 *----------------------------------------------------------------------------*/
int nlwatchdog_allocate_id(void)
{
    int id;

    WATCHDOG_CRITICAL_SECTION_START();
    id = (kMaxIds > sCurrentId) ? sCurrentId++ : -1;
    WATCHDOG_CRITICAL_SECTION_END();
    return id;
}

/**
 * @fn void nlwatchdog_enable_tracking(bool enable, unsigned id)
 * @details Enable or disable (via enable) tracking on a particular
 * monitoring ID, will also perform a refreshFlag operation automatically.
 *  Will also enable or disable the WatchDog itself based upon if any tasks
 * are enrolled for tracking or not (if one or more tasks are enrolled, the
 * WatchDog is enabled).
 *----------------------------------------------------------------------------*/
void nlwatchdog_enable_tracking(bool enable, unsigned id)
{
    if (id < sCurrentId)
    {
        WATCHDOG_CRITICAL_SECTION_START();
        if (enable)
        {
#if (!defined(NL_DISABLE_WATCHDOG_OPERATION) && defined(NL_WATCHDOG_REQUIRE_ENROLLMENT))
            if (0 == sAllFlags)
            {
                watchdog_internal_enable_locked();
            }
#endif
            sAllFlags |= (1 << id);
        }
        else
        {
            sAllFlags &= ~(1 << id);

#if (!defined(NL_DISABLE_WATCHDOG_OPERATION) && defined(NL_WATCHDOG_REQUIRE_ENROLLMENT))
            if (0 == sAllFlags)
            {
                watchdog_internal_disable_locked();
            }
#endif
        }
        WATCHDOG_CRITICAL_SECTION_END();

        nlwatchdog_refresh_flag(id);
    }
}

/**
 * @fn void nlwatchdog_refresh_flag(unsigned id)
 * @details Placate the software watchdog assigned to this particular task ID,
 * once all tasks have placated the software watchdog, the true hardware
 * watchdog will be reset through a call to refresh().
 *----------------------------------------------------------------------------*/
void nlwatchdog_refresh_flag(unsigned id)
{
    if (id < sCurrentId)
    {
        WATCHDOG_CRITICAL_SECTION_START();
        sFlags &= ~(1 << id);
        if (0 == (sFlags & sAllFlags))
        {
            nlwatchdog_refresh();
            sFlags = sAllFlags;
        }
        WATCHDOG_CRITICAL_SECTION_END();
    }

    nl_watchDog_lastFlags = sFlags;
}

/**
 * @fn void nlwatchdog_print_flags(void)
 * @details Print the flags in human readable format.
 *----------------------------------------------------------------------------*/
void nlwatchdog_print_flags(void)
{
    printf(UNIQUE_PRINTF_FORMAT_STRING("SoftDog State : \n"
                                       "\tRegistered IDs - %lu\n"
                                       "\tTracked Tasks - $%08lx\n"
                                       "\tPending Tasks - $%08lx\n"),
           sCurrentId, sAllFlags, sFlags);
}

/**
 * @fn void nlwatchdog_log_flags (char *dest, int len)
 * @details Store the flags in a minimal human readable format for logging.
 *----------------------------------------------------------------------------*/
void nlwatchdog_log_flags (char *dest, int len)
{
    snprintf(dest, len, UNIQUE_STRING_LITERAL("C:%d T:0x%08lx P:0x%08lx"), sCurrentId, sAllFlags, sFlags);
}
