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
 *      Driver for Nordic NRF52x HW timers.
 *
 *      Note: OPS 0.5.1 (latest version we have) has a number of bugs.
 *      1) STATUS register isn't actually implemented
 *      2) SHUTDOWN register is marked "deprecated" but actually has
 *         usefulness. Power can be reduced if the timer is SHUTDOWN,
 *         though the internal COUNTER will read back 0 if SHUTDOWN.
 *      3) Not necessarily a bug, but certainly not clear, a CC[]
 *         register used for COMPARE/EVENT should not be used for
 *         CAPTURE. CAPTURE grabs a snapshot of the internal COUNTER
 *         register to the CC[] register, but writing to CC[] for
 *         COMPARE doesn't not pass through to an internal COMPARE
 *         register. The CC[] register is the compare target, so
 *         if you CAPTURE to it while COMPARE is configured, it will
 *         change the COMPARE target.
 *
 *      Each TIMER has a number of Compare/Capture (CC) registers.
 *      HW instance TIMER0-TIMER2 have 4, TIMER3 and TIMER4 have 6.
 *      The CC registers aren't exposed directly.
 *      We use CC[0] for compare/target time for auto-repeat or
 *      one-shot stop time.
 *      We use CC[1] for capturing the current value of the internal
 *      COUNTER. Note that if the timer has been SHUTDOWN, capture
 *      will return 0.
 */
#include <nlplatform.h>
#include <nlplatform/nltimer.h>

#ifndef NL_BOOTLOADER
#include <FreeRTOS.h>
#endif

#include <nrf52x/device/nrf52840.h>
#include <nrf52x/device/nrf52840_bitfields.h>

// Mapping between timer_id_t to timer controller
typedef struct timer_config_s
{
    NRF_TIMER_Type *timer_controller;
    IRQn_Type irq_num;
} timer_config_t;

static const timer_config_t s_timer_config[] =
{
    {
        TIMER0_CONTROLLER,
        TIMER0_IRQ_NUM
    },
#if NL_NUM_TIMERS > 1
    {
        TIMER1_CONTROLLER,
        TIMER1_IRQ_NUM
    },
#endif
#if NL_NUM_TIMERS > 2
    {
        TIMER2_CONTROLLER,
        TIMER2_IRQ_NUM
    },
#endif
#if NL_NUM_TIMERS > 3
    {
        TIMER3_CONTROLLER,
        TIMER3_IRQ_NUM
    },
#endif
#if NL_NUM_TIMERS > 4
    {
        TIMER4_CONTROLLER,
        TIMER4_IRQ_NUM
    }
#endif
};

typedef struct timer_state_s
{
    nltimer_handler_t callback;
    void *context;
    bool in_use;
    bool configured;
    bool timer_running;
    bool shutdown;
} timer_state_t;

#define NLTIMER_CC_COMPARE   0
#define NLTIMER_CC_CAPTURE   1
#define NLTIMER_SHORTS_CLEAR_ON_COMPARE_MASK (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos)
#define NLTIMER_SHORTS_STOP_ON_COMPARE_MASK (TIMER_SHORTS_COMPARE0_STOP_Enabled << TIMER_SHORTS_COMPARE0_STOP_Pos)

static timer_state_t s_timer_state[NL_NUM_TIMERS];

#define ERROR_NOT_REQUESTED -2
#define ERROR_BAD_STATE     -1

void nltimer_init(void)
{
    // do nothing
}

int nltimer_request(nltimer_id_t timer_id)
{
    int err;
    timer_state_t *timer_state = &s_timer_state[timer_id];

    assert((unsigned)timer_id < NL_NUM_TIMERS);
    nlplatform_interrupt_disable();
    if (timer_state->in_use)
    {
        err = ERROR_BAD_STATE;
    }
    else
    {
        timer_state->in_use = true;
        timer_state->shutdown = true;
        err = 0;
    }
    nlplatform_interrupt_enable();
    return err;
}

/* Since there is no STATUS register to quickly/easily tell us if
 * the TIMER is running or not, we have to figure it out using
 * other means. We could try to CAPTURE the counter register
 * multiple times and see if it's moving, but the difference between
 * the TIMER frequency and the CPU frequency makes it a bit problematic
 * to figure out how many times we need to sample.
 */
static bool timer_is_running_locked(timer_state_t *timer_state, NRF_TIMER_Type *timer_controller)
{
    if (timer_controller->SHORTS & NLTIMER_SHORTS_CLEAR_ON_COMPARE_MASK)
    {
        /* for auto_restart timer, the HW timer should
         * always be running unless nltimer_stop() was called.
         * nltimer_stop() will set timer_running to false
         * to indicate timer was explicitly stopped.
         * timer_state->timer_running should be accurate
         * so just fall through.
         */
    }
    else
    {
        /* for a one-shot timer, we cannot rely on timer_running
         * state alone to tell if the HW timer is stopped because
         * we used the SHORT to stop the timer on compare/match.
         * if timer_running is false, we know for sure the timer is
         * stopped, but if it is true, we have to check the value
         * of EVENTS_COMPARE[NLTIMER_CC_COMPARE]. if the value is
         * 1, then the SHORT should have triggered and stopped the
         * counter, so we update timer_running to false.
         */
        if (timer_state->timer_running)
        {
            assert(timer_controller->SHORTS & NLTIMER_SHORTS_STOP_ON_COMPARE_MASK);
            if (timer_controller->EVENTS_COMPARE[NLTIMER_CC_COMPARE])
            {
                // compare/match triggered, SHORT should have stopped
                // the timer so update timer_running
                timer_state->timer_running = false;
            }
        }
    }
    return timer_state->timer_running;
}

int nltimer_release(nltimer_id_t timer_id)
{
    int err;
    timer_state_t *timer_state = &s_timer_state[timer_id];

    assert((unsigned)timer_id < NL_NUM_TIMERS);
    nlplatform_interrupt_disable();
    if (timer_state->in_use)
    {
        NRF_TIMER_Type *timer_controller = s_timer_config[timer_id].timer_controller;

        // stop it just in case
        timer_controller->TASKS_STOP = 1;
        // shutdown to reduce power consumption
        timer_controller->TASKS_SHUTDOWN = 1;

        memset(timer_state, 0, sizeof(*timer_state));
        timer_state->shutdown = true;
        err = 0;
    }
    else
    {
        err = ERROR_BAD_STATE;
    }
    nlplatform_interrupt_enable();
    return err;
}

int nltimer_set(nltimer_id_t timer_id, uint32_t time_us, nltimer_handler_t callback, void *context, bool auto_restart)
{
    int err = ERROR_NOT_REQUESTED;
    timer_state_t *timer_state = &s_timer_state[timer_id];

    assert((unsigned)timer_id < NL_NUM_TIMERS);
    nlplatform_interrupt_disable();
    if (timer_state->in_use)
    {
        NRF_TIMER_Type *timer_controller = s_timer_config[timer_id].timer_controller;
        // we do not allow changing a timer that is already running
        if (timer_is_running_locked(timer_state, timer_controller) == false)
        {
            timer_state->callback = callback;
            timer_state->context = context;

            // default PRESCALER of 1MHz is what we want so we don't change it

            timer_controller->MODE = TIMER_MODE_MODE_Timer;
            // the default rate is 1MHz, so each unit of the counter register is 1us.
            // based on the target time_us, set the bitmode
            if (time_us <= 0xff)
            {
                timer_controller->BITMODE = TIMER_BITMODE_BITMODE_08Bit;
            }
            else if (time_us <= 0xffff)
            {
                timer_controller->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
            }
            else if (time_us <= 0xffffff)
            {
                timer_controller->BITMODE = TIMER_BITMODE_BITMODE_24Bit;
            }
            else
            {
                timer_controller->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
            }

            // set target time and clear compare
            timer_controller->CC[NLTIMER_CC_COMPARE] = time_us;
            timer_controller->EVENTS_COMPARE[NLTIMER_CC_COMPARE] = 0;

            // configure SHORTS
            if (auto_restart)
            {
                // if timer is to restart automatically, configure SHORTS
                // to clear the counter when compare is hit
                timer_controller->SHORTS = NLTIMER_SHORTS_CLEAR_ON_COMPARE_MASK;
            }
            else
            {
                // if this is a one shot timer, configure SHORTS to STOP when
                // compare is hit. once the CC[NLTIMER_CC_COMPARE] value is reached,
                // this SHORT will stop the COUNTER from incrementing
                // any further, but there is no SHORT for SHUTDOWN, so
                // the power consumption won't be as low as it could be.
                // we rely on the user to invoke nltimer_stop() or
                // nltimer_release() to cause the SHUTDOWN to occur.
                // we don't configure an interrupt to call SHUTDOWN
                // so this code doesn't have to have interrupts (bootloader may
                // not want interrupts).
                // with some additional complexity, we could use a PPI to
                // write to the TASKS_SHUTDOWN register when the COMPARE EVENT
                // occurs, to get the automatic shutdown feature. this is a future
                // optimization we could consider doing.
                timer_controller->SHORTS = NLTIMER_SHORTS_STOP_ON_COMPARE_MASK;
            }

            if (callback)
            {
                // enable IRQ on compare match to invoke callback after time elapsed,
                // otherwise we expect it will just be polled
                nlplatform_irq_clear_pending(s_timer_config[timer_id].irq_num);
                nlplatform_irq_enable(s_timer_config[timer_id].irq_num);
                timer_controller->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
            }
            else
            {
                // disable IRQ
                timer_controller->INTENCLR = TIMER_INTENCLR_COMPARE0_Msk;
                nlplatform_irq_disable(s_timer_config[timer_id].irq_num);
            }
            timer_state->configured = true;

            err = 0;
        }
        else
        {
            err = ERROR_BAD_STATE;
        }
    }
    nlplatform_interrupt_enable();
    return err;
}

int nltimer_start(nltimer_id_t timer_id)
{
    int err = ERROR_NOT_REQUESTED;
    timer_state_t *timer_state = &s_timer_state[timer_id];

    assert((unsigned)timer_id < NL_NUM_TIMERS);
    nlplatform_interrupt_disable();
    if (timer_state->in_use)
    {
        NRF_TIMER_Type *timer_controller = s_timer_config[timer_id].timer_controller;
        // we don't allow starting a timer that is already running or hasn't
        // been configured by a previous call to nltimer_set()
        if ((timer_state->configured == true) && (timer_is_running_locked(timer_state, timer_controller) == false))
        {
            timer_controller->TASKS_CLEAR = 1;
            timer_controller->TASKS_START = 1;
            timer_state->timer_running = true;
            timer_state->shutdown = false;
            err = 0;
        }
        else
        {
            err = ERROR_BAD_STATE;
        }
    }
    nlplatform_interrupt_enable();
    return err;
}

int nltimer_stop(nltimer_id_t timer_id)
{
    int err = ERROR_NOT_REQUESTED;
    timer_state_t *timer_state = &s_timer_state[timer_id];

    assert((unsigned)timer_id < NL_NUM_TIMERS);
    nlplatform_interrupt_disable();
    if (timer_state->in_use)
    {
        // do nothing if already shutdown
        if (timer_state->shutdown == false)
        {
            NRF_TIMER_Type *timer_controller = s_timer_config[timer_id].timer_controller;
            // STOP timer first
            timer_controller->TASKS_STOP = 1;
            // Record the COUNTER value before we shutdown, in case
            // user wants it in nltimer_elapsed()
            timer_controller->TASKS_CAPTURE[NLTIMER_CC_CAPTURE] = 1;
            // SHUTDOWN to save power
            timer_controller->TASKS_SHUTDOWN = 1;

            timer_state->timer_running = false;
            timer_state->shutdown = true;
        }
        err = 0;
    }
    nlplatform_interrupt_enable();
    return err;
}

int nltimer_reset(nltimer_id_t timer_id)
{
    int err = ERROR_NOT_REQUESTED;
    timer_state_t *timer_state = &s_timer_state[timer_id];

    assert((unsigned)timer_id < NL_NUM_TIMERS);
    nlplatform_interrupt_disable();
    if (timer_state->in_use)
    {
        if (timer_state->configured)
        {
            NRF_TIMER_Type *timer_controller = s_timer_config[timer_id].timer_controller;
            timer_controller->TASKS_CLEAR = 1;
            if (timer_state->shutdown)
            {
                /* If shutdown, also clear CC[NLTIMER_CC_CAPTURE]
                 * so nltimer_elapsed() doesn't return an old value
                 */
                timer_controller->CC[NLTIMER_CC_CAPTURE] = 0;
            }
        }
        else
        {
            err = ERROR_BAD_STATE;
        }
    }
    nlplatform_interrupt_enable();
    return err;
}

uint32_t nltimer_elapsed(nltimer_id_t timer_id)
{
    uint32_t result = 0;
    timer_state_t *timer_state = &s_timer_state[timer_id];

    assert((unsigned)timer_id < NL_NUM_TIMERS);
    nlplatform_interrupt_disable();
    if (timer_state->in_use)
    {
        NRF_TIMER_Type *timer_controller = s_timer_config[timer_id].timer_controller;
        /* If the timer was SHUTDOWN, the internal COUNTER will read back
         * as 0, which isn't that useful. We return the last value
         * we captured after STOP and before SHUTDOWN, which might be
         * 0 if timer was never started.
         */
        if (timer_state->shutdown == false)
        {
            /* Capture current COUNTER value */
            timer_controller->TASKS_CAPTURE[NLTIMER_CC_CAPTURE] = 1;
        }
        result = timer_controller->CC[NLTIMER_CC_CAPTURE];
    }
    nlplatform_interrupt_enable();
    return result;
}

int nltimer_active(nltimer_id_t timer_id)
{
    int result = ERROR_NOT_REQUESTED;
    timer_state_t *timer_state = &s_timer_state[timer_id];

    assert((unsigned)timer_id < NL_NUM_TIMERS);
    nlplatform_interrupt_disable();
    if (s_timer_state[timer_id].in_use)
    {
        NRF_TIMER_Type *timer_controller = s_timer_config[timer_id].timer_controller;
        result = timer_is_running_locked(timer_state, timer_controller);
    }
    nlplatform_interrupt_enable();
    return result;
}

static void TimerIRQHandler(nltimer_id_t timer_id)
{
    int task_switch_needed = 0;
    timer_state_t *timer_state = &s_timer_state[timer_id];
    IRQn_Type irq_num = s_timer_config[timer_id].irq_num;
    NRF_TIMER_Type *timer_controller = s_timer_config[timer_id].timer_controller;

    nlplatform_irq_clear_pending(irq_num);
    timer_controller->EVENTS_COMPARE[NLTIMER_CC_COMPARE] = 0;
    if (timer_state->callback)
    {
        task_switch_needed = timer_state->callback(timer_id, timer_state->context);
    }

#ifdef NL_BOOTLOADER
    (void)task_switch_needed;
#else
    portEND_SWITCHING_ISR(task_switch_needed);
#endif
}

void nltimer0_isr(void);
void nltimer0_isr(void)
{
    TimerIRQHandler(0);
}

#if NL_NUM_TIMERS > 1
void nltimer1_isr(void);
void nltimer1_isr(void)
{
    TimerIRQHandler(1);
}
#endif /* NL_NUM_TIMERS > 1 */

#if NL_NUM_TIMERS > 2
void nltimer2_isr(void);
void nltimer2_isr(void)
{
    TimerIRQHandler(2);
}
#endif /* NL_NUM_TIMERS > 2 */

#if NL_NUM_TIMERS > 3
void nltimer3_isr(void);
void nltimer3_isr(void)
{
    TimerIRQHandler(3);
}
#endif /* NL_NUM_TIMERS > 3 */

#if NL_NUM_TIMERS > 4
void nltimer4_isr(void);
void nltimer4_isr(void)
{
    TimerIRQHandler(4);
}
#endif /* NL_NUM_TIMERS > 4 */

