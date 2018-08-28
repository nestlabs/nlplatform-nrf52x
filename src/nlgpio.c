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
 *      This file is a driver for GPIO management on Nordic NRF52x.
 *    GPIOs can be controlled via two different register controls.
 *    One, called GPIO, can be used to configure direction, pull,
 *    set value, get value, etc. It also has SENSE configuration
 *    and a LATCH register which can be used to detect changes in
 *    a GPIO pin. However, the GPIO controller has no interrupt
 *    generation ability.
 *      The other controller, called GPIOTE, can be used to also
 *    configure GPIO pins and also generate interrupts on pin
 *    changes. It is limited to 8 channels though, and when a
 *    channel is configured to detect input changes on a particular
 *    pin, the GPIOTE consumes extra power.
 *      The GPIOTE also has a PORT event/interrupt, which is tied
 *    to the DETECT signal from the GPIO controller. Using the
 *    PORT event/interrupt and LATCH registers, we can implement
 *    interrupt per pin without the GPIOTE channels and save some
 *    power. It's a bit more complicated to do edge interrupts
 *    than level interrupts because the HW doesn't do this for us,
 *    but I think it's possible to do edge detection in SW with a
 *    few more interrupts than if the HW could do it.
 *      There's mention in the documentation that using the
 *    channel events requires a high frequency clock, wherease the
 *    PORT detection mechanism uses only a low frequency clock.
 *    The datasheet doesn't mention any of this however, so it's
 *    something we might want to bring up.
 */

#include <string.h>
#include <errno.h>
#include <stdio.h>

#include <nlplatform.h>
#include <nlplatform/nlgpio.h>

#include <nlplatform_nrf52x/nrf52x_gpio.h>
#include <nrf52x/device/nrf52840.h>
#include <nrf52x/device/nrf52840_bitfields.h>
#include <nrf52x/device/nrf52840_peripherals.h>

// 32 GPIOs per port
#define NUM_GPIO_PER_PORT             ( 32 )

// Get the GPIO port registers given the gpio number
#define GPIO_PORT_REGS(gpio_num) (gpio_num < NUM_GPIO_PER_PORT ? NRF_P0 : NRF_P1)

#define GPIO_PIN_VALUE(gpio_regs, gpio_index) ((gpio_regs->IN >> gpio_index) & 0x01)
#define GPIO_PIN_CFG_SENSE(gpio_regs, gpio_index) ((gpio_regs->PIN_CNF[gpio_index] & GPIO_PIN_CNF_SENSE_Msk) >> GPIO_PIN_CNF_SENSE_Pos)

// Using the PORT interrupt and LATCH register, we aren't limited
// to the 8 GPIOTE channels so can have as many interrupts as pins.
// If we need to reduce RAM usage, we could limit this with a
// mapping table instead.
typedef struct GpioCallbackStruct_s {
    nlgpio_irq_handler_t callback;
    void *arg;
} GpioCallbackStruct;

static volatile GpioCallbackStruct s_gpio_irq_callbacks[NL_NUM_GPIO];
static volatile uint8_t s_gpio_irq_flags[NL_NUM_GPIO];

void nlgpio_init(void)
{
    unsigned i;
    for (i = 0; i < NL_NUM_GPIO; i++) {
        if (GPIOF_NO_CONFIG != gpio_low_power_config[i]) {
            nlgpio_request(i, gpio_low_power_config[i]);
        }
    }

    // Using LDETECT mode, a PORT event is generated on a rising edge of LDETECT which can
    // be generated under the following conditions:
    // 1) When the LATCH register has an initial value of 0, and then any LATCH bit is set
    // 2) When the LATCH register has a non-zero value after attempting to clear any of its bits
    NRF_P0->DETECTMODE = GPIO_DETECTMODE_DETECTMODE_LDETECT;
    NRF_P1->DETECTMODE = GPIO_DETECTMODE_DETECTMODE_LDETECT;

    // Enable NVIC interrupt for GPIOTE.  Nothing should happen
    // until PORT interrupt is triggered by SENSE being configured.
    nlplatform_irq_clear_pending(GPIOTE_IRQn);
    nlplatform_irq_enable(GPIOTE_IRQn);

    // enable PORT interrupt in GPIOTE
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
}

int nlgpio_request(const nlgpio_id_t gpio, nlgpio_flags_t gpio_flags)
{
    assert(gpio < NL_NUM_GPIO);

    NRF_GPIO_Type *gpio_regs = GPIO_PORT_REGS(gpio);
    uint32_t gpio_index = gpio % NUM_GPIO_PER_PORT;
    uint32_t out_value = gpio_flags & INTERNAL_OUT_HIGH;
    uint32_t out_mask = 1 << gpio_index;

    gpio_flags &= ~INTERNAL_OUT_HIGH;

    nlplatform_interrupt_disable();

    // Since the pin_cfg register (for setting direction, pull, etc)
    // is different from the output register (for setting output value),
    // order is important to avoid unwanted output glitches:
    //
    // Old cfg   New cfg       Comment
    // input     output        set out reg first, in case
    //                         previous out reg value is
    //                         different than desired
    // output    not output    set cfg reg first, in case out reg value
    //                         is different than desired, otherwise
    //                         could glitch output.
    //                         we could temporarily have wrong pull
    //                         direction if input but shouldn't matter
    if (gpio_regs->PIN_CNF[gpio_index] & INTERNAL_PIN_CFG_DIR_OUT) {
        // previous mode was an output, set cfg reg first to avoid
        // glitching the output value if the new mode isn't output
        gpio_regs->PIN_CNF[gpio_index] = gpio_flags;
        if (out_value)
        {
            gpio_regs->OUTSET = out_mask;
        }
        else
        {
            gpio_regs->OUTCLR = out_mask;
        }
    } else {
        // previous mode was not an output,
        // set out reg first so the value is right before the new mode is set.
        // if the new mode is output, this is important to avoid glitching. if
        // the new mode is not output, it's doesn't really matter but shouldn't hurt.
        if (out_value)
        {
            gpio_regs->OUTSET = out_mask;
        }
        else
        {
            gpio_regs->OUTCLR = out_mask;
        }
        gpio_regs->PIN_CNF[gpio_index] = gpio_flags;
    }

    nlplatform_interrupt_enable();

    return 0;
}

int nlgpio_release(const nlgpio_id_t gpio)
{
    return nlgpio_request(gpio, gpio_low_power_config[gpio]);
}

int nlgpio_set_input(const nlgpio_id_t gpio)
{
    return nlgpio_request(gpio, GPIOF_IN);
}

int nlgpio_set_output(const nlgpio_id_t gpio, unsigned value)
{
    nlgpio_flags_t const state[] = { (nlgpio_flags_t)GPIOF_OUT_LOW, (nlgpio_flags_t)GPIOF_OUT_HIGH };

    return nlgpio_request(gpio, state[!!value]);
}

int nlgpio_get_value(const nlgpio_id_t gpio)
{
    uint32_t gpio_index = gpio % NUM_GPIO_PER_PORT;
    NRF_GPIO_Type *gpio_regs = GPIO_PORT_REGS(gpio);

    assert(gpio < NL_NUM_GPIO);

    // regardless of config, always read the IN register.
    // if it's a push-pull, the OUT and IN registers should
    // have the same value.  if it's a open-drain, the IN
    // will give you the real value, but OUT might not
    // if there is not a external pull-up resister and the
    // OUT was set to 1.  values for other configs
    // are defined by the chip (e.g. if analog, value is always 1).
    return GPIO_PIN_VALUE(gpio_regs, gpio_index);
}

int nlgpio_set_value(const nlgpio_id_t gpio, unsigned value)
{
    int retval;
    NRF_GPIO_Type *gpio_regs = GPIO_PORT_REGS(gpio);
    uint32_t gpio_index = gpio % NUM_GPIO_PER_PORT;

    assert(gpio < NL_NUM_GPIO);

    // check if this cfg is gpio output
    if (gpio_regs->PIN_CNF[gpio_index] & INTERNAL_PIN_CFG_DIR_OUT)
    {
        if (value)
        {
            gpio_regs->OUTSET = 1 << gpio_index;
        }
        else
        {
            gpio_regs->OUTCLR = 1 << gpio_index;
        }
        retval = 0;
    } else {
        // not a GPIO output we should be setting
        retval = -1;
    }

    return retval;
}

bool nlgpio_is_valid(unsigned number)
{
    return (number < NL_NUM_GPIO);
}

int nlgpio_irq_request(const nlgpio_id_t gpio, unsigned irq_flags, nlgpio_irq_handler_t callback, void *arg)
{
    NRF_GPIO_Type *gpio_regs = GPIO_PORT_REGS(gpio);
    uint32_t gpio_index = gpio % NUM_GPIO_PER_PORT;
    unsigned pin_sense_cfg = GPIO_PIN_CNF_SENSE_Disabled;

    assert(gpio < NL_NUM_GPIO);
    assert(callback);

    nlplatform_interrupt_disable();

    // assert entry isn't already in use.  nlgpio_irq_release() should have
    // been called before changing the callback (not allowed to change callback
    // on the fly - this is just policy to prevent accidental misuse).
    assert(s_gpio_irq_callbacks[gpio].callback == NULL);

    s_gpio_irq_callbacks[gpio].callback = callback;
    s_gpio_irq_callbacks[gpio].arg = arg;
    s_gpio_irq_flags[gpio] = irq_flags;

    // Configure SENSE so that DETECT changes as needed.
    switch(irq_flags)
    {
    case IRQF_TRIGGER_BOTH:
        // to catch both edges, we need to set SENSE to the opposite of what
        // the current state of the pin is. then in the interrupt handler,
        // we need to switch the SENSE.
        if (GPIO_PIN_VALUE(gpio_regs, gpio_index))
        {
            // current state of PIN is high, so configure SENSE for low
            pin_sense_cfg = GPIO_PIN_CNF_SENSE_Low;
        }
        else
        {
            // current state of PIN is low, so configure SENSE for high
            pin_sense_cfg = GPIO_PIN_CNF_SENSE_High;
        }
        break;
    case IRQF_TRIGGER_RISING:
        // SENSE provides level detection only.  if the current level
        // is low, then just configuring for SENSE_High is enough.
        // if the current level is high, we need to first configure
        // for SENSE_Low and then when we get the interrupt,
        // switch to SENSE_High
        if (GPIO_PIN_VALUE(gpio_regs, gpio_index))
        {
            // configure for SENSE low and on interrupt we'll
            // configure for high
            pin_sense_cfg = GPIO_PIN_CNF_SENSE_Low;
            break;
        }
        else
        {
            // fall through to configure for high
        }
    case IRQF_TRIGGER_HIGH:
        pin_sense_cfg = GPIO_PIN_CNF_SENSE_High;
        break;
    case IRQF_TRIGGER_FALLING:
        // SENSE provides level detection only.  if the current level
        // is high, then just configuring for SENSE_Low is enough.
        // if the current level is low, we need to first configure
        // for SENSE_High and then when we get the interrupt,
        // switch to SENSE_Low
        if (GPIO_PIN_VALUE(gpio_regs, gpio_index) == 0)
        {
            // configure for SENSE high and on interrupt we'll
            // configure for low
            pin_sense_cfg = GPIO_PIN_CNF_SENSE_High;
            break;
        }
        else
        {
            // fall through to configure for low
        }
    case IRQF_TRIGGER_LOW:
        pin_sense_cfg = GPIO_PIN_CNF_SENSE_Low;
        break;
    case IRQF_TRIGGER_NONE:
    default:
        /* invalid irq_flag value */
        assert(0);
        break;
    }

    // clear old value and set new value for SENSE
    gpio_regs->PIN_CNF[gpio_index] &= ~GPIO_PIN_CNF_SENSE_Msk;
    gpio_regs->PIN_CNF[gpio_index] |= pin_sense_cfg << GPIO_PIN_CNF_SENSE_Pos;

    nlplatform_interrupt_enable();
    return 0;
}

int nlgpio_irq_release(const nlgpio_id_t gpio)
{
    NRF_GPIO_Type *gpio_regs = GPIO_PORT_REGS(gpio);
    uint32_t gpio_index = gpio % NUM_GPIO_PER_PORT;

    assert(gpio < NL_NUM_GPIO);

    nlplatform_interrupt_disable();

    s_gpio_irq_callbacks[gpio].callback = NULL;
    s_gpio_irq_callbacks[gpio].arg = NULL;
    s_gpio_irq_flags[gpio] = IRQF_TRIGGER_NONE;

    // clear SENSE cfg
    gpio_regs->PIN_CNF[gpio_index] &= ~GPIO_PIN_CNF_SENSE_Msk;
    // clear LATCH bit just in case it was set
    gpio_regs->LATCH = 1 << gpio_index;

    nlplatform_interrupt_enable();
    return 0;
}

/* returns if a gpio irq is pending (but may be blocked due to interrupts being
 * disabled).  used to implement uart RX wake detection where the UART controller
 * is disabled in sleep, the RX pin is configured as a GPIO with interrupts
 * on any edge, and in the code running early in resume from sleep, the uart
 * driver wants to check if the wakeup cause was RX activity while all but interrupts
 * are still disabled.
 */
bool nlgpio_irq_pending(const nlgpio_id_t gpio)
{
    NRF_GPIO_Type *gpio_regs = GPIO_PORT_REGS(gpio);
    uint32_t gpio_index = gpio % NUM_GPIO_PER_PORT;
    bool irq_pending;

    assert(gpio < NL_NUM_GPIO);

    // It is possible for a LATCH bit to be set but the PORT event be cleared
    // so we check both registers to determine if an IRQ is pending
    if ((gpio_regs->LATCH & (1 << gpio_index)) && NRF_GPIOTE->EVENTS_PORT)
    {
        irq_pending = true;
    }
    else
    {
        irq_pending = false;
    }
    return irq_pending;
}

static void change_sense_config(NRF_GPIO_Type *gpio_regs, uint32_t gpio_index, unsigned new_sense_cfg)
{
    // Changing SENSE.  Disable PORT interrupt when we do
    // the change to prevent spurious interrupts
    NRF_GPIOTE->INTENCLR = GPIOTE_INTENCLR_PORT_Msk;

    // Set new SENSE level
    gpio_regs->PIN_CNF[gpio_index] &= ~GPIO_PIN_CNF_SENSE_Msk;
    gpio_regs->PIN_CNF[gpio_index] |= new_sense_cfg << GPIO_PIN_CNF_SENSE_Pos;

    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
}

static void gpio_port_interrupt_handler(NRF_GPIO_Type *gpio_regs, unsigned port_base_index)
{
    // Read gpio_regs->LATCH once into a variable and process
    // all the set bits.  We don't keep re-reading the LATCH register
    // because we've seen problems where clearing a bit in the LATCH
    // register doesn't take effect right away. A subsequent read
    // might indicate the bit is still set, and then a later read
    // then finds the bit cleared, which is hard to code to. I'm not
    // sure what the delay is for the clear to take effect, and sometimes
    // it might not clear because the underlying state is still set.
    // So rather than trying to debounce this LATCH register update
    // issue, we'll read it once, process the set bits, clear them,
    // and then return.  If the bit becomes reset, we should get
    // another interrupt immediately and process again.
    uint32_t latch_reg_value = gpio_regs->LATCH;
    uint32_t latch_reg_value_copy = latch_reg_value;
    while (latch_reg_value)
    {
        // process all the set bits just once. if the state keeps
        // the bit set (i.e. immediately reset), we should get
        // another interrupt which we'll handle next time.
        uint32_t gpio_index = __builtin_ffs(latch_reg_value) - 1;
        const nlgpio_id_t gpio = port_base_index + gpio_index;

        assert(gpio < NL_NUM_GPIO);
        assert(s_gpio_irq_callbacks[gpio].callback);

        switch(s_gpio_irq_flags[gpio])
        {
        case IRQF_TRIGGER_RISING:
        {
            uint32_t old_sense_cfg = GPIO_PIN_CFG_SENSE(gpio_regs, gpio_index);

            // if SENSE was configured for low, then initial state was
            // high and we needed to wait for the low before we could
            // switch to high.
            if (old_sense_cfg == GPIO_PIN_CNF_SENSE_Low)
            {
                change_sense_config(gpio_regs, gpio_index, GPIO_PIN_CNF_SENSE_High);
            }
            else
            {
                // check current gpio state, if it is low, then leave SENSE config unchanged.
                // if it is high, then we need to switch SENSE config to low to
                // restart rising edge detection
                if (GPIO_PIN_VALUE(gpio_regs, gpio_index))
                {
                    change_sense_config(gpio_regs, gpio_index, GPIO_PIN_CNF_SENSE_Low);
                }

                // invoke callback after sense config, callback may decide to disable pin interrupt
                s_gpio_irq_callbacks[gpio].callback(gpio, s_gpio_irq_callbacks[gpio].arg);
            }
            break;
        }
        case IRQF_TRIGGER_FALLING:
        {
            uint32_t old_sense_cfg = GPIO_PIN_CFG_SENSE(gpio_regs, gpio_index);

            // if SENSE was configured for high, then initial state was
            // low and we needed to wait for the high before we could
            // switch to high.
            if (old_sense_cfg == GPIO_PIN_CNF_SENSE_High)
            {
                change_sense_config(gpio_regs, gpio_index, GPIO_PIN_CNF_SENSE_Low);
            }
            else
            {
                // check current gpio state, if it is high, then leave SENSE config unchanged.
                // if it is low, then we need to switch SENSE config to high to
                // restart falling edge detection
                if (GPIO_PIN_VALUE(gpio_regs, gpio_index) == 0)
                {
                    change_sense_config(gpio_regs, gpio_index, GPIO_PIN_CNF_SENSE_High);
                }

                // invoke callback after sense config, callback may decide to disable pin interrupt
                s_gpio_irq_callbacks[gpio].callback(gpio, s_gpio_irq_callbacks[gpio].arg);
            }
            break;
        }
        case IRQF_TRIGGER_BOTH:
        {
            // if we want interrupt on both edges, configure SENSE again so
            // we catch the other transition next. this can be tricky if
            // the pin signal is bouncing around. This is a best effort and not
            // guaranteed to catch signals that can toggle at high frequency
            uint32_t old_sense_cfg = GPIO_PIN_CFG_SENSE(gpio_regs, gpio_index);
            uint32_t new_sense_cfg;

            if (old_sense_cfg == GPIO_PIN_CNF_SENSE_Low)
            {
                new_sense_cfg = GPIO_PIN_CNF_SENSE_High;
            }
            else
            {
                new_sense_cfg = GPIO_PIN_CNF_SENSE_Low;
            }
            change_sense_config(gpio_regs, gpio_index, new_sense_cfg);

            // intentional fall through to callback
        }
        default:
            s_gpio_irq_callbacks[gpio].callback(gpio, s_gpio_irq_callbacks[gpio].arg);
            break;
        }

        // clear shadow reg bit
        latch_reg_value &= ~(1 << gpio_index);
    }

    // Clear previously detected LATCH bits with one write. If we clear one at a time,
    // there is a possibility of generating several new PORT events with each clear when
    // in LDETECT mode. This can cause an extra pending PORT event while we loop through
    // the set bits, and would cause us to take an extra interrupt unnecessarily
    if (latch_reg_value_copy)
    {
        gpio_regs->LATCH = latch_reg_value_copy;
    }
}

// Handler for all GPIOTE event interrupts and PORT interrupt.
// We don't expect any event interrupts.
void nlgpio_isr(void);
void nlgpio_isr(void)
{
    // We clear the PORT event first to avoid the issue where we prematurely clear
    // away a PORT event that is generated when re-configuring SENSE. This is
    // required for the case where we have a short pulse and we're trying to
    // trigger on the second edge.
    NRF_GPIOTE->EVENTS_PORT = 0;
    nlplatform_irq_clear_pending(GPIOTE_IRQn);

    // Need to figure out which pin interrupt has occurred and call it's registered callback.
    // The LATCH register in each port latched any SENSE changes since last time we
    // cleared the LATCH register.
    gpio_port_interrupt_handler(NRF_P0, 0);
    gpio_port_interrupt_handler(NRF_P1, P0_PIN_NUM);
}
