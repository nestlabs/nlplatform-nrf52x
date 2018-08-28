/*
 *
 *    Copyright (c) 2018 Nest Labs, Inc.
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
 *
 *    Description:
 *      Nordic nrf52x Serial Driver for SPI Slave Mode.
 *
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

#include <nlassert.h>
#include <nlproduct_config.h>

#include <nlplatform.h>
#include <nlplatform/nlgpio.h>
#include <nlplatform/nlspi_slave.h>
#include <nlplatform/nlwatchdog.h>

#include <nrf52x/device/nrf52840.h>
#include <nrf52x/device/nrf52840_bitfields.h>

#define SPIS_DEFAULT_DEF        (0xFF)      // Default character. Character clocked out in case of an ignored transaction.
#define SPIS_DEFAULT_ORC        (0xFF)      // Over-read character


typedef struct nlspi_slave_info_s
{
    uint8_t MOSI_pin;
    uint8_t MISO_pin;
    uint8_t SCLK_pin;
    uint8_t nSSEL_pin;
    IRQn_Type irq_num;
    volatile NRF_SPIS_Type *spi_controller;
} nlspi_slave_info_t;

typedef struct nlspi_slave_controller_s
{
    const nlspi_slave_config_t *config;
    nlspi_slave_transaction_complete_callback callback;
} nlspi_slave_controller_t;

static nlspi_slave_info_t const sSpiSlaveInfo[] =
{
    {
        SPIS0_MOSI_PIN,
        SPIS0_MISO_PIN,
        SPIS0_CLK_PIN,
        SPIS0_CS_PIN,
        SPIS0_IRQ_NUM,
        SPIS0_SERIAL_CONTROLLER,
    },
#ifdef SPIS1_MOSI_PIN
    {
        SPIS1_MOSI_PIN,
        SPIS1_MISO_PIN,
        SPIS1_CLK_PIN,
        SPIS1_CS_PIN,
        SPIS1_IRQ_NUM,
        SPIS1_SERIAL_CONTROLLER,
    },
#endif
#ifdef SPIS2_MOSI_PIN
    {
        SSPIS2_MOSI_PIN,
        SSPIS2_MISO_PIN,
        SSPIS2_CLK_PIN,
        SSPIS2_CS_PIN,
        SSPIS2_IRQ_NUM,
        SSPIS2_SERIAL_CONTROLLER,
    },
#endif
};

static nlspi_slave_controller_t sSpiSlaveController[NL_NUM_SPI_SLAVE_CONTROLLERS];

#ifndef NL_NO_RTOS
static void nssel_isr(const nlgpio_id_t gpio, void *data)
{
    nlspi_slave_controller_t *controller = data;

    // deassert the host interrupt
    nlgpio_set_value(controller->config->mHostIntPin, 1);

    return;
}
#endif

int nlspi_slave_request(const nlspi_slave_config_t *aConfig, nlspi_slave_transaction_complete_callback aCallback)
{
    int retval = 0;
    nlspi_slave_controller_t *controller;
    const nlspi_slave_info_t *info;
    volatile NRF_SPIS_Type *spi_ctrl;
    const uint32_t spi_mode_config[] = {
        (SPIS_CONFIG_ORDER_MsbFirst | (SPIS_CONFIG_CPOL_ActiveHigh << SPIS_CONFIG_CPOL_Pos) | (SPIS_CONFIG_CPHA_Leading  << SPIS_CONFIG_CPHA_Pos)),    // spi mode 0
        (SPIS_CONFIG_ORDER_MsbFirst | (SPIS_CONFIG_CPOL_ActiveHigh << SPIS_CONFIG_CPOL_Pos) | (SPIS_CONFIG_CPHA_Trailing << SPIS_CONFIG_CPHA_Pos)),    // spi mode 1
        (SPIS_CONFIG_ORDER_MsbFirst | (SPIS_CONFIG_CPOL_ActiveLow  << SPIS_CONFIG_CPOL_Pos) | (SPIS_CONFIG_CPHA_Leading  << SPIS_CONFIG_CPHA_Pos)),    // spi mode 2
        (SPIS_CONFIG_ORDER_MsbFirst | (SPIS_CONFIG_CPOL_ActiveLow  << SPIS_CONFIG_CPOL_Pos) | (SPIS_CONFIG_CPHA_Trailing << SPIS_CONFIG_CPHA_Pos))     // spi_mode 3
    };

    nlASSERT(aConfig);
    nlASSERT(aConfig->mControllerId < ARRAY_SIZE(sSpiSlaveController));
    nlASSERT(aConfig->mMode < ARRAY_SIZE(spi_mode_config));

    controller = &sSpiSlaveController[aConfig->mControllerId];
    info = &sSpiSlaveInfo[aConfig->mControllerId];
    spi_ctrl = info->spi_controller;

    nlREQUIRE_ACTION(spi_ctrl->ENABLE == (SPIS_ENABLE_ENABLE_Disabled << SPIS_ENABLE_ENABLE_Pos), done, retval = -EALREADY);

    controller->callback = aCallback;
    controller->config = aConfig;

#ifndef NL_NO_RTOS
    nlgpio_request(aConfig->mHostIntPin, GPIOF_OUT_HIGH);
    if (nlgpio_irq_request(info->nSSEL_pin, IRQF_TRIGGER_FALLING, nssel_isr, controller) != 0)
    {
        nlgpio_release(controller->config->mHostIntPin);
        retval = -EIO;
        goto done;
    }
#endif

    nlgpio_request(info->MISO_pin, GPIOF_IN);   // configured as an input per nrf52x spec
    nlgpio_request(info->MOSI_pin, GPIOF_IN);
    nlgpio_request(info->SCLK_pin, GPIOF_IN);
    nlgpio_request(info->nSSEL_pin, GPIOF_IN);

    // route controller to pins
    spi_ctrl->PSEL.SCK  = info->SCLK_pin;
    spi_ctrl->PSEL.MOSI = info->MOSI_pin;
    spi_ctrl->PSEL.MISO = info->MISO_pin;
    spi_ctrl->PSEL.CSN  = info->nSSEL_pin;

    // initialize the rx buffer pointer and length
    spi_ctrl->RXD.PTR = (uint32_t)NULL;
    spi_ctrl->RXD.MAXCNT  = 0;

    // initialize the tx buffer pointer and length
    spi_ctrl->TXD.PTR = (uint32_t)NULL;
    spi_ctrl->TXD.MAXCNT  = 0;

    // set clk mode and bit order
    spi_ctrl->CONFIG = spi_mode_config[controller->config->mMode];

    // set default byte
    spi_ctrl->DEF = SPIS_DEFAULT_DEF;

    // set over-read btye
    spi_ctrl->ORC = SPIS_DEFAULT_ORC;

    // clear spi events end
    spi_ctrl->EVENTS_END = 0;

    // set up spi short cuts
    // the cpu will always acquire the sspi hw semaphore upon sspi operation complete
    spi_ctrl->SHORTS |= SPIS_SHORTS_END_ACQUIRE_Msk;

#ifndef NL_NO_RTOS
    // enable IRQ
    spi_ctrl->INTENSET = SPIS_INTENSET_END_Msk;

    // enable NVIC interrupt
    nlplatform_irq_clear_pending(info->irq_num);
    nlplatform_irq_enable(info->irq_num);
#endif

    // enable SPI slave device
    spi_ctrl->ENABLE = (SPIS_ENABLE_ENABLE_Enabled << SPIS_ENABLE_ENABLE_Pos);

done:
    return retval;
}

void nlspi_slave_release(const nlspi_slave_config_t *aConfig)
{
    nlspi_slave_controller_t *controller;
    const nlspi_slave_info_t *info;
    volatile NRF_SPIS_Type *spi_ctrl;

    nlASSERT(aConfig);
    nlASSERT(aConfig->mControllerId < ARRAY_SIZE(sSpiSlaveController));

    controller = &sSpiSlaveController[aConfig->mControllerId];
    info = &sSpiSlaveInfo[aConfig->mControllerId];
    spi_ctrl = info->spi_controller;

    nlREQUIRE(spi_ctrl->ENABLE != (SPIS_ENABLE_ENABLE_Disabled << SPIS_ENABLE_ENABLE_Pos), done);

    // disable SPI slave device
    spi_ctrl->ENABLE = (SPIS_ENABLE_ENABLE_Disabled << SPIS_ENABLE_ENABLE_Pos);

#ifndef NL_NO_RTOS
    // disable NVIC interrupt
    nlplatform_irq_disable(info->irq_num);
    nlplatform_irq_clear_pending(info->irq_num);

    // disable IRQ
    spi_ctrl->INTENCLR = 0xFFFFFFFF;

    nlgpio_irq_release(info->nSSEL_pin);
#endif

    nlgpio_release(info->MISO_pin);
    nlgpio_release(info->MOSI_pin);
    nlgpio_release(info->SCLK_pin);
    nlgpio_release(info->nSSEL_pin);
#ifndef NL_NO_RTOS
    nlgpio_release(controller->config->mHostIntPin);
#endif

    controller->callback = NULL;

done:
    return;
}

int nlspi_slave_prepare_transaction(const nlspi_slave_config_t *aConfig,
                                    uint8_t *aOutputBuf, size_t aOutputBufLen, uint8_t *aInputBuf, size_t aInputBufLen,
                                    bool aRequestTransactionFlag)
{
    int retval = 0;
    nlspi_slave_controller_t *controller;
    const nlspi_slave_info_t *info;
    volatile NRF_SPIS_Type *spi_ctrl;

    nlASSERT(aConfig);
    nlASSERT(aConfig->mControllerId < ARRAY_SIZE(sSpiSlaveController));

    controller = &sSpiSlaveController[aConfig->mControllerId];
    info = &sSpiSlaveInfo[aConfig->mControllerId];
    spi_ctrl = info->spi_controller;

    nlREQUIRE_ACTION(spi_ctrl->ENABLE != (SPIS_ENABLE_ENABLE_Disabled << SPIS_ENABLE_ENABLE_Pos), done, retval = -ENOENT);

    // acquire buffer access hw semaphore
    spi_ctrl->TASKS_ACQUIRE = 1;

    nlREQUIRE_ACTION((spi_ctrl->SEMSTAT & SPIS_SEMSTAT_SEMSTAT_Msk) == SPIS_SEMSTAT_SEMSTAT_CPU, done, retval = -EBUSY);

    // use the new output buffer if specified
    // if aOutputBuf == NULL then the last output
    // buffer specified via this api will be used
    if (aOutputBuf != NULL)
    {
        // set the tx buffer pointer and length registers
        spi_ctrl->TXD.PTR = (uint32_t)aOutputBuf;
        spi_ctrl->TXD.MAXCNT  = aOutputBufLen;
    }

    // use the new input buffer if specified
    // if aInputBuf == NULL then the last input
    // buffer specified via this api will be used
    if (aInputBuf != NULL)
    {
        // set the rx buffer pointer and length registers
        spi_ctrl->RXD.PTR = (uint32_t)aInputBuf;
        spi_ctrl->RXD.MAXCNT  = aInputBufLen;
    }

    // release the buffer access hw semaphore
    spi_ctrl->TASKS_RELEASE = 1;

    // assert the host interrupt
    nlgpio_set_value(controller->config->mHostIntPin, aRequestTransactionFlag ? 0 : 1);

done:
    return retval;
}

#ifdef NL_NO_RTOS
static void wait_for_end_event(volatile NRF_SPIS_Type *spi_ctrl, int irq_num)
{
    // when polling for end, just sit and wait
    // for the END event. to reduce power consumption, enable interrupt and
    // use WFE. we enable at NVIC level, but disable at global level since
    // we don't want the ISR to actually get triggered (we never set one up)
    while (spi_ctrl->EVENTS_END == 0)
    {
        nlplatform_interrupt_disable();
        nlplatform_irq_clear_pending(irq_num);
        nlplatform_irq_enable(irq_num);
        spi_ctrl->INTENSET = SPIS_INTENSET_END_Msk;
        /* check state again now that interrupts are disabled, just in case
         * the event came in late
         */
        if (spi_ctrl->EVENTS_END == 0)
        {
            // disable watchdog while we're waiting for input
            bool watchdog_was_enabled = nlwatchdog_is_enabled();
            if (watchdog_was_enabled)
            {
                nlwatchdog_set_enable(false);
            }
            __WFE();
            if (watchdog_was_enabled)
            {
                nlwatchdog_set_enable(true);
            }
        }
        spi_ctrl->INTENCLR = 0xFFFFFFFF;
        nlplatform_irq_disable(irq_num);
        nlplatform_irq_clear_pending(irq_num);
        nlplatform_interrupt_enable();
        __enable_irq();
    }
    // clear EVENTS_END
    spi_ctrl->EVENTS_END = 0;
}

int nlspi_slave_transmit(const nlspi_slave_config_t *aConfig, uint8_t *aTxBuf, size_t aTxBufLen, bool aWaitForCompletion)
{
    int retval = 0;
    const nlspi_slave_info_t *info;
    volatile NRF_SPIS_Type *spi_ctrl;

    nlASSERT(aConfig);
    nlASSERT(aConfig->mControllerId < ARRAY_SIZE(sSpiSlaveController));

    info = &sSpiSlaveInfo[aConfig->mControllerId];
    spi_ctrl = info->spi_controller;

    nlREQUIRE_ACTION(spi_ctrl->ENABLE != (SPIS_ENABLE_ENABLE_Disabled << SPIS_ENABLE_ENABLE_Pos), done, retval = -ENOENT);

    // acquire buffer access hw semaphore
    spi_ctrl->TASKS_ACQUIRE = 1;

    // clear EVENTS_END
    spi_ctrl->EVENTS_END = 0;

    nlREQUIRE_ACTION((spi_ctrl->SEMSTAT & SPIS_SEMSTAT_SEMSTAT_Msk) == SPIS_SEMSTAT_SEMSTAT_CPU, done, retval = -EBUSY);

    // set the tx buffer pointer and length registers
    spi_ctrl->TXD.PTR = (uint32_t)aTxBuf;
    spi_ctrl->TXD.MAXCNT = aTxBufLen;

    // set the rx buffer to 0
    spi_ctrl->RXD.PTR = (uint32_t)0;
    spi_ctrl->RXD.MAXCNT = 0;

    // release the buffer access hw semaphore
    spi_ctrl->TASKS_RELEASE = 1;

    if (aWaitForCompletion)
    {
        // wait for packet to be received
        wait_for_end_event(spi_ctrl, info->irq_num);
    }

done:
    return retval;
}

int nlspi_slave_receive(const nlspi_slave_config_t *aConfig, uint8_t *aRxBuf, size_t aRxBufLen, size_t *aRxBytes)
{
    int retval = 0;
    const nlspi_slave_info_t *info;
    volatile NRF_SPIS_Type *spi_ctrl;

    nlASSERT(aConfig);
    nlASSERT(aConfig->mControllerId < ARRAY_SIZE(sSpiSlaveController));

    info = &sSpiSlaveInfo[aConfig->mControllerId];
    spi_ctrl = info->spi_controller;

    nlREQUIRE_ACTION(spi_ctrl->ENABLE != (SPIS_ENABLE_ENABLE_Disabled << SPIS_ENABLE_ENABLE_Pos), done, retval = -ENOENT);

    // acquire buffer access hw semaphore
    spi_ctrl->TASKS_ACQUIRE = 1;

    // clear EVENTS_END
    spi_ctrl->EVENTS_END = 0;

    nlREQUIRE_ACTION((spi_ctrl->SEMSTAT & SPIS_SEMSTAT_SEMSTAT_Msk) == SPIS_SEMSTAT_SEMSTAT_CPU, done, retval = -EBUSY);

    // set the tx buffer to 0
    spi_ctrl->TXD.PTR = (uint32_t)0;
    spi_ctrl->TXD.MAXCNT = 0;

    // set the rx buffer pointer and length registers
    spi_ctrl->RXD.PTR = (uint32_t)aRxBuf;
    spi_ctrl->RXD.MAXCNT = aRxBufLen;

    // release the buffer access hw semaphore
    spi_ctrl->TASKS_RELEASE = 1;

    // wait for packet to be received
    wait_for_end_event(spi_ctrl, info->irq_num);

    // set the number of bytes actually received
    *aRxBytes = spi_ctrl->RXD.AMOUNT;

done:
    return retval;
}

void nlspi_slave_wait_for_transmit_complete(const nlspi_slave_config_t *aConfig)
{
    const nlspi_slave_info_t *info;
    volatile NRF_SPIS_Type *spi_ctrl;

    nlASSERT(aConfig);
    nlASSERT(aConfig->mControllerId < ARRAY_SIZE(sSpiSlaveController));

    info = &sSpiSlaveInfo[aConfig->mControllerId];
    spi_ctrl = info->spi_controller;

    // wait for packet to be received
    wait_for_end_event(spi_ctrl, info->irq_num);
}

#else // NL_NO_RTOS

static void spis_irq_handler(uint8_t aControllerId)
{
    nlspi_slave_controller_t *controller;
    const nlspi_slave_info_t *info;
    volatile NRF_SPIS_Type *spi_ctrl;

    controller = &sSpiSlaveController[aControllerId];
    info = &sSpiSlaveInfo[aControllerId];
    spi_ctrl = info->spi_controller;

    // Check for SPI transaction complete event.
    if (spi_ctrl->EVENTS_END)
    {
        // clear end event
        spi_ctrl->EVENTS_END = 0;

        if (spi_ctrl->ENABLE == (SPIS_ENABLE_ENABLE_Enabled << SPIS_ENABLE_ENABLE_Pos))
        {
            if (controller->callback)
            {
                const bool from_isr = true;
                uint8_t * const tx_buf = (uint8_t *)spi_ctrl->TXD.PTR;
                size_t    const tx_buf_len = spi_ctrl->TXD.MAXCNT;
                uint8_t * const rx_buf = (uint8_t *)spi_ctrl->RXD.PTR;
                size_t    const rx_buf_len = spi_ctrl->RXD.MAXCNT;
                size_t rx_count = spi_ctrl->RXD.AMOUNT;
                size_t tx_count = spi_ctrl->TXD.AMOUNT;

                controller->callback(controller->config, tx_buf, tx_buf_len, rx_buf, rx_buf_len,
                                     (tx_count > rx_count)? tx_count: rx_count, from_isr);
            }
        }
    }
}

/**
 * SPI controller ISR
 *
 * Prototype is included here because this is an ISR handler and there is a symbol replacement
 * done by the linker (SPIM2_SPIS2_SPI2_IRQHandler = nlspis2_isr) so the prototype for nlradio_isr does not
 * exist elsewhere. The compiler will warn of a missing prototype otherwise which will be
 * interpreted as an error for our builds.
 */
void nlspis0_isr(void);
void nlspis0_isr(void)
{
    spis_irq_handler(0);
}

#ifdef SPI1_MOSI_PIN
void nlspis1_isr(void);
void nlspis1_isr(void)
{
    spis_irq_handler(1);
}
#endif // SPI1_MOSI_PIN

#ifdef SPI2_MOSI_PIN
void nlspis2_isr(void);
void nlspis2_isr(void)
{
    spis_irq_handler(2);
}
#endif // SPI2_MOSI_PIN
#endif // NL_NO_RTOS
