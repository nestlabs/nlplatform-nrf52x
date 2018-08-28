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
 *      Nordic NRF52X SPI master driver.  This is using the non-EasyDMA
 *      enabled SPI controller because the EasyDMA controller has
 *      errata issues.
 *
 */
#include <errno.h>
#include <stdio.h>
#include <nlproduct_config.h>

#if NL_NUM_SPI_CONTROLLERS > 0
#include <nlplatform/nlgpio.h>
#include <nlplatform/nlspi.h>

#if defined(BUILD_FEATURE_NL_PROFILE) && !defined(NL_NO_RTOS)
#include <nlplatform/nlprofile.h>
#endif

#include <nrf52x/device/nrf52840.h>
#include <nrf52x/device/nrf52840_bitfields.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

struct nlspi_controller_s {
    StaticSemaphore_t mMutex;
    TaskHandle_t      mTaskHandle;
};

static struct nlspi_controller_s s_spi_controller[NL_NUM_SPI_CONTROLLERS];

typedef struct
{
    uint8_t MOSI_pin;
    uint8_t MISO_pin;
    uint8_t SCLK_pin;
    IRQn_Type irq_num;
    volatile NRF_SPI_Type *spi_controller;
} spi_info_t;

static spi_info_t const s_spi_info[] =
{
    {
        SPI0_MOSI_PIN,
        SPI0_MISO_PIN,
        SPI0_CLK_PIN,
        SPI0_IRQ_NUM,
        SPI0_SERIAL_CONTROLLER,
    },
#ifdef SPI1_MOSI_PIN
    {
        SPI1_MOSI_PIN,
        SPI1_MISO_PIN,
        SPI1_CLK_PIN,
        SPI1_IRQ_NUM,
        SPI1_SERIAL_CONTROLLER,
    },
#endif
#ifdef SPI2_MOSI_PIN
    {
        SPI2_MOSI_PIN,
        SPI2_MISO_PIN,
        SPI2_CLK_PIN,
        SPI2_IRQ_NUM,
        SPI2_SERIAL_CONTROLLER,
    },

#endif
#ifdef SPI3_MOSI_PIN
    {
        SPI3_MOSI_PIN,
        SPI3_MISO_PIN,
        SPI3_CLK_PIN,
        SPI3_IRQ_NUM,
        SPI3_SERIAL_CONTROLLER,
    },
#endif
};

#if defined(NL_BOOTLOADER)
#define USE_RTOS 0
#else
#define USE_RTOS 1
#endif

void nlspi_init(void)
{
    unsigned i;
    for (i = 0; i < NL_NUM_SPI_CONTROLLERS; i++) {
        xSemaphoreCreateMutexStatic(&s_spi_controller[i].mMutex);
    }
}

static void spi_configure_controller(volatile NRF_SPI_Type *spi_ctrl, uint32_t hz, uint32_t mode)
{
    uint32_t i;
    uint32_t freq_reg_val = 0;
    const struct {
        uint32_t freq_reg;
        uint32_t spi_speed_hz;
    } freq_lookup_table[] = {
        { 0x02000000,   125000 },
        { 0x04000000,   250000 },
        { 0x08000000,   500000 },
        { 0x10000000,  1000000 },
        { 0x20000000,  2000000 },
        { 0x40000000,  4000000 },
        { 0x80000000,  8000000 }
    };

    /* From spec doc, SPI frequency options are limited to a fixed set of values:
     *   FREQUENCY reg               Speed
     *   0x02000000                  125kbps
     *   0x04000000                  250kbps
     *   0x08000000                  500kbps
     *   0x10000000                  1 Mbps
     *   0x20000000                  2 Mbps
     *   0x40000000                  4 Mbps
     *   0x80000000                  8 Mbps
     *
     * The EasyDMA equiped SPIM3 controller has additional support for
     * the two higher speeds:
     *   0x0A0000000                 16 Mbps
     *   0x140000000                 32 Mbps
     */
    for (i = 0; i < ARRAY_SIZE(freq_lookup_table); i++)
    {
        if (freq_lookup_table[i].spi_speed_hz == hz)
        {
            freq_reg_val = freq_lookup_table[i].freq_reg;
            break;
        }
    }
    assert(freq_reg_val != 0);
    spi_ctrl->FREQUENCY = freq_reg_val;

    /* mode for phase currently not supported, always using SPI_MODE_0 */
    spi_ctrl->CONFIG = 0;
}

static void spi_release_controller(volatile NRF_SPI_Type *spi_ctrl)
{
    /* Disable the controller */
    spi_ctrl->ENABLE = SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos;
}

/* Currently we hold the lock for the duration of the request.
 * In the future, we should move the lock to just the transaction
 * to be more like I2C.
 */
int nlspi_request(const nlspi_slave_t *spi_slave)
{
    const spi_info_t *spi_info = &s_spi_info[spi_slave->controller_id];
    volatile NRF_SPI_Type *spi_ctrl = spi_info->spi_controller;

#if USE_RTOS
    nlspi_controller_t *spi_ctrl_ctx = &s_spi_controller[spi_slave->controller_id];

    // Lock session mutex
    xSemaphoreTake(&spi_ctrl_ctx->mMutex, portMAX_DELAY);
#endif

    // Set SPI pins
    nlgpio_request(spi_info->MISO_pin, GPIOF_IN);
    nlgpio_request(spi_info->SCLK_pin, GPIOF_OUT_LOW);
    spi_ctrl->PSEL.SCK = spi_info->SCLK_pin;
    spi_ctrl->PSEL.MOSI = spi_info->MOSI_pin;
    spi_ctrl->PSEL.MISO = spi_info->MISO_pin;

    spi_configure_controller(spi_info->spi_controller, spi_slave->max_freq_hz, spi_slave->mode);

    // call product specific enable if specified
    if ((spi_slave->flags & SPI_FLAG_EXTERNAL_ENABLE) == 0) {
        (*spi_slave->enable_fp)(spi_slave);
    }

    // once configured as MOSI, the normal state of the pin is high,
    // which can backpower the slaves, so enable it last, after
    // power was enabled in enable_fp() (if needed).
    nlgpio_request(spi_info->MOSI_pin, GPIOF_OUT_LOW);

    spi_ctrl->EVENTS_READY = 0;
    spi_ctrl->ENABLE = SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos;

#if defined(BUILD_FEATURE_NL_PROFILE) && !defined(NL_NO_RTOS)
    nl_profile_start(nl_profile_spi_controller_mapping[spi_slave->controller_id]);
#endif

    return 0;
}

int nlspi_release(const nlspi_slave_t *spi_slave)
{
    const spi_info_t *spi_info = &s_spi_info[spi_slave->controller_id];
    volatile NRF_SPI_Type *spi_ctrl = spi_info->spi_controller;

    // MOSI is default high so turn off MOSI first to prevent
    // backpowering the slaves chips when we turn off any regulator
    // (if needed) in disable_fp()
    nlgpio_release(spi_info->MOSI_pin);

    spi_release_controller(spi_ctrl);

#if defined(BUILD_FEATURE_NL_PROFILE) && !defined(NL_NO_RTOS)
    nl_profile_stop(nl_profile_spi_controller_mapping[spi_slave->controller_id]);
#endif

    // call product specific disable if specified
    if ((spi_slave->flags & SPI_FLAG_EXTERNAL_ENABLE) == 0) {
        (*spi_slave->disable_fp)(spi_slave);
    }

    // Release other SPI pins.  The idle states should be low
    // so releasing them after the product specific disable should
    // be fine.
    nlgpio_release(spi_info->MISO_pin);
    nlgpio_release(spi_info->SCLK_pin);

    spi_ctrl->PSEL.MISO = SPI_PSEL_MISO_CONNECT_Disconnected << SPI_PSEL_MISO_CONNECT_Pos;
    spi_ctrl->PSEL.MOSI = SPI_PSEL_MOSI_CONNECT_Disconnected << SPI_PSEL_MOSI_CONNECT_Pos;
    spi_ctrl->PSEL.SCK = SPI_PSEL_SCK_CONNECT_Disconnected << SPI_PSEL_SCK_CONNECT_Pos;

#if USE_RTOS
    // Unlock session mutex
    nlspi_controller_t *spi_ctrl_ctx = &s_spi_controller[spi_slave->controller_id];
    xSemaphoreGive(&spi_ctrl_ctx->mMutex);
#endif

    return 0;
}

void nlspi_slave_enable(const nlspi_slave_t *spi_slave)
{
    assert(spi_slave->flags & SPI_FLAG_EXTERNAL_ENABLE);
    (*spi_slave->enable_fp)(spi_slave);
}

void nlspi_slave_disable(const nlspi_slave_t *spi_slave)
{
    assert(spi_slave->flags & SPI_FLAG_EXTERNAL_ENABLE);
    (*spi_slave->disable_fp)(spi_slave);
}

static void read_internal(const nlspi_slave_t *spi_slave, uint8_t *buffer, size_t len)
{
    const spi_info_t *spi_info = &s_spi_info[spi_slave->controller_id];
    volatile NRF_SPI_Type *spi_ctrl = spi_info->spi_controller;

    /* use polled PIO to do transfer. */
    while (len > 0)
    {
        /* send a 0 byte to receive a byte */
        spi_ctrl->TXD = 0x00;

        while (!spi_ctrl->EVENTS_READY) {}
        spi_ctrl->EVENTS_READY = 0;

        /* get the byte received */
        *buffer++ = spi_ctrl->RXD;
        len--;
    }
}

static void write_internal(const nlspi_slave_t *spi_slave, const uint8_t *buffer, size_t len)
{
    const spi_info_t *spi_info = &s_spi_info[spi_slave->controller_id];
    volatile NRF_SPI_Type *spi_ctrl = spi_info->spi_controller;

    size_t bytesSent = 0;

    while ( bytesSent < len )
    {
        /* write byte */
        spi_ctrl->TXD = buffer[bytesSent++];

        while (!spi_ctrl->EVENTS_READY) {}
        spi_ctrl->EVENTS_READY = 0;

        /* dummy read */
        (void)spi_ctrl->RXD;
    }
}

// this is a full duplex single transfer operation.
static void transfer_internal(const nlspi_slave_t *spi_slave, nlspi_transfer_t *xfer)
{
    const uint8_t *tx = xfer->tx;
    uint8_t *rx = xfer->rx;
    unsigned int count = xfer->num;
    const spi_info_t *spi_info = &s_spi_info[spi_slave->controller_id];
    volatile NRF_SPI_Type *spi_ctrl = spi_info->spi_controller;

    if (tx && !rx)
    {
        // transmit only
        write_internal(spi_slave, tx, count);
        return;
    } else if (!tx && rx)
    {
        // receive only
        read_internal(spi_slave, rx, count);
        return;
    }

    // full-duplex transfer using PIO.  change driver
    // when we have silicon where SPIM works with 1 byte transfers.
    for (uint8_t *rx_end = rx + count; rx < rx_end; rx++) {
        uint8_t byte_received;

        /* Write out data */
        spi_ctrl->TXD = *tx++;

        /* wait for EVENTS_READY and clear it */
        while (!(spi_ctrl->EVENTS_READY)) {};
        spi_ctrl->EVENTS_READY = 0;

        /* read the data and put it in the rx buffer */
        byte_received = spi_ctrl->RXD;
        *rx = byte_received;
    }
}

int nlspi_transfer(const nlspi_slave_t *spi_slave, nlspi_transfer_t *xfers, unsigned xfer_cnt)
{
    int result = 0;
    nlspi_transfer_t *xfer;

    if (!xfer_cnt)
    {
        /* nothing to do */
        return 0;
    }

    if (!spi_slave || !xfers)
    {
        /* invalid args */
        return -1;
    }

    /* enable CS of peripheral */
    if (spi_slave->cs_pin != NO_CS_GPIO_PIN)
    {
        nlgpio_request(spi_slave->cs_pin, GPIOF_OUT_LOW);
    }

    xfer = xfers;
    while (1)
    {
        if (xfer->num > 0)
        {
            if (xfer->tx == NULL && xfer->rx == NULL)
            {
                /* invalid args */
                result = -1;
                goto done;
            }
            transfer_internal(spi_slave, xfer);
        }
        if (xfer->callback)
        {
            result = xfer->callback(xfer, result);
            if (result)
            {
                goto done;
            }
        }
        xfer_cnt--;
        if (xfer_cnt == 0)
        {
            break;
        }
        xfer = ++xfers;
    }

done:

    /* Disable CS of peripheral */
    if (spi_slave->cs_pin != NO_CS_GPIO_PIN)
    {
        nlgpio_request(spi_slave->cs_pin, GPIOF_OUT_HIGH);
    }

    return result;
}

int nlspi_write(const nlspi_slave_t *spi_slave, const uint8_t *buf, size_t len)
{
    /* Enable CS of peripheral */
    if (spi_slave->cs_pin != NO_CS_GPIO_PIN)
    {
        nlgpio_request(spi_slave->cs_pin, GPIOF_OUT_LOW);
    }

    write_internal(spi_slave, buf, len);

    /* Disable CS of peripheral */
    if (spi_slave->cs_pin != NO_CS_GPIO_PIN)
    {
        nlgpio_request(spi_slave->cs_pin, GPIOF_OUT_HIGH);
    }

    return len;
}

int nlspi_read(const nlspi_slave_t *spi_slave, uint8_t *buf, size_t len)
{
    /* Enable CS of peripheral */
    if (spi_slave->cs_pin != NO_CS_GPIO_PIN)
    {
        nlgpio_request(spi_slave->cs_pin, GPIOF_OUT_LOW);
    }

    read_internal(spi_slave, buf, len);

    /* Disable CS of peripheral */
    if (spi_slave->cs_pin != NO_CS_GPIO_PIN)
    {
        nlgpio_request(spi_slave->cs_pin, GPIOF_OUT_HIGH);
    }

    return len;
}
#endif /* NL_NUM_SPI_CONTROLLERS > 0 */
