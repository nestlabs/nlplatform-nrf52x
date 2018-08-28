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
 *      Nordic NRF52x Serial Driver for I2C/TWI master mode.
 *
 */
#include <stdio.h>
#include <errno.h>

#include <nlproduct_config.h>

#if NL_NUM_I2C_CONTROLLERS > 0

#include <nrf52x/device/nrf52840.h>
#include <nrf52x/device/nrf52840_bitfields.h>

#include <nlassert.h>
#include <nlplatform.h>
#if defined(BUILD_FEATURE_NL_PROFILE) && !defined(NL_NO_RTOS)
#include <nlplatform/nlprofile.h>
#endif
#include <nlplatform/nli2c.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

/*******************************************************************************
 * !!!!----                  IMPLEMENTATION DETAILS                  ----!!!!
 ******************************************************************************/
/*  This module implements a fully interrupt driven I2C driver using the TWIM EasyDMA
 *  and shortcut features to conduct I2C transactions.
 *
 *  We take advantage of Nordic's "event shortcuts" to have the hardware automatically
 *  issue start and stop commands on completion of writes and reads. This is useful
 *  in the repeated start case where the ISR need only be invoked for errors and
 *  completed transactions.
 *
 *  For I2C writes, NRF52x DMA/TWI hardware does not support switching of buffers
 *  between register address and data without a repeated start in between. Therefore
 *  we copy both into a TX scratch buffer first in order to send in one shot.
 */

/*******************************************************************************
 * !!!!----                 MODULE SHARED / GLOBALS                  ----!!!!
 ******************************************************************************/

#define NRF_TWIM_CLEAR_ALL_MASK (0xFFFFFFFF)
#ifndef NRF_TWIM_SCRATCH_BUFFER_SIZE
#define NRF_TWIM_SCRATCH_BUFFER_SIZE (258)
#endif

typedef struct
{
    uint8_t *buf;
    size_t len;
    uint8_t tx_scratch_buffer[NRF_TWIM_SCRATCH_BUFFER_SIZE];
} nli2c_transaction_t;

typedef struct
{
    StaticSemaphore_t session_mutex_obj;
    SemaphoreHandle_t session_mutex;
    TaskHandle_t      waiting_task_handle;
#if defined(I2C_REGULATOR_ID)
    uint8_t ref_count;
#endif
} nli2c_t;

typedef struct
{
    IRQn_Type irq_num;
    uint32_t clk_freq;
    volatile NRF_TWIM_Type *controller;
    uint8_t SDA_pin;
    uint8_t SCL_pin;
} i2c_info_t;

static nli2c_t s_i2c_context[NL_NUM_I2C_CONTROLLERS];

static const i2c_info_t s_i2c_info[] =
{
#if NL_NUM_I2C_CONTROLLERS > 0
    {
        I2C0_IRQ_NUM,
        I2C0_CLK_FREQUENCY,
        I2C0_SERIAL_CONTROLLER,
        I2C0_SDA_PIN,
        I2C0_SCL_PIN
    },
#endif
#if NL_NUM_I2C_CONTROLLERS > 1
    {
        I2C1_IRQ_NUM,
        I2C1_CLK_FREQUENCY,
        I2C1_SERIAL_CONTROLLER,
        I2C1_SDA_PIN,
        I2C1_SCL_PIN
    }
#endif
};

nlSTATIC_ASSERT_PRINT(ARRAY_SIZE(s_i2c_info) == NL_NUM_I2C_CONTROLLERS, "NL_NUM_I2C_CONTROLLERS does not match number of i2c device instances.");

/*******************************************************************************
 * !!!!----                       IRQ HANDLER                        ----!!!!
 ******************************************************************************/

static void i2c_isr(unsigned controller_id)
{
    nli2c_t *i2c_ctx = &s_i2c_context[controller_id];
    const i2c_info_t *i2c_info = &s_i2c_info[controller_id];
    volatile NRF_TWIM_Type *twi_ctrl = i2c_info->controller;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    twi_ctrl->INTEN = 0;
    nlplatform_irq_disable(i2c_info->irq_num);
    vTaskNotifyGiveFromISR(i2c_ctx->waiting_task_handle, &xHigherPriorityTaskWoken);
    i2c_ctx->waiting_task_handle = NULL;
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

#if NL_NUM_I2C_CONTROLLERS > 0
void nli2c0_isr(void);
void nli2c0_isr(void)
{
    i2c_isr(0);
}
#endif

#if NL_NUM_I2C_CONTROLLERS > 1
void nli2c1_isr(void);
void nli2c1_isr(void)
{
    i2c_isr(1);
}
#endif

/*******************************************************************************
 * !!!!----         DEVICE INITIALIZATION AND STATE CONTROL          ----!!!!
 ******************************************************************************/
/**
 * @fn int nli2c_init(void)
 * @details Initialize the I2C driver, should be called once during startup.
 *----------------------------------------------------------------------------*/
void nli2c_init(void)
{
    for (unsigned id = 0; id < NL_NUM_I2C_CONTROLLERS; id++)
    {
        s_i2c_context[id].session_mutex = xSemaphoreCreateMutexStatic(&s_i2c_context[id].session_mutex_obj);

        const i2c_info_t *i2c_info = &s_i2c_info[id];
        volatile NRF_TWIM_Type *twi_ctrl = i2c_info->controller;

        twi_ctrl->PSEL.SDA  = i2c_info->SDA_pin;
        twi_ctrl->PSEL.SCL  = i2c_info->SCL_pin;
        twi_ctrl->FREQUENCY = i2c_info->clk_freq;

#if defined(I2C_REGULATOR_ID)
        s_i2c_context[id].ref_count = 0;
#endif
    }
}

/**
 * @fn int nli2c_request(nli2c_slave_t *i2c_slave)
 * @details Enable I2C power rail regulator prior to using an I2C controller
 * to communicate with devices. Not required if no regulator is present in product.
 *----------------------------------------------------------------------------*/
int nli2c_request(const nli2c_slave_t *i2c_slave)
{
#if defined(I2C_REGULATOR_ID)
    nlASSERT((i2c_slave->controller_id < NL_NUM_I2C_CONTROLLERS));
    nli2c_t *i2c_ctx = &s_i2c_context[i2c_slave->controller_id];

    xSemaphoreTake(i2c_ctrl->session_mutex, portMAX_DELAY);
    nlASSERT(i2c_ctx->ref_count < UINT8_MAX);
    if (i2c_ctx->ref_count == 0)
    {
        nlregulator_enable(I2C_REGULATOR_ID);
    }
    i2c_ctx->ref_count++;
    xSemaphoreGive(i2c_ctrl->session_mutex);
#endif
    return 0;
}

/**
 * @fn int nli2c_release(const nli2c_slave_t *i2c_slave)
 * @details Disable I2C power rail regulator after using an I2C controller
 * to communicate with devices. Not required if no regulator is present in product.
 *----------------------------------------------------------------------------*/
int nli2c_release(const nli2c_slave_t *i2c_slave)
{
#if defined(I2C_REGULATOR_ID)
    nlASSERT((i2c_slave->controller_id < NL_NUM_I2C_CONTROLLERS));
    nli2c_t *i2c_ctx = &s_i2c_context[i2c_slave->controller_id];

    xSemaphoreTake(i2c_ctrl->session_mutex, portMAX_DELAY);
    nlASSERT(i2c_ctx->ref_count > 0);
    i2c_ctx->ref_count--;
    if (i2c_ctx->ref_count == 0)
    {
        nlregulator_disable(I2C_REGULATOR_ID);
    }
    xSemaphoreGive(i2c_ctrl->session_mutex);
#endif
    return 0;
}

/*******************************************************************************
 * !!!!----                 DATA TRANSFER MANAGEMENT                 ----!!!!
 ******************************************************************************/

static void nli2c_configure_and_start_write_locked(volatile NRF_TWIM_Type *twi_ctrl, nli2c_transaction_t *transaction, uint8_t reg_addr_size)
{
    if (reg_addr_size || !nlplatform_in_data_ram(transaction->buf, transaction->len))
    {
        nlASSERT(reg_addr_size + transaction->len <= NRF_TWIM_SCRATCH_BUFFER_SIZE);
        memcpy(&transaction->tx_scratch_buffer[reg_addr_size], transaction->buf, transaction->len);
        transaction->len += reg_addr_size;
        twi_ctrl->TXD.PTR = (uintptr_t)transaction->tx_scratch_buffer;
    }
    else
    {
        twi_ctrl->TXD.PTR = (uintptr_t)transaction->buf;
    }

    twi_ctrl->TXD.MAXCNT    = transaction->len;
    twi_ctrl->SHORTS        = TWIM_SHORTS_LASTTX_STOP_Msk;
    twi_ctrl->TASKS_STARTTX = 1;
}

static void nli2c_configure_and_start_read_locked(volatile NRF_TWIM_Type *twi_ctrl, nli2c_transaction_t *transaction, uint8_t reg_addr_size)
{
    twi_ctrl->RXD.PTR    = (uintptr_t)transaction->buf;
    twi_ctrl->RXD.MAXCNT = transaction->len;

    if (reg_addr_size)
    {
        twi_ctrl->TXD.PTR       = (uintptr_t)transaction->tx_scratch_buffer;
        twi_ctrl->TXD.MAXCNT    = reg_addr_size;
        twi_ctrl->SHORTS        = (TWIM_SHORTS_LASTTX_STARTRX_Msk | TWIM_SHORTS_LASTRX_STOP_Msk);
        twi_ctrl->TASKS_STARTTX = 1;
    }
    else
    {
        twi_ctrl->SHORTS        = TWIM_SHORTS_LASTRX_STOP_Msk;
        twi_ctrl->TASKS_STARTRX = 1;
    }
}

static int nli2c_transfer(bool write, const nli2c_slave_t *i2c_slave, uint16_t regAddr, const uint8_t *buf, size_t len)
{
    nlASSERT((i2c_slave->controller_id < NL_NUM_I2C_CONTROLLERS));
    nli2c_t *i2c_ctx = &s_i2c_context[i2c_slave->controller_id];
    nlASSERT(i2c_ctx->session_mutex != NULL);
    const i2c_info_t *i2c_info = &s_i2c_info[i2c_slave->controller_id];

    volatile NRF_TWIM_Type *twi_ctrl = i2c_info->controller;
    uint8_t reg_addr_size = i2c_slave->flags & I2C_FLAG_REG_ADDRESS_SIZE_MASK;
    int ret = 0;
    nli2c_transaction_t transaction;

    transaction.buf = (uint8_t*)buf;
    transaction.len = len;

    uint8_t *byte_ptr = (uint8_t*)&regAddr;
    if (reg_addr_size == I2C_FLAG_REG_ADDRESS_SIZE_2_BYTE)
    {
        /* Reverse byte-order of the register address such that DMA sends it MSB first */
        transaction.tx_scratch_buffer[0] = byte_ptr[1];
        transaction.tx_scratch_buffer[1] = byte_ptr[0];
    }
    else if (reg_addr_size == I2C_FLAG_REG_ADDRESS_SIZE_1_BYTE)
    {
        transaction.tx_scratch_buffer[0] = byte_ptr[0];
    }

    xSemaphoreTake(i2c_ctx->session_mutex, portMAX_DELAY);
#if defined(I2C_REGULATOR_ID)
    nlASSERT(i2c_ctx->ref_count > 0);
#endif
    i2c_ctx->waiting_task_handle = xTaskGetCurrentTaskHandle();

    /* Clear any shortcuts, events, and errors */
    twi_ctrl->SHORTS         = 0;
    twi_ctrl->ERRORSRC       = NRF_TWIM_CLEAR_ALL_MASK;
    twi_ctrl->EVENTS_ERROR   = 0;
    twi_ctrl->EVENTS_STOPPED = 0;
    nlplatform_irq_enable(i2c_info->irq_num);

    /* Configure and start */
    twi_ctrl->ADDRESS  = i2c_slave->slave_addr;
    twi_ctrl->INTENSET = (TWIM_INTENSET_STOPPED_Msk | TWIM_INTENSET_ERROR_Msk);
    twi_ctrl->ENABLE   = TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos;

#if defined(BUILD_FEATURE_NL_PROFILE) && !defined(NL_NO_RTOS)
    nl_profile_start(NL_PROFILE_I2C_ENABLED);
#endif

    if (write)
    {
        nli2c_configure_and_start_write_locked(twi_ctrl, &transaction, reg_addr_size);
    }
    else
    {
        nli2c_configure_and_start_read_locked(twi_ctrl, &transaction, reg_addr_size);
    }

    /* Wait for error or completion in ISR to unblock us */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (twi_ctrl->EVENTS_ERROR)
    {
        ret = twi_ctrl->ERRORSRC;
        twi_ctrl->TASKS_STOP = 1;
        while (!twi_ctrl->EVENTS_STOPPED) {}
    }

    twi_ctrl->ENABLE = TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos;

#if defined(BUILD_FEATURE_NL_PROFILE) && !defined(NL_NO_RTOS)
    nl_profile_stop(NL_PROFILE_I2C_ENABLED);
#endif

    xSemaphoreGive(i2c_ctx->session_mutex);

    return ret;
}

/*******************************************************************************
 * !!!!----         TARGET DATA READ AND WRITE (USER FACING)         ----!!!!
 ******************************************************************************/
/**
 * @fn int nli2c_write(const nli2c_slave_t *i2c_slave, uint16_t regAddr, const uint8_t *buf, size_t len)
 * @details Write one or more bytes of data to the specified I2C device at
 * the specified register address.
 *  Blocks until a transfer has completed and will return a nonzero value and
 * close the bus transaction if any errors are encountered.
 *----------------------------------------------------------------------------*/
int nli2c_write(const nli2c_slave_t *i2c_slave, uint16_t regAddr, const uint8_t *buf, size_t len, nli2c_handler_t callback)
{
    /* callback not supported in synchronous implementation */
    nlASSERT(callback == NULL);
    return nli2c_transfer(true, i2c_slave, regAddr, buf, len);
}

/**
 * @fn int nli2c_read(const nli2c_slave_t *i2c_slave, uint16_t regAddr, uint8_t *buf, size_t len)
 * @details Read one or more bytes of data from the specified I2C device from
 * the specified register address.
 *  Blocks until a transfer has completed and will return a nonzero value and
 * close the bus transaction if any errors are encountered.
 *----------------------------------------------------------------------------*/
int nli2c_read(const nli2c_slave_t *i2c_slave, uint16_t regAddr, uint8_t *buf, size_t len, nli2c_handler_t callback)
{
    /* callback not supported in synchronous implementation */
    nlASSERT(callback == NULL);
    nlASSERT(nlplatform_in_data_ram(buf, len));
    return nli2c_transfer(false, i2c_slave, regAddr, buf, len);
}

#else /* NL_NUM_I2C_CONTROLLERS > 0 */

/**
 * @fn int nli2c_init(void)
 * @details Stub function if no I2C controllers are specified
 *----------------------------------------------------------------------------*/
void nli2c_init(void)
{
}

#endif /* NL_NUM_I2C_CONTROLLERS > 0 */
