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
 *
 *    Description:
 *      Nordic NRF52x UARTE driver.  We don't use the non-DMA capable UART block.
 *
 */
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <nlplatform.h>
#include <nlplatform/nluart.h>
#include <nlplatform/nlgpio.h>
#include <nlplatform/nlwatchdog.h>

#if defined(BUILD_FEATURE_NL_PROFILE) && !defined(NL_NO_RTOS)
#include <nlplatform/nlprofile.h>
#endif

#include <nlplatform_nrf52x/nrf52x_gpio.h>

#include <nrf52x/device/nrf52840.h>
#include <nrf52x/device/nrf52840_bitfields.h>

#ifdef BUILD_FEATURE_SWO_VIRTUAL_UART
#include "../include/swo_vuart.h"
#endif

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <nlassert.h>

/*******************************************************************************
 * !!!!----                  IMPLEMENTATION DETAILS                  ----!!!!
 ******************************************************************************/
/*  This module will either use a full interrupt-driven and buffered setup
 * (asynchronous) for receive or polling #ifndef NL_UART_IS_ASYNC. The
 * asynchronous target may be forced (permanently) into a synchronous
 * mode by calling forceSync(), which is intended for use in fault reports
 * or other dead-end scenarios.
 *
 *  There are two types of UART controller blocks, one which is DMA enabled (UARTE),
 * and the other which is not (UART). The non DMA enabled block is considered
 * deprecated by the documentation so we only use UART.
 *
 *  There are two instances of UARTE controller, UARTE0 and UARTE1.
 * The docs indicate there is a 4-byte RX FIFO, but no mention of a TX FIFO.
 *
 * In non-NL_UART_IS_ASYNC mode, we just receive one byte at a time using DMA,
 * don't use interrupts, and poll for the ENDRX event.
 * In NL_UART_IS_ASYNC mode, we still just receive one byte at at time using DMA
 * because our API is such that the interested tasks need to know about data as
 * soon as it is available, and the HW can only report transferred bytes after
 * the DMA is completed.  We use an interrupt to start a new DMA transfer
 * into the next index of our RX buffer until it is full, but we
 * enable any waiting task as soon as we have bytes in our RX buffer to read.
 *
 *   Transmits are currently not SW buffered.  Async only applies to whether
 * the nluart_putchar() function waits for the character to be flushed out
 * of the HW FIFO or not.  nluart_putchar() is not interrupt based so it can
 * be used with interrupts enabled.  It will always spin until there is space
 * in the HW FIFO to add the character given.
 *
 * ---- Callbacks ----
 *  Callbacks are supported for two scenarios : character receive and start
 * bit detection during sleep mode, the latter of which is intended for use
 * as a wake from sleep handler.
 *  Both scenarios will supply the callback function with the base address of
 * the pertinent UART through the pDevice argument.
 *
 *  A callback to the weakly defined uart_callback(Uart *pDevice) upon the
 * reception of a character may be requested by calling reqRxAlert(). This is
 * a single-shot callback and will be automatically disabled after execution.
 */

/*******************************************************************************
 * !!!!----                 MODULE SHARED / GLOBALS                  ----!!!!
 ******************************************************************************/
typedef struct uart_config_s
{
    uint8_t TXD_pin;
    uint8_t RXD_pin;
    uint8_t CTS_pin;
    uint8_t RTS_pin;
    IRQn_Type irq_num;
    volatile NRF_UARTE_Type *uart_controller;
} uart_config_t;

static uart_config_t const s_uart_config[] =
{
    {
        UART0_TX_PIN,
        UART0_RX_PIN,
        UART0_CTS_PIN,
        UART0_RTS_PIN,
        UART0_IRQ_NUM,
        (volatile NRF_UARTE_Type *)UART0_CONTROLLER,
    },
#ifdef UART1_TX_PIN
    {
        UART1_TX_PIN,
        UART1_RX_PIN,
        UART1_CTS_PIN,
        UART1_RTS_PIN,
        UART1_IRQ_NUM,
        (volatile NRF_UARTE_Type *)UART1_CONTROLLER,
    },
#endif
};

/* Maximum duration to wait for the UART to flush its outgoing
 * data when doing a double-enable (for baud rate changes, etc).
 */
#define NL_UART_ENABLE_FLUSH_MS         1000

/* A note about cached baud rate- since we're so memory constrained the UART's
 * configured baud rate will be divided by NL_UART_BAUD_CACHE_SCALAR and
 * stored in a 16-Bit unsigned integer.
 * This value also serves a double purpose of tracking whether or not the
 * UART has been enabled.
 */
#define NL_UART_BAUD_CACHE_MAX          0xFFFF
#define NL_UART_BAUD_CACHE_SCALAR       100

#ifdef NL_UART_IS_ASYNC
    #ifndef NL_UART_RXBUF_LEN
        #ifdef BUILD_CONFIG_DIAGNOSTICS
            #define NL_UART_RXBUF_LEN		64
        #else
            #define NL_UART_RXBUF_LEN		8
        #endif
    #endif
#else  /* NL_UART_IS_ASYNC */
    #ifndef NL_UART_RXBUF_LEN
        #define NL_UART_RXBUF_LEN		2
    #endif
#endif /* NL_UART_IS_ASYNC */

// For now, we only support a 2 byte transfer (one for '\r' if nl_cr_enable is set
#define NL_UART_TXBUF_LEN		2

typedef struct nluart_s
{
    uint16_t baud_cache;
    uint8_t  nl_cr_enable:1;
    uint8_t  flow_control_enable:1;
    uint8_t  echo_recv_chars:1;
    uint8_t  rx_data[NL_UART_RXBUF_LEN];
    uint8_t  tx_data[NL_UART_TXBUF_LEN];
#if NL_UART_IS_ASYNC
    uint8_t  force_sync:1;
    uint8_t  suspended:1;

    volatile int8_t rx_req;
    volatile uint8_t rx_in, rx_out;

    nluart_wakeup_t wakeup_callback;
    nluart_rx_t rx_callback;

    StaticSemaphore_t mutex; // serialize high-level UART operations
    TaskHandle_t      task_waiting_for_recv;
#endif
} nluart_t;

static nluart_t s_uart_controller[NL_NUM_UART_CONTROLLERS];

#ifdef NL_UART_IS_ASYNC

#define NL_UART_MUTEX_LOCK_WITH_RESULT(uart_id, ticks) \
    ((!nlplatform_in_interrupt() && (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)) ? \
        (xSemaphoreTake(&s_uart_controller[uart_id].mutex, (ticks)) == pdTRUE) : true)
#define NL_UART_MUTEX_UNLOCK_WITH_RESULT(uart_id) \
    ((!nlplatform_in_interrupt() && (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)) ? \
        (xSemaphoreGive(&s_uart_controller[uart_id].mutex) == pdTRUE) : true)

#define NL_UART_MUTEX_LOCK(uart_id) \
    ((void) NL_UART_MUTEX_LOCK_WITH_RESULT((uart_id), portMAX_DELAY))
#define NL_UART_MUTEX_UNLOCK(uart_id) \
    ((void) NL_UART_MUTEX_UNLOCK_WITH_RESULT(uart_id))

#else // NL_UART_IS_ASYNC

#define NL_UART_MUTEX_LOCK_WITH_RESULT(uart_id, ticks) (true)
#define NL_UART_MUTEX_UNLOCK_WITH_RESULT(uart_id)      (true)
#define NL_UART_MUTEX_LOCK(uart_id)
#define NL_UART_MUTEX_UNLOCK(uart_id)

#endif // NL_UART_IS_ASYNC

/*******************************************************************************
 * !!!!----                    RINGBUFFER INLINES                    ----!!!!
 ******************************************************************************/
#ifdef NL_UART_IS_ASYNC
static inline uint32_t uart_rxRing_increment(uint32_t index)
{
    return ((index + 1) % NL_UART_RXBUF_LEN);
}

static inline bool uart_rxRing_isFull(uint32_t inIndex, uint32_t outIndex)
{
    return (((inIndex + 1) % NL_UART_RXBUF_LEN) == outIndex);
}

static inline bool uart_rxRing_isEmpty(uint32_t inIndex, uint32_t outIndex)
{
    return (inIndex == outIndex);
}


/*******************************************************************************
 * !!!!----                       IRQ HANDLER                        ----!!!!
 ******************************************************************************/
/**
 * @fn void uarte_irq_handler(unsigned controller_id)
 * @details UARTE IRQ handler, routed by the UART0 or UART1 IRQ Handler.
 *          Handles RX completion DMA interrupt.
 *----------------------------------------------------------------------------*/
void uarte_irq_handler(const nluart_id_t uart_id);
void uarte_irq_handler(const nluart_id_t uart_id)
{
    nluart_t *uart = &s_uart_controller[uart_id];
    const uart_config_t *uart_config = &s_uart_config[uart_id];
    volatile NRF_UARTE_Type *ctrl = uart_config->uart_controller;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    int32_t iReqRxSig = 0, iGotRx = 0;

    uint32_t curRxIn, curRxOut;

    /**
     * ---- RECEIVE HANDLER ----
     */
    if (ctrl->EVENTS_ENDRX)
    {
        /* Clear event */
        ctrl->EVENTS_ENDRX = 0;
        curRxIn = uart->rx_in;
        curRxOut = uart->rx_out;

        /**
         * If this is our first receive in a series, and we're actually waiting
         * on it to arrive (0 != iReqSig), we'll need to let the (hopefully)
         * patient task know that it can wake and get our data by giving it
         * a semaphore at the end of this IRQ.
         */
        if (uart_rxRing_isEmpty(curRxIn, curRxOut) && uart->rx_req)
        {
            iReqRxSig = -1;
            uart->rx_req = 0;
        }

        /* DMA has transfered one byte into our rxRing buffer.  Increment
         * our index and if there is more room, STARTTX again with the
         * new read position.
         */
        curRxIn = uart_rxRing_increment(curRxIn);
        if (!uart_rxRing_isFull(curRxIn, curRxOut))
        {
            ctrl->RXD.PTR = (unsigned)&uart->rx_data[curRxIn];
            ctrl->TASKS_STARTRX = 1;
        }

        iGotRx = -1;
        uart->rx_in = curRxIn;
    }

    /**
     * If our UART receive alert callback is not NULL, call it.
     */
    if (iGotRx && uart->rx_callback)
    {
        uart->rx_callback(uart_id);
        uart->rx_callback = NULL;
    }

    /**
     * Clear the pending flag for the interrupt we just handled, hopefully
     * it will stay clear for awhile.
     */
    nlplatform_irq_clear_pending(uart_config->irq_num);

    /*
     * If someone was waiting on us to capture one or more
     * characters and they went to sleep, wake them up.
     * This sort of behavior happens when using getChar and no incoming data
     * has actually been received yet or using putChar and the outgoing
     * buffers were saturated.
     */
    if (iReqRxSig)
    {
        vTaskNotifyGiveFromISR(uart->task_waiting_for_recv, &xHigherPriorityTaskWoken);
        uart->task_waiting_for_recv = NULL;
    }
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void nluart0_isr(void);
void nluart0_isr(void)
{
    uarte_irq_handler(0);
}

void nluart1_isr(void);
void nluart1_isr(void)
{
    uarte_irq_handler(1);
}
#endif /* defined(NL_UART_IS_ASYNC) */

/*******************************************************************************
 * !!!!----                Forward declarations                      ----!!!!
 ******************************************************************************/
static void config_controller(const nluart_id_t uart_id, bool enable, bool warm);

/*******************************************************************************
 * !!!!----         DEVICE INITIALIZATION AND STATE CONTROL          ----!!!!
 ******************************************************************************/
/**
 * @fn void nluart_init(void)
 * @details Initialize all uarts in the product, this must
 * be done prior to enabling or disabling the UART with nluart_request
 *----------------------------------------------------------------------------*/
void nluart_init(void)
{
    unsigned i;
    for (i = 0; i < NL_NUM_UART_CONTROLLERS; i++) {
#ifdef NL_UART_IS_ASYNC
        xSemaphoreCreateMutexStatic(&s_uart_controller[i].mutex);
#endif
    }
#ifdef BUILD_FEATURE_SWO_VIRTUAL_UART
    swo_vuart_init();
#endif

}

/**
 * @fn int nluart_request(unsigned uart_id, const nluart_config_t *config)
 * @details High level device enable or disable based upon the supplied configuration.
 *----------------------------------------------------------------------------*/
int nluart_request(const nluart_id_t uart_id, const nluart_config_t *config)
{
    int retval = 0;
    nluart_t *uart;
#ifdef BUILD_FEATURE_SWO_VIRTUAL_UART
    if (uart_id == SWO_VIRTUAL_UART_ID) {
        return swo_vuart_request(config);
    }
#endif
    uart = &s_uart_controller[uart_id];
    uint32_t baudDiv;

    NL_UART_MUTEX_LOCK(uart_id);

    /**
     * If we were previously enabled, flush our pending incoming and
     * outgoing data streams before proceeding. This protects against
     * double enable issues for things such as changing baud rate.
     *
     * In the case our flush fails, kill the UART's previous state
     * outright with a hardware disable and re-enable.
     */
    if (uart->baud_cache)
    {
        if (!nluart_flush(uart_id, NL_UART_ENABLE_FLUSH_MS))
        {
            config_controller(uart_id, false, false);
        }
    }

    baudDiv = config->baud_rate / NL_UART_BAUD_CACHE_SCALAR;
    nlREQUIRE_ACTION(((0 == (config->baud_rate % NL_UART_BAUD_CACHE_SCALAR)) &&
              (NL_UART_BAUD_CACHE_MAX >= baudDiv)), exit, retval = -EINVAL);

    uart->baud_cache = baudDiv;
    uart->nl_cr_enable = config->nl_cr_enable;
    uart->flow_control_enable = config->flow_control_enable;
    uart->echo_recv_chars = config->echo_recv_chars;

    config_controller(uart_id, true, false);

exit:
    NL_UART_MUTEX_UNLOCK(uart_id);
    return retval;
}

int nluart_release(const nluart_id_t uart_id)
{
#ifdef BUILD_FEATURE_SWO_VIRTUAL_UART
    if (uart_id == SWO_VIRTUAL_UART_ID) {
        return swo_vuart_release();
    }
#endif
    int retval;
    nluart_t *uart = &s_uart_controller[uart_id];
    NL_UART_MUTEX_LOCK(uart_id);
    if (0 != uart->baud_cache)
    {
        config_controller(uart_id, false, false);
        uart->baud_cache = 0;

        retval = 0;
    }
    else /* not requested */
    {
        retval = -1;
    }
    NL_UART_MUTEX_UNLOCK(uart_id);
    return retval;
}

#ifdef NL_UART_IS_ASYNC
/**
 * @fn void uart_rxd_activity_irq_handler(const nlgpio_id_t gpio, void *data)
 * @details Interrupt handler when activity on RXD pin detected in idle/sleep
 *          Never called because nluart_resume() should be invoked before
 *          any interrupt handlers ever get a chance to run.
 *----------------------------------------------------------------------------*/
static void uart_rxd_activity_irq_handler(const nlgpio_id_t gpio, void *data)
{
    assert(0); /* assert if this is ever called because it's unexpected */
}
#endif /* defined(NL_UART_IS_ASYNC) */

/**
 * @fn void config_controller(const nluart_id_t uart_id, bool enable, bool warm)
 * @details Apply a UART's configuration to the actual hardware and any related
 * software components, controlling the enabling, disabling, suspension (for
 * sleep modes), and resumption of UART activity.
 *
 *  Performs the following actions based upon its two arguments:
 *  Config     |    enable      -   warm
 *-------------+-------------------------------
 * -Enable     |    true        -   false
 * -Disable    |    false       -   false
 * -Resume     |    true        -   true
 * -Suspend    |    false       -   true
 *----------------------------------------------------------------------------*/
static void config_controller(const nluart_id_t uart_id, bool enable, bool warm)
{
    nluart_t *uart = &s_uart_controller[uart_id];
    const uart_config_t *uart_config = &s_uart_config[uart_id];
    volatile NRF_UARTE_Type *ctrl = uart_config->uart_controller;

    uint32_t baud;

    if (enable)
    {
        /**
         * ---- ENABLE UART ----
         */
        uint32_t baudrate_reg_val = 0;
#ifdef NL_UART_IS_ASYNC
        bool rx_irq_detected_in_sleep;
#endif

        /* Extract our baud rate from the baud cache */
        baud = uart->baud_cache * NL_UART_BAUD_CACHE_SCALAR;

        /* Values from 34.10.11 BAUDRATE register description.
         * Note UARTE values different from the older UART
         * controller's values.
         */
        const struct {
            uint32_t baudrate_reg;
            uint32_t uart_baudrate;
        } baudrate_lookup_table[] = {
            { 0x00EB0000,    57600 }, /* actual rate:  55944 */
            { 0x01D60000,   115200 }, /* actual rate: 115108 */
            { 0x03B00000,   230400 }, /* actual rate: 231884 */
            { 0x07400000,   460800 }, /* actual rate: 457143 */
            { 0x0F000000,   921600 }, /* actual rate: 941176 */
            { 0x10000000,  1000000 }
        };

        /* From spec doc, uart baud rate isn't a linear formula
         * from BAUDRATE register.  It lists a discrete set
         * of register values for specific baud rates.  We
         * use a subset since there are a number of baud rates
         * not likely to be ever needed.
         */
        for (unsigned i = 0; i < ARRAY_SIZE(baudrate_lookup_table); i++)
        {
            if (baudrate_lookup_table[i].uart_baudrate == baud)
            {
                baudrate_reg_val = baudrate_lookup_table[i].baudrate_reg;
                break;
            }
        }
        assert(baudrate_reg_val != 0);
        ctrl->BAUDRATE = baudrate_reg_val;

        /* Default : 8 bit data, 1 stop bit, no flow ctrl, no parity */
        ctrl->CONFIG = 0;

#ifdef NL_UART_IS_ASYNC
        if (warm)
        {
            /* Note if RX activity was detected while sleeping and
             * release RXD pin as gpio
             */
            rx_irq_detected_in_sleep = nlgpio_irq_pending(uart_config->RXD_pin);
            nlgpio_irq_release(uart_config->RXD_pin);
            nlgpio_release(uart_config->RXD_pin);
        }
#endif

        /**
         * Request the use of the pins
         */
        ctrl->PSEL.TXD = uart_config->TXD_pin;
        ctrl->PSEL.RXD = uart_config->RXD_pin;

        if (uart->flow_control_enable)
        {
            ctrl->CONFIG |= 1; // enable hardware flow control
            ctrl->PSEL.RTS = uart_config->RTS_pin;
            ctrl->PSEL.CTS = uart_config->CTS_pin;
        }

#ifdef NL_UART_IS_ASYNC
        /**
         * Initialize our buffer offsets, and add this UART to the
         * registry (so its members may be accessed in the IRQ handler).
         *
         * The rx_callback function is NULL by default
         * so we won't get any stray calls.
         * These should only be done if we're freshly initializing a UART,
         * not restoring it out of a suspended state.
         */
        if (!warm)
        {
            uart->rx_in = 0;
            uart->rx_out = 0;
            uart->rx_req = 0;

            uart->rx_callback = NULL;
        }

        /* Clear any pending ENDRX interrupt just in case. */
        ctrl->INTENCLR = UARTE_INTENSET_ENDRX_Msk;

        /* Clear and enable NVIC level interrupt */
        nlplatform_irq_clear_pending(uart_config->irq_num);
        nlplatform_irq_enable(uart_config->irq_num);

        /* Enable ENDRX interrupt and STARTRX. */
        ctrl->INTENSET = UARTE_INTENSET_ENDRX_Msk;
#else /* !defined(NL_UART_IS_ASYNC) */
        /* For non-ASYNC, we still use DMA to transfer but check for
         * data using POLLED IO instead of interrupt
         */
#endif /* defined(NL_UART_IS_ASYNC) */

        // setup TXD.PTR, always the same right now
        ctrl->TXD.PTR = (unsigned)&uart->tx_data[0];

        // Enable UART
#if defined(BUILD_FEATURE_NL_PROFILE) && !defined(NL_NO_RTOS)
        nl_profile_start(nl_profile_uart_controller_mapping[uart_id]);
#endif
        ctrl->ENABLE = UARTE_ENABLE_ENABLE_Enabled;
        // Start receiving using DMA, transfer count of 1
        ctrl->EVENTS_ENDRX = 0;
        // clear any old ENDTX and TXSTARTED events
        ctrl->EVENTS_ENDTX = 0;
        ctrl->EVENTS_TXSTARTED = 0;
        if (!warm)
        {
            ctrl->RXD.PTR = (unsigned)&uart->rx_data[0];
            ctrl->RXD.MAXCNT = 1;
        }
        ctrl->TASKS_STARTRX = 1;
#if !defined(NL_BOOTLOADER)
        /* if we're resuming and have a wakeup handler and the wakeup cause
         * was activity on uart rxd pin, then invoke wakeup callback
         */
        if (warm && uart->wakeup_callback && rx_irq_detected_in_sleep)
        {
            uart->wakeup_callback(uart_id);
        }
#endif /* !defined(NL_BOOTLOADER) */
    }
    else
    {
        /**
         * ---- DISABLE UART ----
         * If we're disabling or suspending the UART, release its pins
         *
         * Note that RTS and CTS are always released regardless of whether
         * or not the UART is using flow control.
         */

#ifdef NL_UART_IS_ASYNC
        /* STOPRX may trigger an ENDRX so disable interrupt first.
         * Otherwise, there could be a pending ENDRX interrupt since
         * interrupts are generally blocked when we're called to suspend
         * so our ISRHandler won't run to clear it. Resume will enable
         * the interrupt again.
         */
        ctrl->INTENCLR = UARTE_INTENSET_ENDRX_Msk;
#endif

        // Request TX STOP and wait until it is.
        ctrl->EVENTS_TXSTOPPED = 0;
        do
        {
            ctrl->TASKS_STOPTX = 1;
        } while (!ctrl->EVENTS_TXSTOPPED);

        // Request RX STOP and wait until it is
        ctrl->EVENTS_RXTO = 0;
        do
        {
            ctrl->TASKS_STOPRX = 1;
        } while (!ctrl->EVENTS_RXTO);

        // Disable UART
        ctrl->ENABLE = 0;
#if defined(BUILD_FEATURE_NL_PROFILE) && !defined(NL_NO_RTOS)
        nl_profile_stop(nl_profile_uart_controller_mapping[uart_id]);
#endif

        // Release pins
        ctrl->PSEL.TXD = 0xffffffff;
        ctrl->PSEL.RXD = 0xffffffff;
        ctrl->PSEL.RTS = 0xffffffff;
        ctrl->PSEL.CTS = 0xffffffff;

#ifdef NL_UART_IS_ASYNC
        if (!warm) {
            uart->wakeup_callback = NULL;
        }
#endif

#ifdef NL_UART_IS_ASYNC
        nlplatform_irq_disable(uart_config->irq_num);

        // if this is a suspend, configure the RX pin as a GPIO
        // with edge both triggering to detect activity for wakeup
        if (warm)
        {
            // if the UART is connected to a external cable like
            // in a dev board use case, and if the external cable
            // is not matched in voltage, enabling
            // any kind of pull could cause leakage current.
            // However, not having any pull could cause the line
            // to float and cause spurious wakeup. It's up to higher
            // level code to release the UART so suspend won't
            // be called and we don't try to detect activity
            // from a floating input case, or the tester should
            // be aware of the potential leakage current and disconnect
            // the external UART cable.
            nlgpio_request(uart_config->RXD_pin, GPIOF_IN);
            nlgpio_irq_request(uart_config->RXD_pin, IRQF_TRIGGER_BOTH,
                               uart_rxd_activity_irq_handler, uart);
        }
#endif
    }
}

#ifdef NL_UART_IS_ASYNC
/*******************************************************************************
 * !!!!----             RECEIVE ALERT CALLBACK MANAGEMENT            ----!!!!
 ******************************************************************************/
/**
 * @fn void nluart_set_rx_callback(const nluart_id_t uart_id, nluart_rx_t callback)
 * @details Request the receive alert callback be called once when any incoming
 * characters are received. It will automatically be disabled afterwards.
 *----------------------------------------------------------------------------*/
void nluart_set_rx_callback(const nluart_id_t uart_id, nluart_rx_t callback)
{
#ifdef BUILD_FEATURE_SWO_VIRTUAL_UART
    if (uart_id == SWO_VIRTUAL_UART_ID) {
        swo_vuart_set_rx_callback(callback);
        return;
    }
#endif
    volatile NRF_UARTE_Type *ctrl = s_uart_config[uart_id].uart_controller;
    nluart_t *uart = &s_uart_controller[uart_id];
    NL_UART_MUTEX_LOCK(uart_id);
    uart->rx_callback = callback;
    // enable ENDRX event interrupt, which will trigger the callback on next
    // received byte
    ctrl->INTENSET = UARTE_INTENSET_ENDRX_Msk;
    NL_UART_MUTEX_UNLOCK(uart_id);
}

/*******************************************************************************
 * !!!!----             WAKEUP ALERT CALLBACK MANAGEMENT            ----!!!!
 ******************************************************************************/
/**
 * @fn void nluart_set_wakeup_callback(const nluart_id_t uart_id, nluart_wakeup_t callback)
 * @details Request the function whose base address is at pTarget be called
 * when a falling edge is detected on a UART's RXD pin in a suspened state.
 *
 * This can be used as a "Wake via UART RXD" notification. Keep in mind that
 * the callback will not be configured until the UART is suspended
 * via halInternalPowerDownUart, and the callback should be disabled as soon
 * possibl after the device wakes via relWakeUpAlert().
 *----------------------------------------------------------------------------*/
void nluart_set_wakeup_callback(const nluart_id_t uart_id, nluart_wakeup_t callback)
{
#ifdef BUILD_FEATURE_SWO_VIRTUAL_UART
    if (uart_id == SWO_VIRTUAL_UART_ID) {
        swo_vuart_set_wakeup_callback(callback);
        return;
    }
#endif
    nluart_t *uart = &s_uart_controller[uart_id];
    NL_UART_MUTEX_LOCK(uart_id);
    uart->wakeup_callback = callback;
    NL_UART_MUTEX_UNLOCK(uart_id);
}
#endif /* defined(NL_UART_IS_ASYNC) */


/*******************************************************************************
 * !!!!----           CHARACTER FETCH AND SEND (USER FACING)         ----!!!!
 ******************************************************************************/

/**
 * @fn void flush_tx(volatile NRF_UARTE_Type *ctrl)
 * @details Helper function to wait until all characters being transmitted
 * have been sent.
 */
static void flush_tx(volatile NRF_UARTE_Type *ctrl)
{
    if (ctrl->EVENTS_TXSTARTED)
    {
        // for non-async, we wait for the character to be sent out of the FIFO
        while (!ctrl->EVENTS_ENDTX);
        // clear ENDTX event once we've seen it
        ctrl->EVENTS_ENDTX = 0;
        ctrl->EVENTS_TXSTARTED = 0;
    }
}

/**
 * @fn int nluart_putchar(const nluart_id_t uart_id, uint8_t ch, unsigned timeout_ms = 0)
 * @details Output a single character inChar over this UART's transmit line,
 * will block if HW FIFO is full, otherwise just puts the character in the
 * FIFO and returns immediately.  If the caller wants to make sure the
 * character has been already sent (like before a reset), then they should
 * call the nluart_flush() function.  The timeout_ms value is ignored
 * in this implementation because we expect there to never be a deadlock
 * and the UART will eventually empty it's relatively small FIFO and have
 * room for another character.
 *----------------------------------------------------------------------------*/
int nluart_putchar(const nluart_id_t uart_id, uint8_t ch, unsigned timeout_ms)
{
#ifdef BUILD_FEATURE_SWO_VIRTUAL_UART
    if (uart_id == SWO_VIRTUAL_UART_ID) {
        return swo_vuart_putchar(ch, timeout_ms);
    }
#endif
    int retval = 1; // One character sent
    nluart_t *uart = &s_uart_controller[uart_id];
    const uart_config_t *uart_config = &s_uart_config[uart_id];
    volatile NRF_UARTE_Type *ctrl = uart_config->uart_controller;
    uint8_t tx_count = 0;
#if NL_UART_IS_ASYNC
    TickType_t timeout_in_ticks;
    TickType_t xTimeOut;

    if (timeout_ms == 0)
    {
        timeout_in_ticks = 0;
        xTimeOut = portMAX_DELAY;
    }
    else
    {
        timeout_in_ticks = MS_TO_TICK_DELAY(timeout_ms);
        xTimeOut = timeout_in_ticks;
    }
#endif

    nlEXPECT_ACTION(
        NL_UART_MUTEX_LOCK_WITH_RESULT(uart_id, xTimeOut),
            exit_no_unlock,
            retval = -ETIMEDOUT);

    // check if uart is enabled or not.
    nlEXPECT_ACTION(ctrl->ENABLE != 0, exit, retval = -1);

    /** Prepend CRs to LFs if enabled */
    if (('\n' == ch) && uart->nl_cr_enable)
    {
        uart->tx_data[0] = '\r';
        tx_count++;
    }
    uart->tx_data[tx_count++] = ch;

#ifdef NL_UART_IS_ASYNC
    // make sure previous TX is stopped if not doing force sync
    if (!uart->force_sync)
    {
        if (ctrl->EVENTS_TXSTARTED)
        {
            // for non-async, we wait for the character to be sent out of the FIFO
            while (!ctrl->EVENTS_ENDTX);
        }
    }
#endif

    // Set count of bytes we're transmitting
    ctrl->TXD.MAXCNT = tx_count;
    // clear ENDTX and TXSTARTED events
    ctrl->EVENTS_ENDTX = 0;
    ctrl->EVENTS_TXSTARTED = 0;
    // Start TX
    ctrl->TASKS_STARTTX = 1;
#ifdef NL_UART_IS_ASYNC
    // if async, but force_sync is requested, wait for
    // character to empty out of FIFO.  if async but not
    // forced, don't wait.  caller would have to call
    // nluart_flush() to make sure FIFO is emptied.
    // If no async, then always wait.
    if (uart->force_sync)
#endif
    {
        flush_tx(ctrl);
    }

exit:
    NL_UART_MUTEX_UNLOCK(uart_id);

exit_no_unlock:
    return retval;
}

/**
 * @fn int nluart_getchar(const nluart_id_t uart_id, uint8_t *out, unsigned timeout_ms)
 * @details Fetch a character over this particular UART, placing it in *outChar
 * and returns the number of characters received. If buffering is enabled (async
 * target), then the character returned will be the oldest one located in the
 * incoming data buffer.
 *  Will block until a character is available if one isn't already, a timeout
 * in milliseconds may be specified by providing a nonzero value for the
 * defaulted argument timeout_ms (zero specifies an indefinite wait time).
 *  Please note that timeouts are an async-only feature.
 *----------------------------------------------------------------------------*/
int nluart_getchar(const nluart_id_t uart_id, uint8_t *ch, unsigned timeout_ms)
{
#ifdef BUILD_FEATURE_SWO_VIRTUAL_UART
    if (uart_id == SWO_VIRTUAL_UART_ID) {
        return swo_vuart_getchar(ch, timeout_ms);
    }
#endif
    nluart_t *uart = &s_uart_controller[uart_id];
    const uart_config_t *uart_config = &s_uart_config[uart_id];
    volatile NRF_UARTE_Type *ctrl = uart_config->uart_controller;

    int iFetched = 1;
#ifdef NL_UART_IS_ASYNC
    TickType_t timeout_in_ticks = MS_TO_TICK_DELAY(timeout_ms);
    TickType_t xTimeOut = (timeout_ms == 0) ? portMAX_DELAY : timeout_in_ticks;
#endif

    nlEXPECT_ACTION(
        NL_UART_MUTEX_LOCK_WITH_RESULT(uart_id, xTimeOut),
            exit_no_unlock,
            iFetched = 0);

#ifdef NL_UART_IS_ASYNC
    uint32_t nextRxOut, prevRxOut;

    /**
     * Check to see if this is our first receive (buffer is empty), in which
     * case we'll let the IRQ wake us up later.
     *
     * We'll briefly disable IRQs this check is done to ensure we don't issue
     * a sem request while a character is already being received.
     */
    nextRxOut = uart_rxRing_increment(uart->rx_out);
    prevRxOut = uart->rx_out;

    nlplatform_interrupt_disable();
    if (uart_rxRing_isEmpty(uart->rx_in, uart->rx_out))
    {
        uart->rx_req = -1;
        // before enabling interrupt, set task_waiting_for_recv
        // so if the character comes in right after we enable the
        // interrupt, we don't block unnecessarily.
        uart->task_waiting_for_recv = xTaskGetCurrentTaskHandle();
        nlplatform_interrupt_enable();

        // block until we have data to read
        if (ulTaskNotifyTake(pdTRUE, xTimeOut) == 0)
        {
            /**
             * If we didn't get a response within our programmed timeout,
             * disable the fetch notification request and make ONE final attempt
             * to grab the TaskNotification in case the receive handler gave it to us
             * between the last check and here.
             */
            iFetched = 0;
            uart->rx_req = 0;

            ulTaskNotifyTake(pdTRUE, 0);
        }
    }
    else
    {
        nlplatform_interrupt_enable();
    }

    /**
     * Fetch our least recently received character from the intermediary buffer
     * and re-enable the receive interrupt if we're leaving a saturated condition.
     */
    if (iFetched > 0)
    {
        *ch = uart->rx_data[uart->rx_out];
        uart->rx_out = nextRxOut;
        if (uart_rxRing_isFull(uart->rx_in, prevRxOut))
        {
            // we have read all the bytes in the rxRing buffer,
            // start RX again
            ctrl->TASKS_STARTRX = 1;
        }
    }
#else /* !defined(NL_UART_IS_ASYNC) */
    /**
     * When doing polled receive, just sit and wait for the character to arrive.
     * To reduce power consumption, enable interrupt and use WFE. Might
     * be able to save even more power by disabling UARTE and using GPIO
     * activity detection to wake, but this is less complicated and good
     * enough for bootloader cmd mode.
     */
    while (ctrl->EVENTS_ENDRX == 0)
    {
        nlplatform_interrupt_disable();
        nlplatform_irq_clear_pending(uart_config->irq_num);
        nlplatform_irq_enable(uart_config->irq_num);
        ctrl->INTENSET = UARTE_INTENSET_ENDRX_Msk;
        /* check state again now that interrupts are disabled, just in case
         * the event came in late
         */
        if (ctrl->EVENTS_ENDRX == 0)
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
        ctrl->INTENCLR = UARTE_INTENSET_ENDRX_Msk;
        nlplatform_irq_disable(uart_config->irq_num);
        nlplatform_irq_clear_pending(uart_config->irq_num);
        nlplatform_interrupt_enable();
    }
    *ch = (uint8_t)uart->rx_data[0];
    /**
     * Start next receive
     */
    ctrl->EVENTS_ENDRX = 0;
    ctrl->TASKS_STARTRX = 1;
#endif /* defined(NL_UART_IS_ASYNC) */

    NL_UART_MUTEX_UNLOCK(uart_id);

    /** If we're in echo mode, send the received characters back out. */
    if (iFetched > 0)
    {
        if (uart->echo_recv_chars)
        {
            nluart_putchar(uart_id, *ch, 0);
        }
    }

exit_no_unlock:
    return iFetched;
}

/*******************************************************************************
 * !!!!----               CHARACTER SENSE (USER FACING)              ----!!!!
 ******************************************************************************/
/**
 * @fn bool nluart_canget(const nluart_id_t uart_id)
 * @details Return whether or not a character is available to collect, good for
 * a preface to getchar and friends.
 *----------------------------------------------------------------------------*/
bool nluart_canget(const nluart_id_t uart_id)
{
#ifdef BUILD_FEATURE_SWO_VIRTUAL_UART
    if (uart_id == SWO_VIRTUAL_UART_ID) {
        return swo_vuart_canget();
    }
#endif
    bool retval;
    NL_UART_MUTEX_LOCK(uart_id);

#ifdef NL_UART_IS_ASYNC
    nluart_t *uart = &s_uart_controller[uart_id];
    retval = !uart_rxRing_isEmpty(uart->rx_in, uart->rx_out);
#else
    volatile NRF_UARTE_Type *ctrl = s_uart_config[uart_id].uart_controller;

    // check if we transferred any data in the last attempt.
    // we clear this in getchar()
    retval = (0 != ctrl->EVENTS_ENDRX);
#endif

    NL_UART_MUTEX_UNLOCK(uart_id);
    return retval;
}

/**
 * @fn bool nluart_canput(const nluart_id_t uart_id)
 * @details Return whether or not a character can be output without being
 * blocked, good for a preface to putchar and friends.
 *----------------------------------------------------------------------------*/
bool nluart_canput(const nluart_id_t uart_id)
{
    bool retval;
#ifdef BUILD_FEATURE_SWO_VIRTUAL_UART
    if (uart_id == SWO_VIRTUAL_UART_ID) {
        return swo_vuart_canput();
    }
#endif
    volatile NRF_UARTE_Type *ctrl = s_uart_config[uart_id].uart_controller;
    NL_UART_MUTEX_LOCK(uart_id);

    // we can transmit if TX is stopped
    retval = (ctrl->EVENTS_TXSTOPPED ? true : false);

    NL_UART_MUTEX_UNLOCK(uart_id);
    return retval;
}

/*******************************************************************************
 * !!!!----      MULTIPLE CHARACTER FETCH AND SEND (USER FACING)     ----!!!!
 ******************************************************************************/
/**
 * @fn size_t nluart_putchars(const nluart_id_t uart_id, const uint8_t *data, size_t len, unsigned timeout_ms)
 * @details timeout_ms is currently not supported
 *----------------------------------------------------------------------------*/
size_t nluart_putchars(const nluart_id_t uart_id, const uint8_t *data, size_t len, unsigned timeout_ms)
{
#ifdef BUILD_FEATURE_SWO_VIRTUAL_UART
    if (uart_id == SWO_VIRTUAL_UART_ID) {
        return swo_vuart_putchars(data, len, 0);
    }
#endif
    volatile NRF_UARTE_Type *ctrl = s_uart_config[uart_id].uart_controller;
    size_t sent = 0;

    if (ctrl->ENABLE == 0)
    {
        // not enabled
        return -1;
    }

    while (sent < len)
    {
        nluart_putchar(uart_id, data[sent++], 0);
    }

    return sent;
}

/**
 * @fn size_t nluart_getchars(const nluart_id_t uart_id, uint8_t *data, size_t max_len, unsigned timeout_ms)
 * @details Capture multiple incoming characters, storing them into the
 * destination array "data." Termination conditions include exhausting all
 * immediately available characters (in hardware or software buffers) or
 * capturing a total of max_len characters, whichever occurs first.
 *  If no characters are immediately available at the start of execution, the
 * routine will wait for at least one character to arrive. For async targets
 * the duration of this wait is adjustable by setting the third (defaulted)
 * argument to a nonzero count in milliseconds.
 *  Returns the number of characters received.
 *----------------------------------------------------------------------------*/
size_t nluart_getchars(const nluart_id_t uart_id, uint8_t *data, size_t max_len, unsigned timeout_ms)
{
    size_t received = 0;

    if (max_len == 0)
    {
        goto done;
    }

#if defined(NL_UART_IS_ASYNC) || defined(BUILD_FEATURE_SWO_VIRTUAL_UART)
    do
    {
        if (nluart_getchar(uart_id, &data[received], timeout_ms) > 0)
        {
            received++;
        }
        else
        {
            break;
        }
    }
    while ((received < max_len) && nluart_canget(uart_id));
#else /* !defined(NL_UART_IS_ASYNC) && !defined(BUILD_FEATURE_SWO_VIRTUAL_UART) */
    volatile NRF_UARTE_Type *ctrl = s_uart_config[uart_id].uart_controller;

    do
    {
        nluart_getchar(uart_id, &data[received++], 0);
    }
    while ((received < max_len) && (ctrl->EVENTS_ENDRX));
#endif /* defined(NL_UART_IS_ASYNC) || defined(BUILD_FEATURE_SWO_VIRTUAL_UART) */

done:
    return received;
}

#ifdef NL_UART_IS_ASYNC
/*******************************************************************************
 * !!!!----               HARDWARE SUSPEND AND RESUME                ----!!!!
 ******************************************************************************/

/**
 * @fn void suspend_uart_controller(const nluart_id_t uart_id)
 * @details Disables the UART controller to save power.
 *----------------------------------------------------------------------------*/
static void suspend_uart_controller(const nluart_id_t uart_id)
{
    nluart_t *uart = &s_uart_controller[uart_id];

    if (uart->baud_cache)
    {
        volatile NRF_UARTE_Type *ctrl = s_uart_config[uart_id].uart_controller;

        flush_tx(ctrl);
        config_controller(uart_id, false, true);
        uart->suspended = true;
    }
}

/**
 * @fn void resume_uart_controller(nluart_t *uart)
 * @details Resumes the UART from a SUSPENDED state.
 *----------------------------------------------------------------------------*/
static void resume_uart_controller(const nluart_id_t uart_id)
{
    nluart_t *uart = &s_uart_controller[uart_id];
    if (uart->baud_cache)
    {
        config_controller(uart_id, true, true);
        uart->suspended = false;
    }
}
#endif /* defined(NL_UART_IS_ASYNC) */

/*******************************************************************************
 * !!!!----                   MAINTENANCE GIBBUMS                    ----!!!!
 ******************************************************************************/

/**
 * @fn bool nluart_flush(const nluart_id_t uart_id, unsigned timeout_ms)
 * @details Flushes all pending outgoing transfers and releases all buffered
 * received characters, returning the UART to an idle state. Optional timeout
 * in milliseconds may be supplied for asynchronous modes to wait for their
 * outgoing data to be released (helpful for flow controlled connections).
 *  Returns whether or not the flush was completed properly (only an issue for
 * asynchronous mode with flow control enabled).
 *----------------------------------------------------------------------------*/
bool nluart_flush(const nluart_id_t uart_id, unsigned timeout_ms)
{
#ifdef BUILD_FEATURE_SWO_VIRTUAL_UART
    if (uart_id == SWO_VIRTUAL_UART_ID) {
        return swo_vuart_flush(timeout_ms);
    }
#endif
    const uart_config_t *uart_config = &s_uart_config[uart_id];
    volatile NRF_UARTE_Type *ctrl = uart_config->uart_controller;

    NL_UART_MUTEX_LOCK(uart_id);

#ifdef NL_UART_IS_ASYNC
    nluart_t *uart = &s_uart_controller[uart_id];
    // If TX active, wait for it to finish
    if (!uart->force_sync)
    {
        flush_tx(ctrl);
    }
#endif
    // Flush RX
    ctrl->TASKS_STOPRX = 1;
    ctrl->TASKS_FLUSHRX = 1;
    if (ctrl->EVENTS_RXTO)
    {
        while (!ctrl->EVENTS_RXTO);
        ctrl->EVENTS_RXTO = 0;
    }
    ctrl->EVENTS_ENDRX = 0;
    // Start RX again
    ctrl->TASKS_STARTRX = 1;

#ifdef NL_UART_IS_ASYNC
    // reset receive buffer
    uart->rx_in = uart->rx_out;
#endif

    NL_UART_MUTEX_UNLOCK(uart_id);
    return true;
}

/**
 * @fn void Uart:forceSync(void)
 * @details Transitions the asynchronus variety of the UART driver into a
 * synchronous state, preflushing any outstanding transfers beforehand.
 *  This is intended to allow the transfer of helpful diagnostic information
 * in a fault handler or similar dead-end interruptless scenario.
 *----------------------------------------------------------------------------*/
#ifdef NL_UART_IS_ASYNC
void nluart_force_sync(const nluart_id_t uart_id)
{
    nluart_t *uart = &s_uart_controller[uart_id];
    // do nothing if uart hasn't been configured
    if (uart->baud_cache)
    {
        volatile NRF_UARTE_Type *ctrl = s_uart_config[uart_id].uart_controller;

        NL_UART_MUTEX_LOCK(uart_id);
        if (uart->suspended)
        {
            // crash happened when UART hadn't driver hadn't been told to
            // resume yet. resume it now so we can dump output to console
            // in fault handler.
            resume_uart_controller(uart_id);
        }

        /* Wait for end of any active TX */
        flush_tx(ctrl);

        uart->force_sync = 1;
        NL_UART_MUTEX_UNLOCK(uart_id);
    }
}

/* Called when FreeRTOS scheduler has decided we can go to sleep.
 * To save the most power, we disable the UARTE controller and
 * use PORT event of the GPIO DETECT signal to do wakeup.
 */
void nluart_suspend(void)
{
    // reverse order of resume
    for (unsigned i = 0; i < NL_NUM_UART_CONTROLLERS; i++)
    {
        suspend_uart_controller(NL_NUM_UART_CONTROLLERS - 1 - i);
    }
#ifdef BUILD_FEATURE_SWO_VIRTUAL_UART
    swo_vuart_suspend();
#endif
}

void nluart_resume(void)
{
#ifdef BUILD_FEATURE_SWO_VIRTUAL_UART
    swo_vuart_resume();
#endif
    for (unsigned i = 0; i < NL_NUM_UART_CONTROLLERS; i++)
    {
        resume_uart_controller(i);
    }
}
#endif /* defined(NL_UART_IS_ASYNC) */

int nluart_is_connected(const nluart_id_t uart_id)
{
#ifdef BUILD_FEATURE_SWO_VIRTUAL_UART
    if (uart_id == SWO_VIRTUAL_UART_ID)
    {
        return -1;
    }
#endif

    int val;
    const uart_config_t *uart_config = &s_uart_config[uart_id];
    volatile NRF_UARTE_Type *ctrl = uart_config->uart_controller;

    (void)ctrl; // to avoid compiler unused variable error in release builds
    assert(ctrl->ENABLE != 0);

    NL_UART_MUTEX_LOCK(uart_id);

    /* Briefly reconfigure the RX pin to have a pull down to test whether there
     * is a transmitter attached driving it high.  Not clear if we need to
     * disable the UART first so no pin config conflict.
     */
    nlgpio_request(uart_config->RXD_pin, GPIOF_IN_PD);

    nlplatform_delay_us(500);

    val = nlgpio_get_value(uart_config->RXD_pin);

    /* Release RX pin as GPIO */
    nlgpio_release(uart_config->RXD_pin);

    NL_UART_MUTEX_UNLOCK(uart_id);

    return val;
}
