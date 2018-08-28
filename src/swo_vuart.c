/*
 *
 *    Copyright (c) 2016-2018 Nest Labs, Inc.
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
 *      This is the implementation of a virtual uart using the
 *    ARM ITM/TPIU/SWO.  Output is pretty straightforward and
 *    just involves writing to the ITM, either directly using
 *    ITM_SendChar() or via using Silab's emDebugSendVuartMessage()
 *    wrapper, which adds additional overhead and protocol.
 *      Input is a bit more custom because SWO is unidirectional
 *    output.  The typical implementation involves a debugger
 *    connected over SWD that writes to a global rxBuffer, and
 *    then triggering an interrupt (Debug Interrupt in EM358x)
 *    to notify the code to go process that buffer.  Silabs' ISA3
 *    debugger uses one method specific to their emDebug API,
 *    and we probably want to implement something similar for
 *    the simpler basic ITM method.
 */
#include "../include/swo_vuart.h"

static nluart_wakeup_t s_wakeup_callback;
static nluart_rx_t s_rx_callback;

#define RX_BUFFER_BYTES 16

// circular buffer for received characters and indices for reading/writing
typedef struct nl_circular_buffer_s {
    uint8_t buffer[RX_BUFFER_BYTES];
    uint8_t read_index;
    uint8_t write_index;
} nl_circular_buffer_t;
static nl_circular_buffer_t rxBuffer;

// circular buffer utility functions
static inline bool nl_circular_buffer_can_read(nl_circular_buffer_t *cbuffer)
{
    return (cbuffer->write_index != cbuffer->read_index);
}
static inline int nl_circular_buffer_write(nl_circular_buffer_t *cbuffer, uint8_t ch)
{
    int result;
    uint8_t next_write_index;

    nlplatform_interrupt_disable();
    // check if buffer is full, which is the case if incrementing
    // the write_index would make it equal to the read_index.
    // we can only store buffer_size - 1 worth of data.
    next_write_index = (cbuffer->write_index + 1) % sizeof(cbuffer->buffer);
    if (next_write_index == cbuffer->read_index) {
        // no space left
        result = -1;
    } else {
        cbuffer->buffer[cbuffer->write_index] = ch;
        cbuffer->write_index = next_write_index;
        result = 0;
    }
    nlplatform_interrupt_enable();
    return result;
}
static inline int nl_circular_buffer_read(nl_circular_buffer_t *cbuffer, uint8_t *ch)
{
    int result;
    nlplatform_interrupt_disable();
    if (nl_circular_buffer_can_read(cbuffer)) {
        *ch = cbuffer->buffer[cbuffer->read_index];
        cbuffer->read_index = (cbuffer->read_index + 1) % sizeof(cbuffer->buffer);
        result = 1;
    } else {
        result = 0;
    }
    nlplatform_interrupt_enable();
    return result;
}

static void receive_vuart_message(uint8_t *data, uint8_t length)
{
    uint8_t i;
    for (i = 0; i < length; i++) {
        if (nl_circular_buffer_write(&rxBuffer, data[i]) != 0) {
            break;
        }
    }
    if (s_rx_callback) {
        s_rx_callback(SWO_VIRTUAL_UART_ID);
        s_rx_callback = NULL;
    }
}

#ifdef BUILD_FEATURE_SILABS_VIRTUAL_UART
#include <stack/platform/micro/debug-channel.h>

void swo_vuart_init(void)
{
    emDebugInit();
    // emDebugInit() calls emDebugPowerUp() automatically,
    // so we have to call emDebugPowerDown() to start in
    // off state
    emDebugPowerDown();
}

int swo_vuart_request(const nluart_config_t *config)
{
    // baud rate is fixed in emDebugPowerUp()
    emDebugPowerUp();
    return 0;
}

int swo_vuart_release(void)
{
    emDebugPowerDown();
    s_wakeup_callback = NULL;
    s_rx_callback = NULL;
    return 0;
}


int swo_vuart_putchar(uint8_t ch, unsigned timeout_ms)
{
#if !defined(NL_BOOTLOADER) // do nothing in bootloader for now
    // timeout not supported
    uint8_t txBuffer[8];
    uint8_t txBufferLength;

    // Create our own frame instead of using emDebugSendVuartMessage()
    // because that function does too much and uses too much stack
    // space (it does a vnsprintf() like parsing internally, which
    // is totally overkill for a single character output
#define EM_DEBUG_VIRTUAL_UART_TX 0x0011
    txBufferLength = emDebugAddInitialFraming(txBuffer, EM_DEBUG_VIRTUAL_UART_TX);
    txBuffer[txBufferLength++] = ch;
    emDebugSend(txBuffer, &txBufferLength);
#endif
    return 1;
}

bool swo_vuart_canput(void)
{
    return true;
}

bool swo_vuart_flush(unsigned timeout_ms)
{
    // no support
    return true;
}

void swo_vuart_suspend(void)
{
    emDebugPowerDown();
}

void swo_vuart_resume(void)
{
    emDebugPowerUp();
    // not fully tested.  not sure the wakeup callback is that
    // useful when using Silabs virtual UART since ISA3 has to
    // be connected and deep sleep is blocked in this case.
    if (s_wakeup_callback) {
        WakeEvents wakeEvents = halGetWakeInfo();
        if (wakeEvents.events.internal.bits.WakeInfoValid &&
            wakeEvents.events.internal.bits.WAKE_CORE_B) {
            s_wakeup_callback(SWO_VIRTUAL_UART_ID);
        }
    }
}

// Silabs hook.  This is called by the emDebug subsystem
// whenever some number of characters have been received.
// If we don't have enough room in our rxBuffer, we just
// drop them.
void halStackReceiveVuartMessage(int8u *data, int8u length)
{
    receive_vuart_message(data, length);
}

#else /* BUILD_FEATURE_SILABS_VIRTUAL_UART */

// This is an implementation based on raw ITM, similar to
// what's in CMSIS.
void swo_vuart_init(void)
{
}

int swo_vuart_request(const nluart_config_t *config)
{
    nlplatform_interrupt_disable();
    // configure baud rate
    TPIU_COSD = (halPeripheralClockHz() / config->baud_rate) - 1;
    // configure port size
    TPIU_CPS = 1;
    // disable formatter
    TPIU_FFC = 0;
    // configure for UART protocol
    TPIU_SPP = 2;
    // enable stimulus port 0
    ITM_TER = 1;
    // enable ITM
    ITM_TCR = ((0x01 << ITM_TCR_ATBID_BIT) | ITM_TCR_ITMEN);

    // enable debug interrupt to get notification from debugger
    // of characters sent to EM358
    INT_CFGSET = INT_DEBUG;

    nlplatform_interrupt_enable();
    return 0;
}

int swo_vuart_release(void)
{
    nlplatform_interrupt_disable();

    // disable debug interrupt
    INT_CFGCLR = INT_DEBUG;

    // wait for ITM to be not busy
    while(ITM_TCR & ITM_TCR_BUSY) {}
    // disable ITM
    ITM_TCR &= ~ITM_TCR_ITMEN;
    if (CPWRUPREQ_STATUS) {
        // wait long enough for any packet in the TPIU to get sent out
        // the wait time should really depend on baud rate but
        // to keep things simple, we just wait a fixed time right now
        nlplatform_delay_us(500);
    }

    nlplatform_interrupt_enable();
    return 0;
}

int swo_vuart_putchar(uint8_t ch, unsigned timeout_ms)
{
    // wait until fifo is ready, no timeout support
    nlplatform_interrupt_disable();
    while (ITM_SP0 == 0) {}
    *(volatile uint8_t*)(ITM_SP0_ADDR) = ch;
    nlplatform_interrupt_enable();
    return 1;
}

bool swo_vuart_canput(void)
{
    return (ITM_SP0 == ITM_SP0_FIFOREADY);
}

bool swo_vuart_flush(unsigned timeout_ms)
{
    // wait until fifo is ready, no timeout support.
    // not a complete guarantee because TPIU could
    // still be sending packet out, but we have
    // no TPIU status register to check.
    nlplatform_interrupt_disable();
    while (ITM_SP0 == 0) {}
    nlplatform_interrupt_enable();
    return 0;
}

void swo_vuart_suspend(void)
{
    // nop for now.  not sure if diabling anything would actually
    // save any power
}

void swo_vuart_resume(void)
{
    // nop for now.
}

// openocd/debugger hook.
volatile int32_t ITM_RxBuffer; /* Variable that debugger writes to */

// function that will be called when debugger triggers Debug Interrupt
void halDebugIsrHandler(void);
void halDebugIsrHandler(void)
{
    receive_vuart_message((uint8_t*)&ITM_RxBuffer, 1);
}

#endif

void swo_vuart_set_rx_callback(nluart_rx_t callback)
{
    s_rx_callback = callback;
}

void swo_vuart_set_wakeup_callback(nluart_wakeup_t callback)
{
    s_wakeup_callback = callback;
}

int swo_vuart_getchar(uint8_t *ch, unsigned timeout_ms)
{
    // no timeout support
    return nl_circular_buffer_read(&rxBuffer, ch);
}

bool swo_vuart_canget(void)
{
    return nl_circular_buffer_can_read(&rxBuffer);
}

size_t swo_vuart_putchars(const uint8_t *data, size_t len, unsigned timeout_ms)
{
    size_t i;
    int result;
    for (i = 0; i < len; i++) {
        result = swo_vuart_putchar(data[i], timeout_ms);
        if (result != 0)
            break;
    }
    return i;
}
