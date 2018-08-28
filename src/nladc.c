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
 *      This file implements the nRF52x-specific implementation of the ADC
 *      interface defined by platform/nlplatform.
 */

#include <stdio.h>
#include <errno.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <nlassert.h>
#include <nlmacros.h>
#include <nlplatform.h>

#if defined(BUILD_FEATURE_NL_PROFILE) && !defined(NL_NO_RTOS)
#include <nlplatform/nlprofile.h>
#endif

#include <nlplatform_nrf52x/soc-utils.h>

/* The calibration offset is available in an undocumented register
 * at offset 0x640 of the SAADC controller
 */
typedef struct
{
    NRF_SAADC_Type saadc;             /* 0x0 - 0x634 */
    __I uint32_t RESERVED[2];         /* 0x638, 0x63C */
    __IO int32_t calibration_offset;  /* 0x640 - 6-bit signed magnitude */
} NRF_SAADC_Extended_Type;

/* calibration offset value is "6-bits signed magnitude representation" so range
 * is limited to -32 to 31
 */
#define SAADC_CALIBRATION_OFFSET_Pos (0UL) /*!< Position of calibration offset field */
#define SAADC_CALIBRATION_OFFSET_Msk (0x3F << SAADC_CALIBRATION_OFFSET_Pos) /*!< Bit mask of calibration offset field. */
#define SAADC_CALIBRATION_OFFSET_MIN (-32)
#define SAADC_CALIBRATION_OFFSET_MAX (31)

#define NRF_SAADC_EXTENDED ((NRF_SAADC_Extended_Type *)NRF_SAADC_BASE)

static int16_t get_calibration_offset(void)
{
    uint32_t raw_offset = NRF_SAADC_EXTENDED->calibration_offset & SAADC_CALIBRATION_OFFSET_Msk;
    // sign extend the value when returning int16_t value
    return (raw_offset & 0x20) ? (int16_t)(raw_offset | ~SAADC_CALIBRATION_OFFSET_Msk) : (int16_t)raw_offset;
}

#define BITS_TO_RES(b) (((b) - 8) / 2)

#define ADC_RESOLUTION(bits) \
    ((BITS_TO_RES(bits) << SAADC_RESOLUTION_VAL_Pos) & SAADC_RESOLUTION_VAL_Msk)

nlSTATIC_ASSERT(BITS_TO_RES(ADC_SAMPLE_SIZE_BITS) >= SAADC_RESOLUTION_VAL_8bit);
nlSTATIC_ASSERT(BITS_TO_RES(ADC_SAMPLE_SIZE_BITS) <= SAADC_RESOLUTION_VAL_14bit);

typedef struct
{
    StaticSemaphore_t   mMutex;
    TaskHandle_t        mWaitingTaskHandle;
    adc_cb_t            mCallback;
    void               *mContext;
} context_t;

static void event_handler_end(void);
static void event_handler_done(void);
static void event_handler_calibratedone(void);

static void mutex_acquire(void);
static void mutex_release(void);

static void notify_take(void);
static void notify_give(void *unused);

static void saadc_enable(bool enable);
static void event_irq_enable(uint32_t event, bool enable);
static void buffer_setup(void *aBuffer, size_t aSamples);
static void channel_setup(size_t aIndex, const nladc_channel_t *aChannel);

static int do_read(const nladc_config_t *aConfig,
                   void *aBuffer, size_t aSamples,
                   adc_cb_t aCallback, void *aContext);


static context_t sContext;

void event_handler_end(void)
{
    adc_cb_t callback = sContext.mCallback;
    void * context = sContext.mContext;

    event_irq_enable(SAADC_INTEN_END_Pos, false);
    event_irq_enable(SAADC_INTEN_DONE_Pos, false);
    saadc_enable(false);

    callback(context);
}

void event_handler_done(void)
{
    if (NRF_SAADC->ENABLE)
    {
        NRF_SAADC->TASKS_SAMPLE = true;
    }
}

void event_handler_calibratedone(void)
{
    adc_cb_t callback = sContext.mCallback;
    void * context = sContext.mContext;

    event_irq_enable(SAADC_INTEN_CALIBRATEDONE_Pos, false);
    saadc_enable(false);

    callback(context);
}

void mutex_acquire(void)
{
    xSemaphoreTake(&sContext.mMutex, portMAX_DELAY);
}

void mutex_release(void)
{
    xSemaphoreGive(&sContext.mMutex);
}

void notify_take(void)
{
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

void notify_give(void *unused)
{
    BaseType_t yield = pdFALSE;

    NL_UNUSED(unused);

    vTaskNotifyGiveFromISR(sContext.mWaitingTaskHandle, &yield);
    sContext.mWaitingTaskHandle = NULL;
    portEND_SWITCHING_ISR(yield);
}

void saadc_enable(bool enable)
{
    uint32_t value;

    if (enable)
    {
        value = SAADC_ENABLE_ENABLE_Enabled;
    }
    else
    {
        size_t i = 0;
        /* Nordic reference driver waits for EVENT_STOPPED with timeout.
         * Not sure why timeout is need but follow their example.
         */
        uint32_t max_event_stop_tries = 10000;

        /* Must do TASKS_STOP or else power consumption is high */
        NRF_SAADC->EVENTS_STOPPED = 0;
        NRF_SAADC->TASKS_STOP = 1;
        while ((NRF_SAADC->EVENTS_STOPPED == 0) && (max_event_stop_tries > 0))
        {
            max_event_stop_tries--;
        }

        /* Set all channels to not-connected. In practice, leaving a gpio
         * configured on an ADC channel has interfered with other uses of the
         * gpio, for example, as an irq source. */
        while (i < NLADC_MAX_CHANNEL)
        {
            channel_setup(i, NULL);
            i++;
        }

        sContext.mCallback = NULL;
        sContext.mContext = NULL;

        value = SAADC_ENABLE_ENABLE_Disabled;
    }

    NRF_SAADC->ENABLE = (value << SAADC_ENABLE_ENABLE_Pos);

#if defined(BUILD_FEATURE_NL_PROFILE) && !defined(NL_NO_RTOS)
    if (enable)
    {
        nl_profile_start(NL_PROFILE_ADC_ENABLED);
    }
    else
    {
        nl_profile_stop(NL_PROFILE_ADC_ENABLED);
    }
#endif
}

void event_irq_enable(uint32_t event, bool enable)
{
    uint32_t mask = 1 << event;

    if (enable)
    {
        *soc_event_reg(NRF_SAADC, event) = 0;
        NRF_SAADC->INTENSET = mask;
    }
    else
    {
        NRF_SAADC->INTENCLR = mask;
    }
}

void buffer_setup(void *aBuffer, size_t aSamples)
{
    NRF_SAADC->RESULT.PTR = (uint32_t)aBuffer;
    NRF_SAADC->RESULT.MAXCNT = aSamples;
}

void channel_setup(size_t aIndex, const nladc_channel_t *aChannel)
{
    int pos = aChannel != NULL ? aChannel->mPos : NLADC_INPUT_NONE;
    int neg = aChannel != NULL ? aChannel->mNeg : NLADC_INPUT_NONE;
    uint32_t cfg = aChannel != NULL ? aChannel->mConfig : 0;

    nlASSERT(NLADC_INPUT_VALID(pos));
    nlASSERT(NLADC_INPUT_VALID(neg));

    NRF_SAADC->CH[aIndex].PSELP = pos;
    NRF_SAADC->CH[aIndex].PSELN = neg;
    NRF_SAADC->CH[aIndex].CONFIG = cfg;
}

int do_read(const nladc_config_t *aConfig,
            void *aBuffer, size_t aSamples,
            adc_cb_t aCallback, void *aContext)
{
    int ret = 0;
    size_t i;

    nlASSERT(aConfig->mCount > 0);
    nlASSERT(aConfig->mCount <= aSamples);

    if (sContext.mCallback != NULL)
    {
        ret = -EBUSY;
        goto done;
    }

    sContext.mCallback = aCallback;
    sContext.mContext = aContext;

    buffer_setup(aBuffer, aSamples);

    i = 0;
    while (i < aConfig->mCount)
    {
        channel_setup(i, &aConfig->mChannels[i]);
        i++;
    }

    NRF_SAADC->RESOLUTION = ADC_RESOLUTION(ADC_SAMPLE_SIZE_BITS);

    saadc_enable(true);
    event_irq_enable(SAADC_INTEN_END_Pos, true);
    event_irq_enable(SAADC_INTEN_DONE_Pos, true);

    NRF_SAADC->TASKS_START = 1;
    NRF_SAADC->TASKS_SAMPLE = 1;

done:

    return ret;
}

void nladc_isr(void);
void nladc_isr(void)
{
    uint32_t inten = NRF_SAADC->INTEN;

    if ((inten & SAADC_INTEN_END_Msk) && NRF_SAADC->EVENTS_END)
    {
        NRF_SAADC->EVENTS_END = 0;
        event_handler_end();
    }

    if ((inten & SAADC_INTEN_DONE_Msk) && NRF_SAADC->EVENTS_DONE)
    {
        NRF_SAADC->EVENTS_DONE = 0;
        event_handler_done();
    }

    if ((inten & SAADC_INTEN_CALIBRATEDONE_Msk) && NRF_SAADC->EVENTS_CALIBRATEDONE)
    {
        NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
        event_handler_calibratedone();
    }
}

void nladc_init(void)
{
    xSemaphoreCreateMutexStatic(&sContext.mMutex);

    nlplatform_irq_clear_pending(SAADC_IRQn);
    nlplatform_irq_enable(SAADC_IRQn);
}

#if defined(BUILD_CONFIG_DIAGNOSTICS)
void nladc_apply_calibration(uint16_t gain, int16_t offset)
{
    nlASSERT(gain == 0);
    nlASSERT((offset >= SAADC_CALIBRATION_OFFSET_MIN) && (offset <= SAADC_CALIBRATION_OFFSET_MAX));

    NRF_SAADC_EXTENDED->calibration_offset = SAADC_CALIBRATION_OFFSET_Msk & offset;
}

void nladc_print_calibration(void)
{
    uint32_t offset = NRF_SAADC_EXTENDED->calibration_offset & SAADC_CALIBRATION_OFFSET_Msk;
    /* Print raw value in hex and the value in decimal, sign extending if needed */
    printf("Offset: 0x%02x (%i)\n", offset, (offset & 0x20) ? (int)(offset | ~SAADC_CALIBRATION_OFFSET_Msk) : offset);
}

void nladc_reset_calibration(void)
{
    /* Not Supported */
}
#endif /* defined(BUILD_CONFIG_DIAGNOSTICS) */

int nladc_calibrate(void)
{
    int ret = 0;

    mutex_acquire();

    if (sContext.mCallback != NULL)
    {
        ret = -EBUSY;
        goto done;
    }

    sContext.mCallback = notify_give;
    sContext.mWaitingTaskHandle = xTaskGetCurrentTaskHandle();

    saadc_enable(true);
    event_irq_enable(SAADC_INTEN_CALIBRATEDONE_Pos, true);

    NRF_SAADC->TASKS_CALIBRATEOFFSET = true;

    notify_take();

done:

    mutex_release();

    return ret;
}

int nladc_get_calibration(nladc_calibration_t *cal)
{
    nlASSERT(cal);
    cal->gain = 0;
    // sign extend the value when returning int16_t value
    cal->offset = get_calibration_offset();
    return 0;
}

int nladc_read_async(const nladc_config_t *cfg, void *buf, size_t samples, adc_cb_t cb, void *context)
{
    int ret;

    mutex_acquire();

    ret = do_read(cfg, buf, samples, cb, context);

    mutex_release();

    return ret;
}

int nladc_read(const nladc_config_t *cfg, void* buf, size_t samples)
{
    int ret;

    mutex_acquire();

    sContext.mWaitingTaskHandle = xTaskGetCurrentTaskHandle();
    ret = do_read(cfg, buf, samples, notify_give, NULL);
    if (ret >= 0)
    {
        notify_take();
    }

    mutex_release();

    return ret;
}
