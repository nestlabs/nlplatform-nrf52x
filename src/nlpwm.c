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
#include <nlproduct_config.h>
#if NL_NUM_PWM_CONTROLLERS > 0


#include <errno.h>

#include <FreeRTOS.h>
#include <semphr.h>

#include <nrf52x/device/nrf52840.h>
#include <nrf52x/device/nrf52840_bitfields.h>

#include <nlassert.h>
#include <nlmacros.h>
#include <nlplatform/nlgpio.h>
#include <nlplatform/nlpwm.h>


#define FREQ_BASE       NL_PLATFORM_PCLK_16M
#define DUTY_MAX        NL_PWM_DUTY_MAX

#define COUNTERTOP_MIN  3
#define COUNTERTOP_MAX  32767

#define PRESCALER_MIN   1
#define PRESCALER_MAX   128

#define CMP_TO_SEQ(c)   (((c) & 0x3fff) | (1 << 15))

#define CEIL_DIV(n, d) (((n) + (d) - 1) / (d))


typedef struct
{
    NRF_PWM_Type       *mRegisters;
    nlgpio_id_t         mGpio;
} info_t;


static const info_t sInfo[] =
{
#if NL_NUM_PWM_CONTROLLERS > 0
    {
        .mRegisters = PWM0_CONTROLLER,
        .mGpio      = PWM0_GPIO,
    },
#endif
#if NL_NUM_PWM_CONTROLLERS > 1
    {
        .mRegisters = PWM1_CONTROLLER,
        .mGpio      = PWM1_GPIO,
    },
#endif
#if NL_NUM_PWM_CONTROLLERS > 2
    {
        .mRegisters = PWM2_CONTROLLER,
        .mGpio      = PWM2_GPIO,
    },
#endif
#if NL_NUM_PWM_CONTROLLERS > 3
    {
        .mRegisters = PWM3_CONTROLLER,
        .mGpio      = PWM3_GPIO,
    },
#endif
};
nlSTATIC_ASSERT(ARRAY_SIZE(sInfo) == NL_NUM_PWM_CONTROLLERS);

static StaticSemaphore_t sMutex[NL_NUM_PWM_CONTROLLERS];
static uint16_t sSequence[NL_NUM_PWM_CONTROLLERS];


void nlpwm_init(void)
{
    size_t id;

    for (id = 0 ; id < NL_NUM_PWM_CONTROLLERS ; id++)
    {
        xSemaphoreCreateMutexStatic(&sMutex[id]);
    }
}

int nlpwm_request(nlpwm_id_t aId, const nlpwm_config_t *aConfig)
{
    const info_t *info;
    NRF_PWM_Type *regs;
    SemaphoreHandle_t mutex;
    uint16_t *sequence;
    uint8_t prescaler;
    uint16_t countertop;
    uint16_t compare;
    int ret = 0;

    nlASSERT(aId < NL_NUM_PWM_CONTROLLERS);
    nlASSERT(aConfig != NULL);

    info = &sInfo[aId];
    regs = info->mRegisters;
    mutex = &sMutex[aId];
    sequence = &sSequence[aId];

    xSemaphoreTake(mutex, portMAX_DELAY);

    if (((regs->PSEL.OUT[0] & PWM_PSEL_OUT_CONNECT_Msk) >> PWM_PSEL_OUT_CONNECT_Pos) == PWM_PSEL_OUT_CONNECT_Connected)
    {
        ret = -EBUSY;
        goto done;
    }

    if (aConfig->mFreq > FREQ_BASE / (COUNTERTOP_MAX + 1))
    {
        prescaler = PRESCALER_MIN;
    }
    else
    {
        prescaler = CEIL_DIV(FREQ_BASE, aConfig->mFreq * (COUNTERTOP_MAX + 1));
        prescaler = 1 << (LOG2(prescaler - 1) + 1);
    }

    countertop = FREQ_BASE / (aConfig->mFreq * prescaler);
    compare = aConfig->mDuty * countertop / (DUTY_MAX + 1);

    if ((prescaler < PRESCALER_MIN) ||
        (prescaler > PRESCALER_MAX) ||
        (countertop < COUNTERTOP_MIN) ||
        (countertop > COUNTERTOP_MAX) ||
        (compare > countertop))
    {
        ret = -EINVAL;
        goto done;
    }

    if (nlgpio_set_output(info->mGpio, 0) < 0)
    {
        ret = -EIO;
        goto done;
    }

    regs->PSEL.OUT[0] = (info->mGpio << PWM_PSEL_OUT_PIN_Pos) |
                        ((PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos) & PWM_PSEL_OUT_CONNECT_Msk);

    regs->ENABLE = PWM_ENABLE_ENABLE_Enabled;

    regs->MODE = PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos;
    regs->COUNTERTOP = countertop;
    regs->PRESCALER = LOG2(prescaler);

    regs->DECODER = PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos |
                    PWM_DECODER_LOAD_Common << PWM_DECODER_LOAD_Pos;

    regs->SHORTS = PWM_SHORTS_LOOPSDONE_SEQSTART0_Enabled << PWM_SHORTS_LOOPSDONE_SEQSTART0_Pos;

    *sequence = CMP_TO_SEQ(compare);
    regs->SEQ[0].PTR = (uintptr_t)sequence;
    regs->SEQ[0].CNT = 1;
    regs->SEQ[0].REFRESH = 0;
    regs->SEQ[0].ENDDELAY = 0;

    regs->TASKS_SEQSTART[0] = 1;

done:

    xSemaphoreGive(mutex);

    return ret;
}

int nlpwm_release(nlpwm_id_t aId)
{
    const info_t *info;
    NRF_PWM_Type *regs;
    SemaphoreHandle_t mutex;
    int ret = 0;

    nlASSERT(aId < NL_NUM_PWM_CONTROLLERS);

    info = &sInfo[aId];
    regs = info->mRegisters;
    mutex = &sMutex[aId];

    xSemaphoreTake(mutex, portMAX_DELAY);

    if (regs->ENABLE == 0)
    {
        ret = -EINVAL;
        goto done;
    }

    regs->TASKS_STOP = 1;
    regs->ENABLE = 0;
    regs->PSEL.OUT[0] = PWM_PSEL_OUT_CONNECT_Disconnected << PWM_PSEL_OUT_CONNECT_Pos;

    nlgpio_release(info->mGpio);

done:

    xSemaphoreGive(mutex);

    return ret;
}


#else /* NL_NUM_PWM_CONTROLLERS > 0 */


void nlpwm_init(void);
void nlpwm_init(void)
{
    /* Do Nothing */
}


#endif /* NL_NUM_PWM_CONTROLLERS > 0 */
