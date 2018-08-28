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
#include <stdint.h>

#include <nlcompiler.h>
#include <nlplatform_nrf52x/soc-utils.h>


#define CTZ(n) (0 != (n) ? __builtin_ctz(n) : -1)


void event_handler(void *aRegBase, uint32_t aEvents, const event_handler_t *aHandlers)
{
    uint32_t active = 0;

    // Generate bitmap of triggered events filtered by
    // the aEvents mask and clear the triggered state.
    while (aEvents)
    {
        uint8_t bit = CTZ(aEvents);
        uint32_t msk = 1UL << bit;
        volatile uint32_t *reg = soc_event_reg(aRegBase, bit);

        if (*reg)
        {
            *reg = 0;
            active |= msk;
        }

        aEvents &= ~msk;
    }

    /* Call the provided handlers in the table aHandler. */
    while (active)
    {
        uint8_t bit = CTZ(active);
        uint32_t msk = 1UL << bit;
        event_handler_t handler = aHandlers[bit];

        if (handler)
        {
            (*handler)(bit);
        }

        active &= ~msk;
    }
}
