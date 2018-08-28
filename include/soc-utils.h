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
 *      This file contains utilities for interacting with the platform-internal
 *      event system.
 */

#ifndef _SOC_UTILS_H_
#define _SOC_UTILS_H_


#define SOC_EVENT_WORD_OFFSET    64


typedef void (*event_handler_t)(uint32_t event);


static inline volatile uint32_t *soc_event_reg(void *base, uint32_t event)
{
    return (volatile uint32_t*)base + SOC_EVENT_WORD_OFFSET + event;
}


void event_handler(void *aRegBase, uint32_t aEvents, const event_handler_t *aHandlers);


#endif /* _SOC_UTILS_H_ */
