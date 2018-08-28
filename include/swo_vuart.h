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
 *      This file defines APIs for SWO based virtual uart
 */
#ifndef _SWO_VUART_H_
#define _SWO_VUART_H_

#include <nlplatform.h>
#include <nlplatform/nluart.h>

void swo_vuart_init(void);
int swo_vuart_request(const nluart_config_t *config);
int swo_vuart_release(void);
void swo_vuart_set_rx_callback(nluart_rx_t callback);
void swo_vuart_set_wakeup_callback(nluart_wakeup_t callback);
int swo_vuart_putchar(uint8_t ch, unsigned timeout_ms);
int swo_vuart_getchar(uint8_t *ch, unsigned timeout_ms);
bool swo_vuart_canget(void);
bool swo_vuart_canput(void);
size_t swo_vuart_putchars(const uint8_t *data, size_t len, unsigned timeout_ms);
bool swo_vuart_flush(unsigned timeout_ms);
void swo_vuart_suspend(void);
void swo_vuart_resume(void);

#endif /* _SWO_VUART_H_ */
