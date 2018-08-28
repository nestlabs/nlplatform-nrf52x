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
 *      This file defines an object for accessing NRF52x internal
 *      flash.
 *
 */

#ifndef __NLFLASH_NRF52X_H_INCLUDED__
#define __NLFLASH_NRF52X_H_INCLUDED__

#include <nlplatform/nlflash.h>

// Main flash block defines
#define NRF52X_INTERNAL_FLASH_SIZE               (NL_SOC_FLASH_SIZE)
#define NRF52X_INTERNAL_FLASH_ERASE_SIZE         (4096)
#define NRF52X_INTERNAL_FLASH_START              (NL_SOC_FLASH_BASE_ADDRESS)
#define NRF52X_INTERNAL_FLASH_WRITE_SIZE         (4)

const nlflash_info_t *nlflash_nrf52x_get_info(void);
int nlflash_nrf52x_erase(uint32_t from, size_t len, size_t *retlen, nlloop_callback_fp callback);
int nlflash_nrf52x_read(uint32_t from, size_t len, size_t *retlen, uint8_t *buf, nlloop_callback_fp callback);
int nlflash_nrf52x_write(uint32_t to, size_t len, size_t *retlen, const uint8_t *buf, nlloop_callback_fp callback);

#endif // __NLFLASH_NRF52X_H_INCLUDED__
