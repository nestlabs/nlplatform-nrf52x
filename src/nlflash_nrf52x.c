/*
 *
 *    Copyright (c) 2014-2018 Nest Labs, Inc.
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
 *      This file implements an object for accessing NRF52x internal
 *      flash.
 *
 */


#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include <nlplatform_nrf52x/nlflash_nrf52x.h>

#include <nrf52x/device/nrf52840.h>
#include <nrf52x/device/nrf52840_bitfields.h>

static const nlflash_info_t s_flash_info =
{
    .name = "NRF52X",
    .base_addr = NRF52X_INTERNAL_FLASH_START,
    .size = NRF52X_INTERNAL_FLASH_SIZE,
    .erase_size = NRF52X_INTERNAL_FLASH_ERASE_SIZE,
    .fast_erase_size = 0,
    .write_size = NRF52X_INTERNAL_FLASH_WRITE_SIZE
};

const nlflash_info_t *nlflash_nrf52x_get_info(void)
{
    return &s_flash_info;
}

int nlflash_nrf52x_erase(uint32_t from, size_t len, size_t *retlen, nlloop_callback_fp callback)
{
    int retval = -1;
    uint32_t eraseSize = s_flash_info.erase_size;

    *retlen = 0;

    if ( from & (eraseSize - 1) ) {
        // Start not properly aligned on a eraseSize boundary.
        goto done;
    }

    // Enable write
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
    while ( len ) {
        NRF_NVMC->ERASEPAGE = from;
        *retlen += eraseSize;
        if ( callback != NULL ) {
            callback();
        }
        from += eraseSize;
        if ( len > eraseSize) {
            len -= eraseSize;
        } else {
            len = 0;
        }
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
    }
    // Disable write, set FLASH to read only
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
    retval = 0;
done:
    return retval;
}


int nlflash_nrf52x_read(uint32_t from, size_t len, size_t *retlen, uint8_t *buf, nlloop_callback_fp callback)
{
    // internal flash is memory mapped so just memcpy.  we currently
    // don't expect 'from' to be relative to base_addr but an absolute address
    memcpy(buf, (uint8_t*)from, len);
    *retlen = len;
    return 0;
}

int nlflash_nrf52x_write(uint32_t to, size_t len, size_t *retlen,
                        const uint8_t *buf, nlloop_callback_fp callback)
{
    int retval = -1;
    uint32_t *src = (uint32_t*)buf;
    *retlen = 0;

    if ( len & (s_flash_info.write_size - 1) )
    {
        // Flash can only write full words
        goto done;
    }
    if ( to & (s_flash_info.write_size - 1) )
    {
        // Flash can only write on word boundaries
        goto done;
    }
    *retlen = len;
    // Enable write
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
    while ( len ) {
        *(uint32_t*)to = *src;
        to += sizeof(uint32_t);
        src++;
        if ( callback != NULL ) {
            callback();
        }
        len -= sizeof(uint32_t);
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
    }
    // Disable write, set FLASH to read only
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
    retval = 0;
done:
    return retval;
}
