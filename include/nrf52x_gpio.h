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
 *
 *      This file provides Nordic-specific GPIO information. This includes:
 *
 *      - Symbols for referencing GPIO according to port and pin number.
 *      - The number of GPIOs available.
 *      - The number of GPIO ports available.
 */

#ifndef _NRF52X_GPIO_H_
#define _NRF52X_GPIO_H_

#include <nlgpio_defines.h>

typedef enum {
    NRF52X_P0_0,  /* 0 */
    NRF52X_P0_1,  /* 1 */
    NRF52X_P0_2,  /* 2 */
    NRF52X_P0_3,  /* 3 */
    NRF52X_P0_4,  /* 4 */
    NRF52X_P0_5,  /* 5 */
    NRF52X_P0_6,  /* 6 */
    NRF52X_P0_7,  /* 7 */
    NRF52X_P0_8,  /* 8 */
    NRF52X_P0_9,  /* 9 */
    NRF52X_P0_10, /* 10 */
    NRF52X_P0_11, /* 11 */
    NRF52X_P0_12, /* 12 */
    NRF52X_P0_13, /* 13 */
    NRF52X_P0_14, /* 14 */
    NRF52X_P0_15, /* 15 */
    NRF52X_P0_16, /* 16 */
    NRF52X_P0_17, /* 17 */
    NRF52X_P0_18, /* 18 */
    NRF52X_P0_19, /* 19 */
    NRF52X_P0_20, /* 20 */
    NRF52X_P0_21, /* 21 */
    NRF52X_P0_22, /* 22 */
    NRF52X_P0_23, /* 23 */
    NRF52X_P0_24, /* 24 */
    NRF52X_P0_25, /* 25 */
    NRF52X_P0_26, /* 26 */
    NRF52X_P0_27, /* 27 */
    NRF52X_P0_28, /* 28 */
    NRF52X_P0_29, /* 29 */
    NRF52X_P0_30, /* 30 */
    NRF52X_P0_31, /* 31 */

    NRF52X_P1_0,  /* 32 */
    NRF52X_P1_1,  /* 33 */
    NRF52X_P1_2,  /* 34 */
    NRF52X_P1_3,  /* 35 */
    NRF52X_P1_4,  /* 36 */
    NRF52X_P1_5,  /* 37 */
    NRF52X_P1_6,  /* 38 */
    NRF52X_P1_7,  /* 39 */
    NRF52X_P1_8,  /* 40 */
    NRF52X_P1_9,  /* 41 */
    NRF52X_P1_10, /* 42 */
    NRF52X_P1_11, /* 43 */
    NRF52X_P1_12, /* 44 */
    NRF52X_P1_13, /* 45 */
    NRF52X_P1_14, /* 46 */
    NRF52X_P1_15, /* 47 */

    NRF52X_INVALID_GPIO_ID = 0x7f
} nlgpio_id_t;

#define NL_NUM_GPIO	(NRF52X_P1_15 + 1)
#define NL_NUM_PORTS   2

// Array that stores the low power gpio configuration
extern const nlgpio_flags_t gpio_low_power_config[];

#endif /* _NRF52X_GPIO_H_ */
