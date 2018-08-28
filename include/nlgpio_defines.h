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
 *      This file contains SOC-specific definitions used in the NLGPIO API
 *
 */

#ifndef _NLGPIO_DEFINES_H_
#define _NLGPIO_DEFINES_H_

// Encoding representing feature set of GPIO controller

// The NRF52x uses two registers for configuring and setting output level.
// Output level can be set with a single bit write into either the OUTSET
// or OUTCLR register.
// Configuration register is used to configure direction, input buffer
// enable/disable, pull enable/disable, drive strength, and pin sensing.

// Flag Bits 0-17 - PIN_CFG register values
#define INTERNAL_PIN_CFG_DIR_IN                  (0x0 << 0)
#define INTERNAL_PIN_CFG_DIR_OUT                 (0x1 << 0)
#define INTERNAL_PIN_CFG_INPUT_BUFFER_CONNECT    (0x0 << 1)
#define INTERNAL_PIN_CFG_INPUT_BUFFER_DISCONNECT (0x1 << 1)
#define INTERNAL_PIN_CFG_PULL_DISABLED           (0x0 << 2)
#define INTERNAL_PIN_CFG_PULL_DOWN               (0x1 << 2)
#define INTERNAL_PIN_CFG_PULL_UP                 (0x3 << 2)
#define INTERNAL_PIN_CFG_DRIVE_S0S1              (0x0 << 8)
#define INTERNAL_PIN_CFG_DRIVE_H0S1              (0x1 << 8)
#define INTERNAL_PIN_CFG_DRIVE_S0H1              (0x2 << 8)
#define INTERNAL_PIN_CFG_DRIVE_H0H1              (0x3 << 8)
#define INTERNAL_PIN_CFG_DRIVE_D0S1              (0x4 << 8)
#define INTERNAL_PIN_CFG_DRIVE_D0H1              (0x5 << 8)
#define INTERNAL_PIN_CFG_DRIVE_S0D1              (0x6 << 8)
#define INTERNAL_PIN_CFG_DRIVE_H0D1              (0x7 << 8)
#define INTERNAL_PIN_CFG_DRIVE_SENSE_DISABLED    (0x0 << 16)
#define INTERNAL_PIN_CFG_DRIVE_SENSE_HIGH        (0x2 << 16)
#define INTERNAL_PIN_CFG_DRIVE_SENSE_LOW         (0x3 << 16)

// Flag Bit 31 - output level if used with an output mode,
#define INTERNAL_OUT_LOW   (0 << 31)
#define INTERNAL_OUT_HIGH  (1 << 31)

typedef uint32_t nlgpio_flags_t;

// Supported pin configurations.  Use these as arguments to the nlgpio_request() function.
#define GPIOF_IN               (INTERNAL_PIN_CFG_DIR_IN | INTERNAL_PIN_CFG_INPUT_BUFFER_CONNECT)
#define GPIOF_IN_PU            (GPIOF_IN | INTERNAL_PIN_CFG_PULL_UP)
#define GPIOF_IN_PD            (GPIOF_IN | INTERNAL_PIN_CFG_PULL_DOWN)
#define GPIOF_OUT_LOW          (INTERNAL_PIN_CFG_DIR_OUT | INTERNAL_OUT_LOW)
#define GPIOF_OUT_HIGH         (INTERNAL_PIN_CFG_DIR_OUT | INTERNAL_OUT_HIGH)
// Open drain is equivalent to S0D1.  When we request a 0, we want it to go to zero,
//  but when we request a 1, we disconnect and allow an external pull up or another
//  chip to control the line (like for shared SPI or I2C case).
#define GPIOF_OUT_OD_LOW       (INTERNAL_PIN_CFG_DIR_OUT | INTERNAL_PIN_CFG_DRIVE_S0D1 | INTERNAL_OUT_LOW)
#define GPIOF_OUT_OD_HIGH      (INTERNAL_PIN_CFG_DIR_OUT | INTERNAL_PIN_CFG_DRIVE_S0D1 | INTERNAL_OUT_HIGH)
// Analog function is controlled by the peripheral, not by the GPIO block.
#define GPIOF_ANALOG           (INTERNAL_PIN_CFG_INPUT_BUFFER_DISCONNECT)
// Do not pass this into GPIO request function, it is used only in initial pad configuration
#define GPIOF_NO_CONFIG        (INTERNAL_PIN_CFG_INPUT_BUFFER_DISCONNECT)

// Compatibility wrapper for coexistence with K60
#define GPIOF_IN_INIT          (GPIOF_IN)
#define GPIOF_OUT_INIT_LOW     (GPIOF_OUT_LOW)
#define GPIOF_OUT_INIT_HIGH    (GPIOF_OUT_HIGH)

// Interrupt flags.  Want this to fit into uint8_t
// because of nlgpio_button.h, so will have to shift when used.

#define IRQF_TRIGGER_NONE      (0)
#define IRQF_TRIGGER_RISING    (1) /* interrupt when rising edge seen */
#define IRQF_TRIGGER_FALLING   (2) /* interrupt when falling edge seen */
#define IRQF_TRIGGER_BOTH      (3) /* interrupt when either edge seen */
#define IRQF_TRIGGER_HIGH      (4) /* interrupt when GPIO pin level is high */
#define IRQF_TRIGGER_LOW       (5) /* interrupt when GPIO pin level is low */

#endif /* _NLGPIO_DEFINES_H_ */
