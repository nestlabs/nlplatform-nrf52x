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
 *      This file defines APIs specific to nrf52x
 */
#ifndef _NLPLATFORM_SOC_H_
#define _NLPLATFORM_SOC_H_

/* PACKAGE is defined by weave, but used as a struct member name by nrf52840.h, so
 * we get a conflict. To work around it, undef PACKAGE and redefine it later.
 */
#ifdef PACKAGE
#define WEAVE_PACKAGE PACKAGE
#undef PACKAGE
#endif
#include <nrf52x/device/nrf52840.h>
#include <nrf52x/device/nrf52840_bitfields.h>
#ifdef WEAVE_PACKAGE
#undef PACKAGE
#define PACKAGE WEAVE_PACKAGE
#endif

#define USE_INLINE_FUNCTIONS 0
#include <nlplatform/arch/nlplatform_arm_cm3.h>
#undef USE_INLINE_FUNCTIONS
#include <stdbool.h>
#include <string.h>

#include <nlplatform_nrf52x/nrf52x_gpio.h>
#include <nlplatform_nrf52x/nlflash_nrf52x.h>

#define FLASH_INTERNAL_MIN_WRITE_SIZE NRF52X_INTERNAL_FLASH_WRITE_SIZE
#define FLASH_INTERNAL_ERASE_SIZE     NRF52X_INTERNAL_FLASH_ERASE_SIZE

// Add 1 octet before the nlopenthread allocated PSDU transmit buffer
// so the nlradio driver can do zero-copy transmits by putting the
// length in this spot in the buffer and passing it straight to the
// Nordic radio HW as the transmit packet.
#define NLOT_RADIO_PRE_TX_PSDU_PADDING_OCTETS 1

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint32_t SP_main;
    void (*ResetExceptionHandler)(void);
    void (*NMIExceptionHandler)(void);
    void (*HardFaultExceptionHandler)(void);
    void (*MemoryManagementExceptionHandler)(void);
    void (*BusFaultExceptionHandler)(void);
    void (*UsageFaultExceptionHandler)(void);
    void (*reserved0)(void);
    void (*reserved1)(void);
    void (*reserved2)(void);
    void (*reserved3)(void);
    void (*SVCExceptionHandler)(void);
    void (*DebugMonExceptionHandler)(void);
    void (*reserved4)(void);
    void (*PendSVExceptionHandler)(void);
    void (*SysTickExceptionHandler)(void);

    /* NVIC external interrupts */
    void (*PowerClock_IRQHandler)(void);
    void (*Radio_IRQHandler)(void);
    void (*UARTE0_IRQHandler)(void);
    void (*SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler)(void);
    void (*SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler)(void);
    void (*NFCT_IRQHandler)(void);
    void (*GPIOTE_IRQHandler)(void);
    void (*SAADC_IRQHandler)(void);
    void (*TIMER0_IRQHandler)(void);
    void (*TIMER1_IRQHandler)(void);
    void (*TIMER2_IRQHandler)(void);
    void (*RTC0_IRQHandler)(void);
    void (*TEMP_IRQHandler)(void);
    void (*RNG_IRQHandler)(void);
    void (*ECB_IRQHandler)(void);
    void (*CCM_AAR_IRQHandler)(void);
    void (*WDT_IRQHandler)(void);
    void (*RTC1_IRQHandler)(void);
    void (*QDEC_IRQHandler)(void);
    void (*COMP_LPCOMP_IRQHandler)(void);
    void (*SWI0_EGU0_IRQHandler)(void);
    void (*SWI1_EGU1_IRQHandler)(void);
    void (*SWI2_EGU2_IRQHandler)(void);
    void (*SWI3_EGU3_IRQHandler)(void);
    void (*SWI4_EGU4_IRQHandler)(void);
    void (*SWI5_EGU5_IRQHandler)(void);
    void (*TIMER3_IRQHandler)(void);
    void (*TIMER4_IRQHandler)(void);
    void (*PWM0_IRQHandler)(void);
    void (*PDM_IRQHandler)(void);
    void (*reserved5)(void);
    void (*reserved6)(void);
    void (*MWU_IRQHandler)(void);
    void (*PWM1_IRQHandler)(void);
    void (*PWM2_IRQHandler)(void);
    void (*SPIM2_SPIS2_SPI2_IRQHandler)(void);
    void (*RTC2_IRQHandler)(void);
    void (*I2S_IRQHandler)(void);
    void (*FPU_IRQHandler)(void);
    void (*USBD_IRQHandler)(void);
    void (*UARTE1_IRQHandler)(void);
    void (*QSPI_IRQHandler)(void);
    void (*CRYPTOCELL_IRQHandler)(void);
    void (*SPIM3_IRQHandler)(void);
    void (*reserved7)(void);
    void (*PWM3_IRQHandler)(void);
} vector_table_t;

/* Structure we put at the beginning of an app image. Used
 * by the bootloader to get information about the app.
 */
typedef struct
{
    uint32_t SP_main;
    void (*AppEntryPoint)(void);
    // address of image signature, for secure boot verification.
    // it is also the address of the end of the image since the
    // signature is placed by linker script at the end.
    void *signatureAddress;
} app_image_header_t;

static const uint32_t reset_evMasks[] =
{
    POWER_RESETREAS_RESETPIN_Msk,
    POWER_RESETREAS_DOG_Msk,
    POWER_RESETREAS_SREQ_Msk,
    POWER_RESETREAS_LOCKUP_Msk,
    POWER_RESETREAS_OFF_Msk,
    POWER_RESETREAS_LPCOMP_Msk,
    POWER_RESETREAS_DIF_Msk,
    POWER_RESETREAS_NFC_Msk,
    POWER_RESETREAS_VBUS_Msk
};

static const char * const reset_evDescs[] =
{
    "Reset pin",
    "WatchDog",
    "Software Reset",
    "CPU Lockup",
    "GPIO detected",
    "Analog detected",
    "Debug interface",
    "NFC field detected",
    "VBUS rising"
};

extern uint32_t nl_watchDog_lastFlags;

static inline void nlplatform_irq_enable(IRQn_Type irq) {
    NVIC_EnableIRQ(irq);
}

static inline void nlplatform_irq_disable(IRQn_Type irq) {
    NVIC_DisableIRQ(irq);
}

static inline void nlplatform_irq_set_pending(IRQn_Type irq) {
    NVIC_SetPendingIRQ(irq);
}

static inline void nlplatform_irq_clear_pending(IRQn_Type irq) {
    NVIC_ClearPendingIRQ(irq);
}

static inline bool nlplatform_in_data_ram(void const * const ptr, size_t len)
{
    uintptr_t start = (uintptr_t)ptr;
    uintptr_t end = (uintptr_t)((uint8_t*)ptr + len);
    return (start >= NL_SOC_RAM_BASE_ADDRESS) && (end <= (NL_SOC_RAM_BASE_ADDRESS + NL_SOC_RAM_SIZE));
}

typedef enum {
    NLADC_INPUT_NONE = 0,
    NLADC_INPUT_AIN0 = 1,
    NLADC_INPUT_AIN1 = 2,
    NLADC_INPUT_AIN2 = 3,
    NLADC_INPUT_AIN3 = 4,
    NLADC_INPUT_AIN4 = 5,
    NLADC_INPUT_AIN5 = 6,
    NLADC_INPUT_AIN6 = 7,
    NLADC_INPUT_AIN7 = 8,
    NLADC_INPUT_VDD  = 9,
    NLADC_INPUT_VDDHDIV5 = 0xD
} nladc_input_t;

#define NLADC_INPUT_VALID(x) ((((x) >= NLADC_INPUT_NONE) && ((x) <= NLADC_INPUT_VDD)) || ((x) == NLADC_INPUT_VDDHDIV5))

typedef struct
{
    nladc_input_t   mPos;
    nladc_input_t   mNeg;
    uint32_t        mConfig; // typically a constant using NLADC_CHANNEL_CONFIG
} nladc_channel_t;

/* The output of the adc is based on these formulas, assuming resolution of 12-bits (max without oversampling):
 *   For single ended measurements: output = V(p) * GAIN/REFERENCE * 2^12 (unsigned 12-bit result)
 *   For differential measurements: output = (V(p) - V(n)) * GAIN / REFERENCE * 2^11 (signed 11-bit result)
 *
 * GAIN can be 4, 2, 1, 1/2, 1/3, 1/4, 1/5, 1/6
 * REFERENCE is either VDD/4 or an internal 0.6V reference.
 *
 * If VDD is very stable, it can be used as the reference, otherwise the 0.6V internal
 * voltage is probably a better reference. With a GAIN of 1/6, we can get full range/highest
 * resolution when measuring an AIN pin (AIN is limited to VDD, so max valid AIN is 3.6V).
 *
 * When measuring a battery or other AIN signal that is known to have a max that is lower
 * than 3.6V, then using another GAIN setting could give you more resolution. Here are
 * some limits using the 0.6V internal reference and the various gains:
 *
 * Gain     Voltage range for AIN (but must be <= VDD)
 *  4          0 - 0.15V (150mV)
 *  2          0 - 0.3V  (300mV)
 *  1          0 - 0.6V  (600mV)
 *  1/2        0 - 1.2V  (1200mV)
 *  1/3        0 - 1.8V  (1800mV)
 *  1/4        0 - 2.4V  (2400mv)
 *  1/5        0 - 3.0V  (3000mV)
 *  1/6        0 - 3.6V  (3600mV)
 */
#define NLADC_CHANNEL_CONFIG(resp, resn, gain, reference, tacq, diff, burst)  \
    ( \
        (((resp)       << SAADC_CH_CONFIG_RESP_Pos)    & SAADC_CH_CONFIG_RESP_Msk)     | \
        (((resn)       << SAADC_CH_CONFIG_RESN_Pos)    & SAADC_CH_CONFIG_RESN_Msk)     | \
        (((gain)       << SAADC_CH_CONFIG_GAIN_Pos)    & SAADC_CH_CONFIG_GAIN_Msk)     | \
        (((reference)  << SAADC_CH_CONFIG_REFSEL_Pos)  & SAADC_CH_CONFIG_REFSEL_Msk)   | \
        (((tacq)       << SAADC_CH_CONFIG_TACQ_Pos)    & SAADC_CH_CONFIG_TACQ_Msk)     | \
        (((diff)       << SAADC_CH_CONFIG_MODE_Pos)    & SAADC_CH_CONFIG_MODE_Msk)     | \
        (((burst)      << SAADC_CH_CONFIG_BURST_Pos)   & SAADC_CH_CONFIG_BURST_Msk)      \
    )

/* Some common configs */
#define NLADC_CHANNEL_CONFIG_SINGLE_MAX_1200MV(resp, resn, tacq) \
    NLADC_CHANNEL_CONFIG(resp, resn, SAADC_CH_CONFIG_GAIN_Gain1_2, SAADC_CH_CONFIG_REFSEL_Internal, tacq, SAADC_CH_CONFIG_MODE_SE, SAADC_CH_CONFIG_BURST_Disabled)

#define NLADC_CHANNEL_CONFIG_SINGLE_MAX_1800MV(resp, resn, tacq) \
    NLADC_CHANNEL_CONFIG(resp, resn, SAADC_CH_CONFIG_GAIN_Gain1_3, SAADC_CH_CONFIG_REFSEL_Internal, tacq, SAADC_CH_CONFIG_MODE_SE, SAADC_CH_CONFIG_BURST_Disabled)

#define NLADC_CHANNEL_CONFIG_SINGLE_MAX_2400MV(resp, resn, tacq) \
    NLADC_CHANNEL_CONFIG(resp, resn, SAADC_CH_CONFIG_GAIN_Gain1_4, SAADC_CH_CONFIG_REFSEL_Internal, tacq, SAADC_CH_CONFIG_MODE_SE, SAADC_CH_CONFIG_BURST_Disabled)

#define NLADC_CHANNEL_CONFIG_SINGLE_MAX_3000MV(resp, resn, tacq) \
    NLADC_CHANNEL_CONFIG(resp, resn, SAADC_CH_CONFIG_GAIN_Gain1_5, SAADC_CH_CONFIG_REFSEL_Internal, tacq, SAADC_CH_CONFIG_MODE_SE, SAADC_CH_CONFIG_BURST_Disabled)

#define NLADC_CHANNEL_CONFIG_SINGLE_MAX_3600MV(resp, resn, tacq) \
    NLADC_CHANNEL_CONFIG(resp, resn, SAADC_CH_CONFIG_GAIN_Gain1_6, SAADC_CH_CONFIG_REFSEL_Internal, tacq, SAADC_CH_CONFIG_MODE_SE, SAADC_CH_CONFIG_BURST_Disabled)


struct nladc_config_s {
    // Number of Channels, valid values are from 1 to NLADC_MAX_CHANNEL
    size_t                  mCount;
    // Configuration for each channel
    const nladc_channel_t  *mChannels;
};

#define NLADC_MAX_CHANNEL     8 // Maximum number of channels
#define ADC_SAMPLE_SIZE_BITS 12 // 12-bits

#define NL_PLATFORM_PCLK_1M         (1UL * 1000 * 1000)
#define NL_PLATFORM_PCLK_16M        (16UL * 1000 * 1000)
#define NL_PLATFORM_PWM_FREQ_MAX    (NL_PLATFORM_PCLK_16M / 3)

 // CryptoCell CRYS_HASHUserContext_t is 240 bytes, same for SHA1 and SHA256
struct nlplatform_sha1_s
{
    uint32_t hidden[240/4];
};

struct nlplatform_sha256_s
{
    uint32_t hidden[240/4];
};

/* For AES-CMAC, the context consists of the SaSiAesUserContext_t and a
 * buffer for keeping one AES-128 worth of data. We collect data
 * passed to nlplatform_AES_CMAC_update() until we have more than
 * a buffer's worth and pass that to the CryptoCell AesBlock() function.
 * Having the last block is needed in order to finish correctly.
 */
struct nlplatform_aes_cmac_s
{
    uint32_t hidden[19+4+1]; /* 76 bytes for SaSiAesUserContext_t, 16 for AES-128 block buffer, 1 for bytes in block buffer */
};

uint32_t nlplatform_get_last_reset_cause(void);
uint32_t nlplatform_get_wakeup_source(void);
uint32_t nlplatform_get_random_number(void);
int nlplatform_get_entropy(unsigned char *outEntropy, size_t inSize);
void nlplatform_cstartup_init(void);
void nlplatform_reset_info_init_done(void);
void nlplatform_configure_stack_guard(void);
void nlproduct_prepare_reset(nl_reset_reason_t reset_reason);
nl_reset_reason_t nlplatform_get_reset_reason(void);
bool nlplatform_bootloader_is_locked(void); // Returns if bootloader FLASH is locked or not
void nlplatform_crypto_init(void);
void nlplatform_cstartup_init(void);

// Nordic NRF52840 ACL support. Since it's not likely
// that other chips will have an ACL, not making this a
// nlplatform module out of it.
#define NL_MAX_ACL_REGIONS 8

// Platform reserves two ACLs to prevent write & erase
// to the bootloader and sysenv.
//
// App can use the remaining 6.
typedef enum {
    REGION0_ACL_ID = 0,
    REGION1_ACL_ID = 1,
    REGION2_ACL_ID = 2,
    REGION3_ACL_ID = 3,
    REGION4_ACL_ID = 4,
    REGION5_ACL_ID = 5,
    REGION6_ACL_ID = 6,
    REGION7_ACL_ID = 7,
    MAX_ACL_ID     = REGION7_ACL_ID,
    FLASH_ACL_ID   = REGION0_ACL_ID,
    SYSENV_ACL_ID  = REGION1_ACL_ID
} nlacl_id_t;

// Helper permission definitions based on the Nordic bitfield definitions.
// Attemps to do a blocked action generates a bus fault.
#define NL_ACL_PERMISSION_BLOCK_NONE (0)
#define NL_ACL_PERMISSION_BLOCK_WRITE_ERASE  (ACL_ACL_PERM_WRITE_Msk) /* execute and read are allowed, write and erase are blocked */
#define NL_ACL_PERMISSION_BLOCK_EXECUTE_READ (ACL_ACL_PERM_READ_Msk)  /* execute and read are blocked, write and erase are allowed. doesn't seem that useful */
#define NL_ACL_PERMISSION_BLOCK_ALL (ACL_ACL_PERM_WRITE_Msk | ACL_ACL_PERM_READ_Msk)

void nlplatform_acl_configure(nlacl_id_t acl_id, unsigned region_start_addr, unsigned region_size, unsigned permission);

/********************/
/* nrf52840 UICR    */
/********************/
 /* UICR register used to indicate device should be kept
  * unlocked (so product release bootloader doesn't try to
  * lock it on each boot).
  * Value of 0xffffffff means not unlocked.
  * Any other value means device unlocked
  */
#define UNLOCK_UICR_CUSTOMER_INDEX 0
#define  UNLOCK_VALUE_NO_UNLOCK_TOKEN_RECEIVED 0xffffffff
#define  UNLOCK_VALUE_UNLOCK_TOKEN_RECEIVED 0x0

// Nordic external debugger control:

// If UICR->APPROTECT is enabled, erase and restore UICR except
//   for the APPROTECT value (leaving it disabled) and sets
//   UICR flag to indicate device is in persistent unlocked state.
void nlplatform_external_debugger_access_enable(void *scratch_buffer, size_t scratch_buffer_size);
// If UICR->APPROTECT is disabled and device is not in a persistent
//   unlocked state, set UICR->APPROTECT to enabled state to block
//   external debugger access and reboot.
void nlplatform_external_debugger_access_disable(void);
// Returns whether external debugger access is enabled or not
bool nlplatform_get_external_debugger_access(void);

#ifdef __cplusplus
}
#endif

#endif /* _NLPLATFORM_SOC_H_ */
