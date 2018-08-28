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

#if defined(BUILD_FEATURE_OPENTHREAD) && defined(BUILD_FEATURE_OPENTHREAD_DIAGS)

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <nlassert.h>
#include <nlplatform.h>
#include <nlproduct_config.h>
#include <nlplatform/nlplatform_diags.h>
#include <nlplatform/nlgpio.h>
#include <nlplatform/nlradio.h>
#include "nlradio_priv.h"

#include <openthread/platform/radio.h>
#include <openthread/platform/alarm-milli.h>

#include <nrf52x/device/nrf52840_bitfields.h>

struct Command
{
    const char *mName;
    void (*mCommand)(int argc, char *argv[], char *aOutput, size_t aOutputMaxLen);
};

#if defined(PIN_6LO_FEM_CRX) && defined(PIN_6LO_FEM_CTX)
static void ProcessFemMode(int argc, char *argv[], char *aOutput, size_t aOutputMaxLen);
#endif
static void ProcessGpio(int argc, char *argv[], char *aOutput, size_t aOutputMaxLen);
static void ProcessRFPowerDcDcEn(int argc, char *argv[], char *aOutput, size_t aOutputMaxLen);
#if defined(PIN_DYBUCK_EN)
static void ProcessVoltage(int argc, char *argv[], char *aOutput, size_t aOutputMaxLen);

#if defined(BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM)
static void ProcessTransmit(int argc, char *argv[], char *aOutput, size_t aOutputMaxLen);

#if defined(BUILD_FEATURE_TXPOWLOOKUP_TABLE_IN_SYSENV)
static void ProcessTxPower(int argc, char *argv[], char *aOutput, size_t aOutputMaxLen);
#endif

#endif // BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM
#endif // PIN_DYBUCK_EN

static int32_t sPacketsLeftToTransmit;
static uint32_t sRepeatPeriodMilliseconds;
static bool sUserPayload;
static otInstance *sInstance;
static otRadioFrame *sTxPacket;

static const struct Command sCommands[] =
{
    { "dcdcen",   &ProcessRFPowerDcDcEn },
#if defined(PIN_6LO_FEM_CRX) && defined(PIN_6LO_FEM_CTX)
    { "femmode",  &ProcessFemMode },
#endif
    { "gpio",     &ProcessGpio },
#if defined(PIN_DYBUCK_EN)
    { "vcc2",     &ProcessVoltage },
#if defined(BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM)
    { "transmit", &ProcessTransmit },
#if defined(BUILD_FEATURE_TXPOWLOOKUP_TABLE_IN_SYSENV)
    { "txPower",  &ProcessTxPower },
#endif
#endif // defined(BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM)
#endif // defined(PIN_DYBUCK_EN)
};

#if defined(PIN_DYBUCK_EN)
static const char *const voltage_table[] = {
    "v0",  // EN|SEL0|SEL1|SEL2: 1000
    "v1",  // EN|SEL0|SEL1|SEL2: 1001
    "v2",  // EN|SEL0|SEL1|SEL2: 1010
    "v3",  // EN|SEL0|SEL1|SEL2: 1011
    "v4",  // EN|SEL0|SEL1|SEL2: 1100
    "v5",  // EN|SEL0|SEL1|SEL2: 1101
    "v6",  // EN|SEL0|SEL1|SEL2: 1110
    "v7",  // EN|SEL0|SEL1|SEL2: 1111
};
#endif // PIN_DYBUCK_EN

#if defined(PIN_6LO_FEM_CRX) && defined(PIN_6LO_FEM_CTX)
static const struct {
    uint8_t csd_value:1;
    uint8_t cps_value:1;
    uint8_t crx_value:1;
    uint8_t ctx_value:1;
    uint8_t chl_value:1;
    uint8_t unused:3;
} femmode_table[] =
{
    {
        // DEEP SLEEP MODE: CSD|CPS|CRX|CTX|CHL: 00000
        .csd_value = 0,
        .cps_value = 0,
        .crx_value = 0,
        .ctx_value = 0,
        .chl_value = 0,
    },
    {
        // RECEIVE LNA MODE: CSD|CPS|CRX|CTX|CHL: 10100
        .csd_value = 1,
        .cps_value = 0,
        .crx_value = 1,
        .ctx_value = 0,
        .chl_value = 0,
    },
    {
        // TX HIGH-POWER MODE: CSD|CPS|CRX|CTX|CHL: 10011
        .csd_value = 1,
        .cps_value = 0,
        .crx_value = 0,
        .ctx_value = 1,
        .chl_value = 1,
    },
    {
        // TX LOW-POWER MODE: CSD|CPS|CRX|CTX|CHL: 10010
        .csd_value = 1,
        .cps_value = 0,
        .crx_value = 0,
        .ctx_value = 1,
        .chl_value = 0,
    },
    {
        // RX BYPASS MODE: CSD|CPS|CRX|CTX|CHL: 11100
        .csd_value = 1,
        .cps_value = 1,
        .crx_value = 1,
        .ctx_value = 0,
        .chl_value = 0,
    },
    {
        // TX BYPASS MODE: CSD|CPS|CRX|CTX|CHL: 11010
        .csd_value = 1,
        .cps_value = 1,
        .crx_value = 0,
        .ctx_value = 1,
        .chl_value = 0,
    },
    {
        // SLEEP MODE: CSD|CPS|CRX|CTX|CHL: 10000
        .csd_value = 1,
        .cps_value = 0,
        .crx_value = 0,
        .ctx_value = 0,
        .chl_value = 0,
    },
};
#endif // PIN_6LO_FEM_CRX && PIN_6LO_FEM_CTX

void nlplatform_diags(otInstance *aInstance, int argc, char *argv[], char *aOutput, size_t aOutputMaxLen)
{
    sInstance = aInstance;
    if (sTxPacket == NULL)
    {
        sTxPacket = otPlatRadioGetTransmitBuffer(sInstance);
    }
    if (argc > 0)
    {
        unsigned int i;

        for (i = 0; i < ARRAY_SIZE(sCommands); i++)
        {
            if (strcmp(argv[0], sCommands[i].mName) == 0)
            {
                sCommands[i].mCommand(argc - 1, ((argc > 1) ? &argv[1]: NULL), aOutput, aOutputMaxLen);
                break;
            }
        }

        if (i == ARRAY_SIZE(sCommands))
        {
           snprintf(aOutput, aOutputMaxLen, "diagnostics feature '%s' is not supported\n", argv[0]);
        }
    }
    else
    {
        snprintf(aOutput, aOutputMaxLen, "diagnostics feature is not specified\n");
    }
}

void nlplatform_diags_alarm(otInstance *aInstance)
{
    // check if we were cancelled
    if (sPacketsLeftToTransmit)
    {
        if (sUserPayload)
        {
            // increment the seq_num in the user payload to help distinguish them
            // for packet error rate testing.  don't increment the seq_num for
            // generated packets that RF team typically uses for factory calibration.
            ieee802154_packet_t *tx_packet = (ieee802154_packet_t*)&sTxPacket->mPsdu[-NLOT_RADIO_PRE_TX_PSDU_PADDING_OCTETS];
            tx_packet->mpdu.seq_num++;
        }

#if defined(BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM) && defined(BUILD_FEATURE_TXPOWLOOKUP_TABLE_IN_SYSENV)
        // Instruct product layer FEM module to skip powercal table lookup & use the FEM settings came as part of the transmit command
        nlproduct_FEM_use_factory_txpower_settings();
#endif

        otPlatRadioTransmit(aInstance, sTxPacket);

#if defined(BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM) && defined(BUILD_FEATURE_TXPOWLOOKUP_TABLE_IN_SYSENV)
        // Reset the use of hardcoded factory fem settings to restore the powercal table lookup.
        nlproduct_FEM_reset_use_factory_txpower_settings();
#endif

        if (sPacketsLeftToTransmit > 0)
        {
            sPacketsLeftToTransmit--;
        }
        if (sPacketsLeftToTransmit)
        {
            // schedule for next packet
            otPlatAlarmMilliStartAt(aInstance, 0, sRepeatPeriodMilliseconds);
        }
    }
}

static void ProcessRFPowerDcDcEn(int argc, char *argv[], char *aOutput, size_t aOutputMaxLen)
{
    if (argc == 1)
    {
        if (atoi(argv[0]) == 0)
        {
            NRF_POWER->DCDCEN = 0;
            snprintf(aOutput, aOutputMaxLen, "dcdcen is disabled\n");
        }
        else
        {
            NRF_POWER->DCDCEN = 1;
            snprintf(aOutput, aOutputMaxLen, "dcdcen is enabled\n");
        }
    }
    else
    {
        snprintf(aOutput, aOutputMaxLen, "error parameter\n");
    }
}

static void ProcessGpio(int argc, char *argv[], char *aOutput, size_t aOutputMaxLen)
{
    bool isSet = strcmp(argv[0], "set") == 0;
    bool isGet = strcmp(argv[0], "get") == 0;

    if (argc == 2 && isGet)
    {
        if (atoi(argv[1]) == -1)
        {
            printf("spinning to cause watchdog\n");
            while(1);
        }
        else if (atoi(argv[1]) == -2)
        {
            printf("causing an assert failure\n");
            assert(0);
            while(1);
        }
        else if (atoi(argv[1]) == -3)
        {
            printf("rebooting\n");
            nlplatform_reset(NL_RESET_REASON_SW_REQUESTED);
            while(1);
        }

        // get gpio_num
        snprintf(aOutput, aOutputMaxLen, "GPIO current input is %d\n", nlgpio_get_value(atoi(argv[1])));
    }
    else if (argc == 3 && isSet)
    {
        // set gpio_num value
        nlgpio_set_output(atoi(argv[1]), atoi(argv[2]));
        snprintf(aOutput, aOutputMaxLen, "GPIO set output value succeeded\n");
    }
    else if (argc == 4 && isSet)
    {
        // accepts a gpio config param as argv[2] but ignores it.
        // set gpio_num pin_config value
        nlgpio_set_output(atoi(argv[1]), atoi(argv[3]));
        snprintf(aOutput, aOutputMaxLen, "GPIO set output value succeeded\n");
    }
    else
    {
        snprintf(aOutput, aOutputMaxLen,
                          "Error: Illegal arguments\n"
                           "Usage:\n"
                           "   gpio get <pinnum>\n"
                           "   gpio set <pinnum> <value>\n"
                           "   gpio set <pinnum> <config> <value>\n");
    }
}

#if defined(PIN_DYBUCK_EN)
static void ProcessVoltage(int argc, char *argv[], char *aOutput, size_t aOutputMaxLen)
{
    bool error = false;

    if (argc == 1)
    {
        unsigned int i;

        for (i = 0; i < ARRAY_SIZE(voltage_table); i++)
        {
            if (strcmp(voltage_table[i], argv[0]) == 0)
            {
                break;
            }
        }

        if (i < ARRAY_SIZE(voltage_table))
        {
            nlgpio_set_output(PIN_DYBUCK_EN, 1);
            nlgpio_set_output(PIN_DYBUCK_SEL0, (i >> 2) & 0x1);
            nlgpio_set_output(PIN_DYBUCK_SEL1, (i >> 1) & 0x1);
            nlgpio_set_output(PIN_DYBUCK_SEL2, i & 0x1);
            snprintf(aOutput, aOutputMaxLen, "Set %s voltage value succeeded\n", argv[0]);
        }
        else
        {
            error = true;
        }
    }

    if (argc == 0 || argc > 1 || error)
    {
        snprintf(aOutput, aOutputMaxLen,
                          "Error: Illegal arguments\n"
                           "Usage:\n"
                           "   vcc2 <voltage value>\n"
                           "   <voltage value>:v0 ~ v7\n");
    }
}

#if defined(BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM)
// Similar to the generic OT diags cmd "send" and "repeat" except all the needed information
// for the transmit is given on the cmd line, including vcc2 for FEM + tx_power
// encoded together as the power argument to otPlatRadioTransmit(), channel,
// count, interval, payload. Count is the number of packets to transmit (-1 for infinite).
// Interval is the time in milliseconds between packets.
// Payload is a string of hex digits, two characters per octet. The number of octets in
// the payload string is the length of the payload. A negative payload number means send
// a generated number of bytes as the payload instead of using the user specified payload.
static void ProcessTransmit(int argc, char *argv[], char *aOutput, size_t aOutputMaxLen)
{
    bool error = false;

    if (argc == 1)
    {
        if (strcmp("stop", argv[0]) == 0)
        {
            if (sPacketsLeftToTransmit)
            {
                otPlatAlarmMilliStop(sInstance);
                sRepeatPeriodMilliseconds = 0;
                sPacketsLeftToTransmit = 0;
                snprintf(aOutput, aOutputMaxLen,
                         "Stopped previous transmit operation\n");
            }
            else
            {
                snprintf(aOutput, aOutputMaxLen,
                         "Error: transmit not in progress\n");
            }
        }
        else
        {
            error = true;
        }
    }
    else if (argc == 7)
    {
        unsigned int i;
        int chl_setting = strtol(argv[0], NULL, 10);
        int txpower_setting = strtol(argv[2], NULL, 10);
        int channel = strtol(argv[3], NULL, 10);
        int num_packets = strtol(argv[4], NULL, 10);
        int packet_length = 0;
#if defined(BUILD_FEATURE_USE_NORDIC_OT_RADIO_DRIVER)
        uint8_t temp_channel;
#endif

        if (sPacketsLeftToTransmit)
        {
            snprintf(aOutput, aOutputMaxLen,
                     "Error: repeating transmit already active, stop first\n");
            goto out;
        }

        if ((num_packets < 1) && (num_packets != -1))
        {
            snprintf(aOutput, aOutputMaxLen,
                     "Error: num_packets must be 1 or more, or -1 for infinite\n");
            error = true;
            goto err_out;
        }

        if ((chl_setting < 0) || (chl_setting > 2))
        {
            snprintf(aOutput, aOutputMaxLen, "Error: Valid FEM mode(CHL) must be between 0 and 2\n");
            error = true;
            goto err_out;
        }

        for (i = 0; i < ARRAY_SIZE(voltage_table); i++)
        {
            if (strcmp(voltage_table[i], argv[1]) == 0)
            {
                break;
            }
        }

        if (i < ARRAY_SIZE(voltage_table))
        {
#if defined(BUILD_FEATURE_TXPOWLOOKUP_TABLE_IN_SYSENV)
            nlproduct_encode_factory_transmit_power(chl_setting, i, txpower_setting);
#else
            nlencoded_transmit_power_t encoded_power = nlproduct_encode_transmit_power(chl_setting, i, txpower_setting);
            otPlatRadioSetTransmitPower(sInstance, encoded_power.raw_int);
#endif
        }
        else
        {
            snprintf(aOutput, aOutputMaxLen, "Error: VCC2 must be between 0 and 7\n");
            error = true;
            goto err_out;
        }

        if (channel < 11 || channel > 26)
        {
            snprintf(aOutput, aOutputMaxLen, "Error: Channel must be between 11 and 26\n");
            error = true;
            goto err_out;
        }

        // payload is expected to be an ascii hex string, or a single negative
        // integer that specifies the number of octets to send in a
        // generated payload.
        const char *payload_string = argv[6];
        if (payload_string[0] == '-')
        {
            packet_length = -strtol(payload_string, NULL, 10);
            if (packet_length < 0 || packet_length > OT_RADIO_FRAME_MAX_SIZE)
            {
                snprintf(aOutput, aOutputMaxLen,
                         "Error: payload length %d too long, max is %d\n", packet_length, OT_RADIO_FRAME_MAX_SIZE);
                error = true;
                goto err_out;
            }
            for (i = 0; i <  packet_length - 2; i++) // last 2 octets of packet_length is CRC added by driver
            {
                sTxPacket->mPsdu[i] = i;
            }
            sUserPayload = false;
        }
        else
        {
            // convert two characters at a time to hex digits for the psdu.
            char buf[3];
            buf[2] = 0; // null terminate
            i = 0;
            while (*payload_string)
            {
                buf[0] = *payload_string++;
                buf[1] = *payload_string++;
                sTxPacket->mPsdu[i++] = strtoul(buf, NULL, 16);
            }
            if (i == 0)
            {
                snprintf(aOutput, aOutputMaxLen,
                         "Error: no valid payload provided\n");
                error = true;
                goto err_out;
            }
            packet_length = i + 2; // add 2 for the crc/fcs bytes
            if (i > OT_RADIO_FRAME_MAX_SIZE)
            {
                snprintf(aOutput, aOutputMaxLen,
                         "Error: payload string %d too long, max is %d octets\n", i - 2, OT_RADIO_FRAME_MAX_SIZE - 2);
                error = true;
                goto err_out;
            }
            sUserPayload = true;
        }

        sTxPacket->mChannel = channel;
        sTxPacket->mLength = packet_length;

#if defined(BUILD_FEATURE_USE_NORDIC_OT_RADIO_DRIVER)
       /* With each diag transmit command we could have a diff set of FEM settings for the current channel which needs to be applied before transmit.
        * Radio driver channel caching feature avoids the use of these FEM settings if the incoming channel matches the cached channel.
        * So to bypass this radio driver behavior, create a temporary channel change by going into adjecent channel.
        */
        temp_channel = (channel == 26) ? 11 : (channel + 1);
        otPlatRadioReceive(sInstance, temp_channel);
#endif

        if ((num_packets == -1) || (num_packets > 1))
        {
            sRepeatPeriodMilliseconds = strtoul(argv[5], NULL, 10);
            if (sRepeatPeriodMilliseconds == 0)
            {
                snprintf(aOutput, aOutputMaxLen,
                         "Error: repeat period must be non-zero\n");
                goto out;
            }
            sPacketsLeftToTransmit = num_packets;
            otPlatAlarmMilliStartAt(sInstance, otPlatAlarmMilliGetNow(), sRepeatPeriodMilliseconds);
            if (num_packets == -1)
            {
                snprintf(aOutput, aOutputMaxLen,
                         "sending infinite packets of length %u, of %s payload, channel %d, with delay of %u ms\r\n",
                         sTxPacket->mLength, sUserPayload ? "user provided" : "auto generated",
                         sTxPacket->mChannel, sRepeatPeriodMilliseconds);
            }
            else
            {
                snprintf(aOutput, aOutputMaxLen,
                         "sending %u packets of length %u, of %s payload, channel %d, with delay of %u ms\r\n",
                         num_packets, sTxPacket->mLength, sUserPayload ? "user provided" : "auto generated",
                         sTxPacket->mChannel, sRepeatPeriodMilliseconds);
            }
        }
        else
        {
            snprintf(aOutput, aOutputMaxLen, "sending one packet of length %u, channel %d\r\n",
                     sTxPacket->mLength, sTxPacket->mChannel);

#if defined(BUILD_FEATURE_TXPOWLOOKUP_TABLE_IN_SYSENV)
            // Instruct product layer FEM module to skip powercal table lookup & use the FEM settings came as part of the transmit command
            nlproduct_FEM_use_factory_txpower_settings();
#endif

            otPlatRadioTransmit(sInstance, sTxPacket);

#if defined(BUILD_FEATURE_TXPOWLOOKUP_TABLE_IN_SYSENV)
            // Reset the use of hardcoded factory fem settings to restore the powercal table lookup.
            nlproduct_FEM_reset_use_factory_txpower_settings();
#endif

            sPacketsLeftToTransmit = 0;
            sRepeatPeriodMilliseconds = 0;
        }
    }
err_out:
    if (argc == 0 || argc > 8 || error)
    {
        snprintf(aOutput, aOutputMaxLen,
                 "Usage:\ttransmit [stop | <chl> <vcc2> <txpwr> <chan> <#pkts> <repeat_ms> <payload>]\n"
                 "\te.g. transmit 0 v7 0 11 -1 16 -64\n"
                 "\te.g. transmit 0 v7 0 11 100 100 000c000000560a00010030b418\n");
    }
out:
    return;
}

#if defined(BUILD_FEATURE_TXPOWLOOKUP_TABLE_IN_SYSENV)
static void ProcessTxPower(int argc, char *argv[], char *aOutput, size_t aOutputMaxLen)
{
    int target_power;
    uint16_t reg_code;

    if (argc != 2)
    {
        return;
    }

    reg_code = atoi(argv[0]);
    target_power = strtol(argv[1], NULL, 10);

    snprintf(aOutput, aOutputMaxLen, "Setting PowerCal override reg_code(%u) target_power(%d)\n", reg_code, target_power);

    nlproduct_factory_override_reg_domain(reg_code);
    nlradio_set_target_txpower(target_power);    
}
#endif // BUILD_FEATURE_TXPOWLOOKUP_TABLE_IN_SYSENV

#endif // BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM
#endif // PIN_DYBUCK_EN

#if defined(PIN_6LO_FEM_CRX) && defined(PIN_6LO_FEM_CTX)
static void ProcessFemMode(int argc, char *argv[], char *aOutput, size_t aOutputMaxLen)
{
    bool error = false;

    if (argc == 1)
    {
        unsigned int mode = atoi(argv[0]);

        if (mode < ARRAY_SIZE(femmode_table))
        {
#if !defined(PIN_6LO_FEM_CSD)
            // if PIN_6LOW_FEM_CSD is not defined in product config header, FEM mode 0 would not be supported
            if (mode == 0)
            {
                error = true;
                goto exit;
            }
#else
            nlgpio_set_output(PIN_6LO_FEM_CSD, femmode_table[mode].csd_value);
#endif

#if !defined(PIN_6LO_FEM_CHL)
            // if PIN_6LO_FEM_CHL is not defined in product config header, FEM mode 2 would not be supported
            if (mode == 2)
            {
                error = true;
                goto exit;
            }
#else
            nlgpio_set_output(PIN_6LO_FEM_CHL, femmode_table[mode].chl_value);
#endif

#if !defined(PIN_6LO_FEM_CPS)
            // if PIN_6LO_FEM_CPS is not defined in product config header, FEM mode 4,5 would not be supported
            if (mode == 4 || mode == 5)
            {
                error = true;
                goto exit;
            }
#else
            nlgpio_set_output(PIN_6LO_FEM_CPS, femmode_table[mode].cps_value);
#endif

            nlgpio_set_output(PIN_6LO_FEM_CRX, femmode_table[mode].crx_value);
            nlgpio_set_output(PIN_6LO_FEM_CTX, femmode_table[mode].ctx_value);
            snprintf(aOutput, aOutputMaxLen, "Set FEM mode %s succeeded\n", argv[0]);
        }
        else
        {
            error = true;
        }
    }

#if !defined(PIN_6LO_FEM_CSD) || !defined(PIN_6LO_FEM_CPS) || !defined(PIN_6LO_FEM_CHL)
exit:
#endif

    if (argc == 0 || argc > 1 || error)
    {
        snprintf(aOutput, aOutputMaxLen,
                          "Error: Illegal arguments\n"
                           "Usage:\n"
                           "   femmode <mode>\n"
                           "   <mode>:"
#if defined(PIN_6LO_FEM_CSD)
                           "0, "
#endif
                           "1, "
#if defined(PIN_6LO_FEM_CHL)
                           "2, "
#endif
                           "3, "
#if defined(PIN_6LO_FEM_CPS)
                           "4, 5, "
#endif
                           "6\n"
                           );
    }
}
#endif // PIN_6LO_FEM_CRX && PIN_6LO_FEM_CTX

#endif // defined(BUILD_FEATURE_OPENTHREAD) && defined(BUILD_FEATURE_OPENTHREAD_DIAGS)
