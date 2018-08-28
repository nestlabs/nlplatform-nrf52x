/*
 *
 *    Copyright (c) 2018 Nest Labs, Inc.
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
 *       This is an nlradio abstraction layer for the Nordic OpenThread
 *       radio driver.
 */

#include <stdlib.h>
#include <string.h>

#include <nlassert.h>
#include <nlutilities.h>
#include <nlsysenv.h>
#include <nlopenthread.h>
#include <nlplatform.h>
#include <nlplatform/nlradio.h>
#include <nlplatform/nltime.h>
#include <nlplatform/nltimer.h>
#if defined(BUILD_FEATURE_NL_PROFILE)
#include <nlplatform/nlprofile.h>
#endif

#include <nrf_802154.h>
#include <nrf_802154_const.h>
#include <nrf_802154_critical_section.h>
#include <nrf_802154_core.h>

/**
 **************************
 * Local Symbols and Types
 **************************
 */

#ifdef NLRADIO_DEFAULT_NUM_RX_BUFFERS
#define NLRADIO_NUM_RX_BUFFERS          (NLRADIO_DEFAULT_NUM_RX_BUFFERS)
#else
#define NLRADIO_NUM_RX_BUFFERS          (4)
#endif

#define NLRADIO_SLEEP_WAIT_MS           (20)
#define NLRADIO_TX_WAIT_MS              (10)
#define NLRADIO_ESCAN_WAIT_MS           (10)

#if defined(BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM)
#define NLRADIO_MAX_RADIO_TX_POWER      (RADIO_TXPOWER_TXPOWER_0dBm)
#else
#define NLRADIO_MAX_RADIO_TX_POWER      (RADIO_TXPOWER_TXPOWER_Pos8dBm)
#endif

#define NLRADIO_RECEIVE_SENSITIVITY     (-100)   // dBm

#if defined(BUILD_FEATURE_ANTENNA_DIVERSITY)
// Indicates the number of bits we use to save the history
#define NL_RADIO_NUM_BITS                   ((sizeof(s_tx_failure_ota))*8)
// Switch antenna if we see more than 50% failures
#define NL_RADIO_FAILURE_THRESH             ((NL_RADIO_NUM_BITS)/2)
// Invalid or unknown RSSI value
#define NLRADIO_INVALID_RSSI                (127)
// Precision multiple for RSS average (1 << NLRADIO_PRECISON_BIT_SHIFT)
#define NLRADIO_RSSI_PRECISON_SHIFT         (3)
#define NLRADIO_PRECISON                    (1 << NLRADIO_RSSI_PRECISON_SHIFT)
#define NLRADIO_INVALID_RSSI_PRECISON_SHIFT (NLRADIO_INVALID_RSSI << NLRADIO_RSSI_PRECISON_SHIFT)
#define NLRADIO_RSSI_PRECISON_MASK          (NLRADIO_PRECISON - 1)
// Coefficient used for exponentially weighted filter (1 << NLRADIO_RSSI_COEFF_SHIFT)
#define NLRADIO_RSSI_COEFF_SHIFT            (3)
#endif

/**
 ****************************
 * Typedefs and Prototypes
 ****************************
 */

/**
 * This structure contains an IEEE 802.15.4 radio frame data.
 */
typedef struct nlradio_packet_s
{
    uint8_t                            length;                  ///< Length of the PSDU.
    uint8_t                            psdu[MAX_PACKET_SIZE];   ///< The PSDU.
    uint8_t                            channel;                 ///< Channel used to transmit/receive the frame.
    int8_t                             power;                   ///< Transmit/receive power in dBm.
    uint8_t                            lqi;                     ///< Link Quality Indicator for received frames.
    volatile bool                      in_use;                  ///< packet is in use
    volatile struct nl_radio_packet_s *next;                    ///< next pointer used for RX packets
#ifdef BUILD_FEATURE_RADIO_HEADER_IE
    uint64_t                           timestamp;               ///< The timestamp in microseconds when the frame was start to receive (rx sfd).
#endif
} nlradio_packet_t;

#ifdef BUILD_FEATURE_ANTENNA_DIVERSITY
/**
 * Internal data structure contains per antenna transmission statistics counters.
 */
typedef struct nlradio_per_ant_stats_internal
{
    uint32_t tx_success_cnt;
    uint32_t tx_fail_cnt;
} nlradio_per_ant_stats_internal_t;

/**
 * Internal data structure to store per antenna tx stats & precison shifted ack rssi average.
 */
typedef struct nlradio_ant_stats_internal
{
    nlradio_per_ant_stats_internal_t antenna_stats[ANTENNA_MAX];
    int16_t                          shifted_avg_ack_rssi[ANTENNA_MAX];
    uint32_t                         ant_switch_cnt;
} nlradio_ant_stats_internal_t;
#endif

static void check_single_task(void);
static void nlradio_rx_packet_init(void);
static nlradio_packet_t *nlradio_rx_packet_alloc_isr(bool *all_used);
static bool nlradio_rx_packet_free(uint8_t *buffer);
static void nlradio_rx_packet_list_enqueue_isr(nlradio_packet_t *packet);
static nlradio_packet_t *nlradio_rx_packet_list_dequeue(void);
static void nlradio_init_internal(void);
static void nlradio_sleep_internal(void);
static void nlradio_wakeup_internal(void);

#if defined(BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM)
static void nlradio_set_transmit_power(void);
static void nlradio_set_channel_internal(uint8_t channel);
#else
static void nlradio_set_tx_power_internal(int8_t power);
#endif // BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM

#ifdef BUILD_FEATURE_ANTENNA_DIVERSITY
static void nlradio_update_avg_ack_rssi(int8_t rssi);
#endif

/**
 ****************************
 * Static control structures
 ****************************
 */

/**
 * Intermediate nlradio buffers
 */
static nlradio_packet_t s_tx_packet;
static nlradio_packet_t s_rx_packet_pool[NLRADIO_NUM_RX_BUFFERS];
static volatile nlradio_packet_t *s_rx_packet_list_head;
static volatile nlradio_packet_t *s_rx_packet_list_tail;

/**
 * Tracks the state of the radio driver.
 */
static volatile nlradio_state_t s_radio_state = k_nlradio_state_disabled;

/**
 * A transmit callback to be called when the requested TX operation completes.
 */
static volatile transmit_complete_cb s_tx_complete_callback = NULL;

#ifdef BUILD_FEATURE_RADIO_HEADER_IE
/**
 * A transmit callback to be called when the requested TX operation starts (tx sfd).
 */
static volatile transmit_start_cb s_tx_start_callback = NULL;
#endif

/**
 * A receive callback to be called when the requested RX operation completes.
 */
static volatile receive_complete_cb s_rx_callback = NULL;

/**
 * A callback to be called when energy scan completes
 */
static volatile escan_complete_cb s_escan_callback = NULL;

/**
 * State of the radio before starting an energy scan
 */
static nlradio_state_t s_pre_escan_radio_state = k_nlradio_state_disabled;

/**
 * Tracks the channel that was used to configure the radio for receive.
 */
static volatile uint8_t s_channel = 0;

#if defined(BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM)
/**
 * Target Tx power for a channel in units of 0.01 dBm
 */
static int s_target_txpower = 0;

/**
 * Tx Power backoff for a channel in units of 0.01 dBm
 */
static int s_backoff_txpower = 0;

/*
 * current product level tx power value, updated during a channel change &
 * target/offset power set operation.
 */
static nlencoded_transmit_power_t s_txpower_settings = INVALID_ENCODED_TXPOWER;
#else
/**
 * Encoded transmit power for ACK frames
 */
static int8_t s_default_tx_power;
#endif //BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM

/**
 * Tracks whether the nrf_802154_received_raw() call has been
 * made from ISR context
 */
static bool s_in_radio_isr = false;

/**
 * Track the thread that is using nlradio.
 * It's used to ensure that only one thread calls nlradio API.
 */
#ifdef NLRADIO_DEBUG
static xTaskHandle s_task = NULL;
#endif /* DEBUG */

#ifdef BUILD_FEATURE_ANTENNA_DIVERSITY
/*
 * Bits of this integer is used to keep track of
 * rolling success and failure of each TX packet.
 */
static uint32_t s_tx_failure_ota = 0;

/*
 * Bit index for the above integer
 */
static uint32_t s_tx_failure_ota_index = 0;

/*
 * Boolean to track whether antenna switching is enabled or not
 */
static bool s_prevent_antenna_switch = false;

/*
 * Data structure to hold the antenna stats.
 * Volatile type qualifier is used since this data structure is updated
 * in the radio ISR context & read from a thread context.
 */
static volatile nlradio_ant_stats_internal_t s_antenna_stats;
#endif

/**
 * This function ensures that only one task is calling nlradio API.
 * nlradio is not thread-safe and requires this check.
 */
static void check_single_task(void)
{
#ifdef NLRADIO_DEBUG
    /* Start tracking the task, which uses nlradio API */
    if (s_task == NULL)
    {
        s_task = xTaskGetCurrentTaskHandle();
    }

    assert(s_task == xTaskGetCurrentTaskHandle());
#endif /* DEBUG */
}

/**
 * Intialize the rx packet pool
 *
 * @note
 *    This function should only be called
 *    when RX interrupts are disabled.
 */
static void nlradio_rx_packet_init(void)
{
    s_rx_packet_list_head = s_rx_packet_list_tail = NULL;

    for (size_t i = 0; i < NLRADIO_NUM_RX_BUFFERS; i++)
    {
        s_rx_packet_pool[i].in_use = false;
    }
}

/**
 * Allocate a rx packet from the rx packet pool
 *
 * @note
 *    This functions is called from the RX ISR.
 *    So, this alloc function will always
 *    operate atomically with respect to the
 *    free function.
 *
 * @param[out] all_used    A pointer to a boolean variable.
 *     The value will be set to indicate whether all rx
 *     packets are in-use (after allocating the rx packet).
 *     This could be used to decide whether to disable
 *     auto-ack.
 *
 * @retval pointer to a rx packet if one is available
 * @retval NULL if a rx packet is unavailable
 */
static nlradio_packet_t *nlradio_rx_packet_alloc_isr(bool *all_used)
{
    nlradio_packet_t *p = NULL;

    *all_used = true;

    for (size_t i = 0; i < NLRADIO_NUM_RX_BUFFERS; i++)
    {
        if (!p)
        {
            if (!s_rx_packet_pool[i].in_use)
            {
                s_rx_packet_pool[i].in_use = true;
                s_rx_packet_pool[i].next = NULL;
                p = &s_rx_packet_pool[i];
            }
        }
        else
        {
            /* After allocating a rx packet (`p` being non-NULL), check
             * if we can still find another free buffer from the pool.
             * If we can, set `all_used` to false and return.
             */

            if (!s_rx_packet_pool[i].in_use)
            {
                *all_used = false;
                break;
            }
        }
    }

    return p;
}

/**
 * Free a rx packet to the rx packet pool
 *
 * @note
 *    This functions is called from the nlradio API.
 *
 * @param[in] buffer  a pointer to the psdu member of a rx packet
 *
 * @retval true if packet was freed
 * @retval false if packet was not freed
 */
static bool nlradio_rx_packet_free(uint8_t *buffer)
{
    bool result = false;

    for (size_t i = 0; i < NLRADIO_NUM_RX_BUFFERS; i++)
    {
        if (buffer == s_rx_packet_pool[i].psdu)
        {
            /* As the rx packet buffer is being freed (`in_use` set to
             * `false`) the auto-ack should be enabled (in case it
             * was disabled from the receive ISR earlier). These two
             * operations should be done atomically to ensure that the
             * freed buffer cannot be claimed by the rx ISR callback
             * before we get the chance to enable the auto-ack. So
             * they are protected by disabling/enabling interrupts.
             */

            nrf_802154_critical_section_enter();

            s_rx_packet_pool[i].next = NULL;
            s_rx_packet_pool[i].in_use = false;

            if (!nrf_802154_auto_ack_get())
            {
                nrf_802154_auto_ack_set(true);
            }

            nrf_802154_critical_section_exit();

            result = true;
            break;
        }
    }

    return result;
}

/**
 * Enqueue a rx packet to the received packet list.
 *
 * @note
 *    This functions is called from the RX ISR.
 *    So, it operates atomically with respect to
 *    the dequeue function.  If this function is
 *    not called from inside of the RX ISR there
 *    could potentially be a race condition with
 *    respect to the management of the list head
 *    and tail pointers.
 *
 * @param[in] packet  a pointer to a rx packet
 */
static void nlradio_rx_packet_list_enqueue_isr(nlradio_packet_t *packet)
{
    packet->next = NULL;

    if (s_rx_packet_list_tail == NULL)
    {
        s_rx_packet_list_tail = s_rx_packet_list_head = packet;
    }
    else
    {
        s_rx_packet_list_tail->next = (volatile struct nl_radio_packet_s *)packet;
        s_rx_packet_list_tail = packet;
    }
}

/**
 * Dequeue a rx packet from the received packet list.
 *
 * @note
 *    This functions is called from the nlradio API
 *    So, the RX interrupt must be disabled
 *    when managing the list head and tail pointers.
 *
 * @param[in] packet  a pointer to a rx packet
 *
 * @retval pointer to a rx packet if one is available
 * @retval NULL if a rx packet is unavailable
 */
static nlradio_packet_t *nlradio_rx_packet_list_dequeue(void)
{
    nlradio_packet_t *p = NULL;

    if (s_rx_packet_list_head != NULL)
    {
        p = (nlradio_packet_t *)s_rx_packet_list_head;

        nrf_802154_critical_section_enter();

        s_rx_packet_list_head = (nlradio_packet_t *)(p->next);
        if (s_rx_packet_list_head == NULL)
        {
            s_rx_packet_list_tail = NULL;
        }

        nrf_802154_critical_section_exit();

        p->next = NULL;
    }

    return p;
}

/**
 * Initialize the nlradio driver
 *
 * @note
 *    This function is called from nlradio_enable()
 *    and nlradio_disable().  In both cases nlradio
 *    state, nrf radio state, and the radio hardware
 *    state should be left in the same condition.
 *    That is radio reset with power on, channel 11,
 *    default tx power, auto ack enabled, and nrf
 *    radio driver state and hardware state should
 *    be sleep as defined by the radio state
 *    diagram in OT platform radio.h.
 */
static void nlradio_init_internal(void)
{
#ifdef BUILD_FEATURE_ANTENNA_DIVERSITY
    uint32_t count;

    for (count = 0; count < ANTENNA_MAX; count++)
    {
        s_antenna_stats.shifted_avg_ack_rssi[count] = NLRADIO_INVALID_RSSI_PRECISON_SHIFT;
    }
#endif

    // reset the radio
    nrf_radio_power_set(false);
    nrf_radio_power_set(true);

#if defined(BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM)
    nlradio_set_channel_internal(11);
#else
    s_channel = 11;

    nrf_802154_channel_set(s_channel);

    nrf_802154_tx_power_set(s_default_tx_power);
#endif //BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM

    nrf_802154_auto_ack_set(true);

    nlradio_sleep_internal();

    nlradio_rx_packet_init();

    s_rx_callback = NULL;
    s_tx_complete_callback = NULL;
#ifdef BUILD_FEATURE_RADIO_HEADER_IE
    s_tx_start_callback = NULL;
#endif
    s_escan_callback = NULL;
}

/**
 * Sleep the nlradio driver
 *
 * @note
 *    This function stops all radio activity.
 *    It stops any ack timeout timer, disables
 *    the FEM, puts the nrf radio driver and hardware
 *    in sleep mode and then releases the nlplatform
 *    sleep lock.
 */
static void nlradio_sleep_internal(void)
{
    uint16_t msTick = NLRADIO_SLEEP_WAIT_MS;

    // Call sleep until we get success indicating that we are on the way to sleep
    while (!nrf_802154_sleep() && --msTick)
    {
        nlplatform_delay_ms(1);
    }

    // Wait for the state transition to actually happen indicating that the radio is asleep
    while ((nrf_802154_state_get() != NRF_802154_STATE_SLEEP) && msTick--)
    {
        nlplatform_delay_ms(1);
    }

#if defined(BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM)
    nlproduct_FEM_disable();
#endif

#if defined(BUILD_FEATURE_NL_PROFILE)
    nl_profile_stop(NL_PROFILE_RADIO_ENABLED);
#endif
}

/**
 * Wake the nlradio driver
 *
 * @note
 *    This function prepares the radio for operation
 *    by acquiring the nlplatform sleep lock and
 *    enabling the FEM.
 */
static void nlradio_wakeup_internal(void)
{
#if defined(BUILD_FEATURE_NL_PROFILE)
    nl_profile_start(NL_PROFILE_RADIO_ENABLED);
#endif

#if defined(BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM)
    nlproduct_FEM_enable();
#endif
}

/**
 * Convert format of short address
 *
 * @note
 *    This function converts a uint16_t src
 *    to a 2 byte uint8_t array with the lsb
 *    byte of the uint16_t stored in the first
 *    byte of the array and the msb of the uint16_t
 *    stored in the second byte of the array.
 *
 * @param[in]  src  a uint16_t byte short address
 * @param[out] dest a pointer to a 2 byte Uint8_t array
 *
 */
static inline void nlradio_convert_short_address(uint8_t *dest, uint16_t src)
{
    dest[0] = (uint8_t) src;
    dest[1] = (uint8_t)(src >> 8);
}

#if defined(BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM)
/**
 * lookup tx power settings in flash and configure radio power register
 */
static void nlradio_set_transmit_power(void)
{
    nlencoded_transmit_power_t power;
    int8_t radio_tx_power;

    if (nlproduct_lookup_transmit_power(s_channel, s_target_txpower, s_backoff_txpower, &power) >= 0)
    {
        s_txpower_settings = power;
    }
    else
    {
        s_txpower_settings = INVALID_ENCODED_TXPOWER;
    }

    // when there is a FEM, the power value shouldn't be used directly
    // but must be parsed by the product FEM library, which will
    // return to us the TXPOWER register setting to use and also
    // configure the FEM accordingly when the TX is enabled next.
    // This API always returns a valid tx power radio register settings.
    radio_tx_power = nlproduct_FEM_configure_transmit_power(s_txpower_settings);

    nrf_802154_tx_power_set(radio_tx_power);
}

/**
 * Channel change API
 * @param[in] channel number
 */
static void nlradio_set_channel_internal(uint8_t channel)
{
    assert((channel >= 11) && (channel <= 26));
    s_channel = channel;
    nrf_802154_channel_set(channel);
    nlradio_set_transmit_power();
}

#else //!BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM

/**
 * Set the tx power
 *
 * @param[in] TX power register value
 *
 */
static void nlradio_set_tx_power_internal(int8_t power)
{
    nrf_802154_tx_power_set(power);
}

#endif// BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM

#ifdef BUILD_FEATURE_ANTENNA_DIVERSITY
/**
 * Internal API to calculate average ack RSSI for an antenna.
 * Called from tx compete ISR context.
 * @param[in] ack rssi.
 */
static void nlradio_update_avg_ack_rssi(int8_t rssi)
{
    int16_t new_rssi, old_average, new_average;
    antenna_e cur_antenna = nlproduct_get_current_antenna();

    nlREQUIRE(rssi != NLRADIO_INVALID_RSSI, done);

    assert(cur_antenna < ANTENNA_MAX);

    // Multiply the RSS value by a precision multiple (currently 8)
    new_rssi = rssi << NLRADIO_RSSI_PRECISON_SHIFT;

    // Get a snapshot of the current avg.
    old_average = s_antenna_stats.shifted_avg_ack_rssi[cur_antenna];

    if (NLRADIO_INVALID_RSSI_PRECISON_SHIFT == old_average)
    {
        // first time
        new_average = new_rssi;
    }
    else
    {
        // Maintain an exponentially weighted moving average using coefficient of (1/2^NLRADIO_RSSI_COEFF_SHIFT)
        // new_average = new_rssi * 1/2^j + old_average * (1 - 1/2^j), for j = NLRADIO_RSSI_COEFF_SHIFT
        new_average = (int16_t)(((old_average << NLRADIO_RSSI_COEFF_SHIFT) - old_average + new_rssi) >> NLRADIO_RSSI_COEFF_SHIFT);
    }

    // store the shifted rssi for future calculation
    s_antenna_stats.shifted_avg_ack_rssi[cur_antenna] = new_average;

done:
    return;
}

/**
 * Internal API to calculate the actual 8bit avg ack rssi from a precision shifted rssi value
 * @param[in] precicion shifted rssi value
 *
 * @retval rssi in 8bit format
 */
static int8_t nlradio_get_avg_ack_rssi(int16_t shifted_rssi)
{
    int8_t rssi = NLRADIO_INVALID_RSSI;

    nlREQUIRE(shifted_rssi != NLRADIO_INVALID_RSSI_PRECISON_SHIFT, done);

    rssi = (int8_t)(shifted_rssi >> NLRADIO_RSSI_PRECISON_SHIFT);

    // Check for possible round up (e.g., average of -71.5 --> -72)
    if ((shifted_rssi & NLRADIO_RSSI_PRECISON_MASK) >= (NLRADIO_PRECISON >> 1))
    {
        rssi--;
    }

done:
    return rssi;
}
#endif // BUILD_FEATURE_ANTENNA_DIVERSITY

/**
 * API to get the wpan antenna stats.
 * @param[in] pointer to a location to hold the stats counter.
 *
 * @retval k_nlradio_error_none   If antenna diversity is supported for the product
 * @retval k_nlradio_error_fail   If antenna diversity is not supported for the product
 */
int nlradio_get_wpan_antenna_stats(nlradio_wpan_antenna_stats_t *wpan_antenna_stats)
{
#ifdef BUILD_FEATURE_ANTENNA_DIVERSITY
    uint32_t count;

    assert(NULL != wpan_antenna_stats);

    for (count = 0 ; count < ANTENNA_MAX; count++)
    {
        wpan_antenna_stats->antenna_stats[count].tx_success_cnt = s_antenna_stats.antenna_stats[count].tx_success_cnt;
        wpan_antenna_stats->antenna_stats[count].tx_fail_cnt    = s_antenna_stats.antenna_stats[count].tx_fail_cnt;
        wpan_antenna_stats->antenna_stats[count].avg_ack_rssi   = nlradio_get_avg_ack_rssi(s_antenna_stats.shifted_avg_ack_rssi[count]);
    }

    wpan_antenna_stats->ant_switch_cnt = s_antenna_stats.ant_switch_cnt;

    return k_nlradio_error_none;
#else
    (void)wpan_antenna_stats;

    return k_nlradio_error_fail;
#endif
}

/**
 * Called when TX is complete with success or failure
 * @param[in]  nlradio_tx_error_t  TX success or failure
 *
 */
static void nlradio_tx_done(nlradio_tx_error_t tx_error)
{
#ifdef BUILD_FEATURE_ANTENNA_DIVERSITY
    if (!s_prevent_antenna_switch)
    {
        antenna_e cur_antenna = nlproduct_get_current_antenna();
        volatile nlradio_per_ant_stats_internal_t *cur_antenna_stats;

        assert(cur_antenna < ANTENNA_MAX);

        cur_antenna_stats = &s_antenna_stats.antenna_stats[cur_antenna];

        // Antenna switch logic
        if (tx_error == k_nlradio_tx_error_no_ack)
        {
            // Set bit in case of error over the air
            s_tx_failure_ota |= (1UL << s_tx_failure_ota_index);

            cur_antenna_stats->tx_fail_cnt++;
        }
        else if (tx_error == k_nlradio_tx_error_none)
        {
            // Clear bit in case of tx success
            s_tx_failure_ota &= ~(1UL << s_tx_failure_ota_index);

            cur_antenna_stats->tx_success_cnt++;
        }

        s_tx_failure_ota_index++;

        // Reset index when we reach the max number of bytes.
        // Start filling out from the beginning without erasing the rest
        // of the bytes to keep history.
        if (s_tx_failure_ota_index == NL_RADIO_NUM_BITS)
        {
            s_tx_failure_ota_index = 0;
        }

        // If we see more than 50% of the transmits fail, then switch antenna
        if (__builtin_popcount(s_tx_failure_ota) > NL_RADIO_FAILURE_THRESH)
        {
            // Switch antenna
            nlproduct_select_next_antenna();

            s_antenna_stats.ant_switch_cnt++;

            s_tx_failure_ota = 0;
            s_tx_failure_ota_index = 0;
        }
    }
#endif // BUILD_FEATURE_ANTENNA_DIVERSITY
}

/**
 * Mechanism to prevent/allow antenna switching
 *
 * @param[in]  prevent_switching     Indicates whether antenna switching should be on or off
 *
 */
#ifdef BUILD_FEATURE_ANTENNA_DIVERSITY
void nlradio_set_prevent_antenna_switch(bool prevent_switching)
{
    if (prevent_switching)
    {
        s_tx_failure_ota = 0;
        s_tx_failure_ota_index = 0;
    }

    s_prevent_antenna_switch = prevent_switching;
}
#endif // BUILD_FEATURE_ANTENNA_DIVERSITY

/**
 * Get the nest factory-assigned IEEE EUI-64 for this interface.
 *
 * @param[out] eui64_addr  A pointer to where the nest factory-assigned IEEE EUI-64 will be placed.
 *
 * @retval true   If the Extended Address get operation succeeded.
 * @retval false  If the Extended Address get operation failed.
 */
static bool nlradio_get_nest_ieee_eui64(uint8_t *eui64_addr)
{
    bool retval = false;
    char value[24]; // "xx:xx:xx:xx:xx:xx:xx:xx";
    size_t value_length = sizeof(value) / sizeof(value[0]);

    nlREQUIRE(nl_sysenv_get_string(k802154AddrKey, value, value_length) >= 0, done);

    nlREQUIRE(nl_getCharSeparatedBytes(value, eui64_addr, EXTENDED_ADDRESS_SIZE, ':', 16) == EXTENDED_ADDRESS_SIZE, done);

    retval = true;

done:
    return retval;
}

/**
 ****************
 * NL Radio API *
 ****************
 */

/**
 * Intialize the radio.
 *
 * @param[in] inContext           A pointer to a nlradio specific context
 *                                provided by nlradio client.
 *
 * @retval k_nlradio_error_none   If radio driver initialization succeeded.
 * @retval k_nlradio_error_fail   If radio driver initialization failed.
 */
int nlradio_init(void *inContext)
{
    check_single_task();

    nrf_802154_init();

#if defined(BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM)
    // default value set in nlproduct_config.h
    s_target_txpower = NLRADIO_DEFAULT_ACK_TX_POWER;
#else
    s_default_tx_power = NLRADIO_MAX_RADIO_TX_POWER;
#endif  // BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM

    return k_nlradio_error_none;
}

/**
 * Initialize and enable the radio, then transition to sleep state.
 *
 * @retval k_nlradio_error_none   If radio driver transitioned to sleep.
 * @retval k_nlradio_error_fail   If radio driver initialization failed.
 */
int nlradio_enable(void)
{
    int retval = k_nlradio_error_fail;

    check_single_task();

    if (s_radio_state == k_nlradio_state_disabled)
    {
        nlradio_init_internal();

        s_radio_state = k_nlradio_state_sleep;

        retval = k_nlradio_error_none;
    }

    return retval;
}

/**
 * Disable radio and transition to disabled.
 *
 * @retval k_nlradio_error_none   If transitioned to disabled state.
 * @retval k_nlradio_error_fail   If failed to disable radio.
 */
int nlradio_disable(void)
{
    int retval = k_nlradio_error_fail;

    check_single_task();

    if (s_radio_state == k_nlradio_state_sleep)
    {
        nlradio_init_internal();

        s_radio_state = k_nlradio_state_disabled;

        retval = k_nlradio_error_none;
    }

    return retval;
}

/**
 * Get the current state of the radio driver.
 *
 * @retval The current radio driver state, one of nlradio_state_t.
 */
nlradio_state_t nlradio_get_state(void)
{
    check_single_task();

    return s_radio_state;
}

/**
 * Set the PAN ID for address filtering.
 *
 * @param[in] pan_id  The IEEE 802.15.4 PAN ID.
 *
 * @retval k_nlradio_error_none  If the PAN ID set operation succeeded.
 * @retval k_nlradio_error_fail  If the PAN ID set operation failed.
 */
int nlradio_set_pan_id(uint16_t pan_id)
{
    receive_complete_cb callback;
    uint8_t id[PAN_ID_SIZE];

    check_single_task();

    nlradio_convert_short_address(id, pan_id);

    // setting the current callback to NULL
    // to ignore Rx packets until new pan id is set
    callback = s_rx_callback;
    s_rx_callback = NULL;

    nrf_802154_pan_id_set(id);

    // restore the rx callback
    s_rx_callback = callback;

    return k_nlradio_error_none;
}

/**
 * Set the default TX power, used for ACK.
 *
 * @param[in] aPower The TX power to use in dBm
 *
 * @retval k_nlradio_error_none  If the default TX power set operation succeeded.
 * @retval k_nlradio_error_fail  If the default TX power set operation failed.
 */
int nlradio_set_tx_power(int8_t aPower)
{
#if defined(BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM)
    //TODO: This API needs to be removed as soon as nlopenthread starts using the new target tx power API.
    (void)aPower;
#else
    check_single_task();

    s_default_tx_power = aPower;

    nlradio_set_tx_power_internal(s_default_tx_power);
#endif

    return k_nlradio_error_none;
}

/**
 * Set the target TX power to be used for packet transmission & acks
 *
 * @param[in] aPower The target TX power to use in units of 0.01 dBm
 *
 * @retval k_nlradio_error_none  If the default TX power set operation succeeded.
 * @retval k_nlradio_error_fail  If the default TX power set operation failed.
 */
int nlradio_set_target_txpower(int aPower)
{
#if defined(BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM)
    if (aPower == s_target_txpower)
    {
        return k_nlradio_error_none;
    }

    check_single_task();

    s_target_txpower = aPower;

    nlradio_set_transmit_power();
#else
    (void)aPower;
#endif

    return k_nlradio_error_none;
}

/**
 * Set the tx power backoff
 *
 * @param[in] backoff value in units of 0.01 dBm.
 *
 * @retval k_nlradio_error_none  If the power backoff set operation succeeded.
 * @retval k_nlradio_error_fail  If the power backoff set operation failed.
 */
int nlradio_set_txpower_backoff(int aBackoff)
{
#if defined(BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM)
    if (aBackoff == s_backoff_txpower)
    {
        return k_nlradio_error_none;
    }

    check_single_task();

    s_backoff_txpower = aBackoff;

    nlradio_set_transmit_power();
#else
    (void)aBackoff;
#endif

    return k_nlradio_error_none;
}

/**
 * Set the Extended Address for address filtering.
 *
 * @param[in] extended_address  A pointer of 8 bytes containing the IEEE 802.15.4 Extended Address.
 *
 * @retval k_nlradio_error_none  If the Extended Address set operation succeeded.
 * @retval k_nlradio_error_fail  If the Extended Address set operation failed.
 */
int nlradio_set_extended_address(const uint8_t *extended_address)
{
    receive_complete_cb callback;

    check_single_task();

    // setting the current callback to NULL
    // to ignore Rx packets until new extended address is set
    callback = s_rx_callback;
    s_rx_callback = NULL;

    nrf_802154_extended_address_set(extended_address);

    // restore the rx callback
    s_rx_callback = callback;

    return k_nlradio_error_none;
}

/**
 * Set the Short Address for address filtering.
 *
 * @param[in] short_address  The IEEE 802.15.4 Short Address.
 *
 * @retval k_nlradio_error_none  If the Short Address set operation succeeded.
 * @retval k_nlradio_error_fail  If the Short Address set operation failed.
 */
int nlradio_set_short_address(uint16_t short_address)
{
    receive_complete_cb callback;
    uint8_t address[SHORT_ADDRESS_SIZE];

    check_single_task();

    nlradio_convert_short_address(address, short_address);

    // setting the current callback to NULL
    // to ignore Rx packets until new short address is set
    callback = s_rx_callback;
    s_rx_callback = NULL;

    nrf_802154_short_address_set(address);

    // restore the rx callback
    s_rx_callback = callback;

    return k_nlradio_error_none;
}

/**
 * Put the radio to sleep and the driver into the sleep state.
 *
 * @retval k_nlradio_error_none  If the radio was successfully put in sleep mode.
 * @retval k_nlradio_error_fail  If the function failed.
 */
int nlradio_sleep(void)
{
    int retval;

    check_single_task();

    switch (s_radio_state)
    {
        case k_nlradio_state_receive:
            s_rx_callback = NULL;
            nlradio_sleep_internal();
            nlradio_rx_packet_init();
            // intentional fall-thru
        case k_nlradio_state_disabled:
            s_radio_state = k_nlradio_state_sleep;
            // intentional fall-thru
        case k_nlradio_state_sleep:
            retval = k_nlradio_error_none;
            break;
        case k_nlradio_state_energy_scan:
        default:
            retval = k_nlradio_error_fail;
            break;
    }

    return retval;
}

/**
 * Transmit the provided buffer on the specified channel.
 *
 * @note
 *    Upon return from this function the buffer can be recycled.
 *    A successful return indicates that the radio driver has transitioned
 *    into the k_nlradio_state_transmit state. The radio will remain in this state
 *    until the transmit operation completes upon which it will execute the optional
 *    callback (tx_complete_cb) if provided.
 *    Both tx_complete_cb and tx_start_cb are executed by the ISR and must therefore be
 *    ISR safe.
 *
 * @param[in] buffer          A buffer containing length bytes for transmission.
 * @param[in] length          The number of bytes in buffer to transmit.
 * @param[in] channel         The radio channel upon which the transmission shall occur.
 * @param[in] power           The transmit power for transmission (dBm).
 * @param[in] tx_complete_cb  An optional callback that will be called when the transmit completes. Can be NULL.
 * @param[in] tx_start_cb     An optional callback that will be called when the transmit starts. Can be NULL.
 *
 * @retval k_nlradio_error_none  If the transmission was started successfully and the radio transitioned to the k_nlradio_state_transmit.
 * @retval k_nlradio_error_fail  If the transmission failed to start.
 */
#ifdef BUILD_FEATURE_RADIO_HEADER_IE
int nlradio_transmit(nlradio_tx_params_t *radio_tx_params, transmit_complete_cb tx_complete_cb, transmit_start_cb tx_start_cb)
#else
int nlradio_transmit(nlradio_tx_params_t *radio_tx_params, transmit_complete_cb tx_complete_cb)
#endif
{
    int retval = k_nlradio_error_fail;

    check_single_task();

    if (s_radio_state == k_nlradio_state_receive)
    {
        // copy the tx packet information into the local tx packet structure
        s_tx_packet.length = radio_tx_params->length;
        memcpy(s_tx_packet.psdu, radio_tx_params->buffer, radio_tx_params->length);

        // store the optional caller supplied transmit callback for use in handle_transmit_isr().
        s_tx_complete_callback = tx_complete_cb;
#ifdef BUILD_FEATURE_RADIO_HEADER_IE
        s_tx_start_callback = tx_start_cb;
#endif

#if defined(BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM)
        if (radio_tx_params->channel != s_channel)
        {
            nlradio_set_channel_internal(radio_tx_params->channel);
        }
#else
        s_channel = radio_tx_params->channel;

        nrf_802154_channel_set(s_channel);

        nlradio_set_tx_power_internal(radio_tx_params->power);
#endif //BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM

        // change the radio state to indicate that it is now in the transmit state.
        s_radio_state = k_nlradio_state_transmit;

        if (radio_tx_params->is_cca_enabled)
        {
            nrf_802154_transmit_csma_ca_raw(&s_tx_packet.length);

            retval = k_nlradio_error_none; // indicate success to the caller.
        }
        else
        {
            uint16_t msTick = NLRADIO_TX_WAIT_MS;

            while (!nrf_802154_transmit_raw(&s_tx_packet.length, radio_tx_params->is_cca_enabled) && --msTick)
            {
                nlplatform_delay_ms(1);
            }

            if (msTick)
            {
                retval = k_nlradio_error_none; // indicate success to the caller.
            }
        }
    }

    return retval;
}

/**
 * Transition the radio to receive and execute the callback when complete.
 *
 * @note
 *    A successful return indicates that the radio driver has transitioned
 *    into the k_nlradio_state_receive state. The radio will remain in this state
 *    until the receive operation completes upon which it will execute the callback
 *    (cb).  The callback is executed by the ISR and must therefore be
 *    ISR safe.
 *
 * @param[in] channel       The radio channel upon which the receive operation shall occur.
 * @param[in] cb            A callback that will be called when the receive completes.
 *
 * @retval k_nlradio_error_none  If the reception was started successfully and the radio transitioned to the k_nlradio_state_receive.
 * @retval k_nlradio_error_fail  If the reception failed to start.
 */
int nlradio_receive(uint8_t channel, receive_complete_cb cb)
{
    int retval = k_nlradio_error_fail;

    check_single_task();

    if (s_radio_state == k_nlradio_state_sleep || // configure the radio for receive.
        (s_radio_state == k_nlradio_state_receive && (channel != s_channel || cb != s_rx_callback))) // change the receive channel or callback
    {
        if (s_radio_state == k_nlradio_state_sleep)
        {
            nlradio_wakeup_internal();
        }

        // setting the current callback to NULL
        // to ignore Rx packets
        s_rx_callback = NULL;

        // change the radio state to indicate that it is now in the receive state.
        s_radio_state = k_nlradio_state_receive;

#if defined(BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM)
        nlradio_set_channel_internal(channel);
#else
        s_channel = channel;

        nrf_802154_channel_set(s_channel);
        nlradio_set_tx_power_internal(s_default_tx_power);
#endif //BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM

        nrf_802154_receive();

        s_rx_callback = cb;

        retval = k_nlradio_error_none; // indicate success back to the caller.
    }
    else if (s_radio_state == k_nlradio_state_receive)
    {
        retval = k_nlradio_error_none; // indicate success back to the caller.
    }

    return retval;
}

/**
 * Read radio RSSI.
 *
 * @note
 *    RSSI should be read when there are no detectable ongoing
 *    15.4 packet transmissions.
 *
 * @param[out] rssi                A pointer to hold the RSSI value.
 *
 * @retval k_nlradio_error_none    Successfully read RSSI.
 * @retval k_nlradio_error_fail    Radio was busy, could not read RSSI.
 */
int nlradio_get_rssi(int8_t *rssi)
{
    check_single_task();

    *rssi = nrf_802154_rssi_last_get();

    return k_nlradio_error_none;
}

/**
 * Called to perform post-processing of a previously received frame.
 *
 * @note
 *    The caller is expected to call this API only after having been notified
 *    by execution of the receive_complete_cb that was passed into nlradio_receive().
 *
 * @param[out] buffer        A buffer pointer to hold the received frame on completion.
 * @param[out] num_bytes     The number of bytes used in buffer to store the RX frame.
 * @param[out] channel       The channel the radio was on when it received the frame.
 * @param[out] power         The signal strength (RSSI) of the received frame.
 * @param[out] lqi           The link quality indicator for the received frame.
 * @param[out] timestamp     The time since boot when the radio began to receive the frame (rx sfd), in microseconds.
 *
 * @retval k_nlradio_error_none  If the driver specific operation completed successfully.
 * @retval k_nlradio_error_fail  If the driver specific operation failed.
 */
#ifdef BUILD_FEATURE_RADIO_HEADER_IE
int nlradio_post_process_receive(uint8_t **buffer, uint32_t *num_bytes, uint8_t *channel, int8_t *power, uint8_t *lqi, uint64_t *timestamp)
#else
int nlradio_post_process_receive(uint8_t **buffer, uint32_t *num_bytes, uint8_t *channel, int8_t *power, uint8_t *lqi)
#endif
{
    int retval = k_nlradio_error_fail;
    nlradio_packet_t *p;

    check_single_task();

    if (buffer != NULL && num_bytes != NULL)
    {
        if ((p = nlradio_rx_packet_list_dequeue()) != NULL)
        {
            *buffer = p->psdu;
            *num_bytes = p->length;
            *channel = p->channel;
            *power = p->power;
            *lqi = p->lqi;
#ifdef BUILD_FEATURE_RADIO_HEADER_IE
            *timestamp = p->timestamp;
#endif

            retval = k_nlradio_error_none;
        }
    }

    return retval;
}

/**
 * Called to free a buffer received in nlradio_post_process_receive.
 *
 * @note
 *    The caller is expected to call this API when it is ready to free a buffer it
 *    acquired from nlradio_post_process_receive.
 *
 * @param[in] buffer        A buffer to be freed.
 *
 * @retval k_nlradio_error_none  If the driver specific operation completed successfully.
 * @retval k_nlradio_error_fail  If the driver specific operation failed.
 */
int nlradio_buffer_free(uint8_t *buffer)
{
    int retval = k_nlradio_rx_error_none;

    check_single_task();

    if (!nlradio_rx_packet_free(buffer))
    {
        retval = k_nlradio_error_fail;
    }

    return retval;
}

/**
 * Returns the capabilities of the radio.
 *
 * @retval A bitfield consisting of zero or more nlradio_capabilities_t values.
 */
nlradio_capabilities_t nlradio_get_capabilities(void)
{
    check_single_task();

    // use OT energy scan for now so don't include k_nlradio_capability_energy_scan
    return k_nlradio_capability_ack_timeout | k_nlradio_capability_csma_backoff;
}

/**
 * Override source address matching and set frame pending
 * for all short and extended address data polls.
 *
 * @param[in] enable    true, to set frame pending for all acks
 *                      false, to use the frame pending source address match tables
 *                             to determine whether to set frame pending in ack
 *
 * @retval k_nlradio_error_none  If the override source match operation succeeded.
 * @retval k_nlradio_error_fail  If the override source match operation failed.
 */
int nlradio_override_source_match(bool enable)
{
    check_single_task();

    nrf_802154_auto_pending_bit_set(enable);

    return k_nlradio_error_none;
}

/**
 * Clear all Extended Address source match entries.
 *
 * @retval k_nlradio_error_none  If the clear all extended address match operation succeeded.
 * @retval k_nlradio_error_fail  If the clear all extended address match operation failed.
 */
int nlradio_clear_extended_source_match_address_entries(void)
{
    const bool address_is_extended = true;

    check_single_task();

    nrf_802154_pending_bit_for_addr_reset(address_is_extended);

    return k_nlradio_error_none;
}

/**
 * Set/Clear the Extended Address source match entry.
 *
 * @param[in] extended_address  A pointer of 8 bytes containing the IEEE 802.15.4 Extended Address.
 * @param[in] pending           true, to add this extended address to the source match table,
 *                              false, to remove this extended address from the source match table
 *
 * @retval k_nlradio_error_none  If the set extended address match operation succeeded.
 * @retval k_nlradio_error_fail  If the address could not be added to the table when pending == true
 *                               or if the address was not found when pending == false
 */
int nlradio_set_extended_source_match_address_entry(const uint8_t *extended_address, bool pending)
{
    int retval = k_nlradio_error_fail;
    const bool address_is_extended = true;

    check_single_task();

    if (pending)
    {
        if (nrf_802154_pending_bit_for_addr_set(extended_address, address_is_extended))
        {
            retval = k_nlradio_error_none;
        }
    }
    else
    {
        if (nrf_802154_pending_bit_for_addr_clear(extended_address, address_is_extended))
        {
            retval = k_nlradio_error_none;
        }
    }

    return retval;
}

/**
 * Clear all Short Address source match entries.
 *
 * @retval k_nlradio_error_none  If the clear all short address match operation succeeded.
 * @retval k_nlradio_error_fail  If the clear all short address match operation failed.
 */
int nlradio_clear_short_source_match_address_entries(void)
{
    const bool address_is_extended = false;

    check_single_task();

    nrf_802154_pending_bit_for_addr_reset(address_is_extended);

    return k_nlradio_error_none;
}

/**
 * Set/Clear the Short Address source match entry.
 *
 * @param[in] short_address  The IEEE 802.15.4 Short Address.
 * @param[in] pending           true, to add this short address to the source match table,
 *                              false, to remove this short address from the source match table
 *
 * @retval k_nlradio_error_none  If the set short address match operation succeeded.
 * @retval k_nlradio_error_fail  If the address could not be added to the table when pending == true
 *                               or if the address was not found when pending == false
 */
int nlradio_set_short_source_match_address_entry(uint16_t short_address, bool pending)
{
    int retval = k_nlradio_error_fail;
    uint8_t address[SHORT_ADDRESS_SIZE];
    const bool address_is_extended = false;

    check_single_task();

    nlradio_convert_short_address(address, short_address);

    if (pending)
    {
        if (nrf_802154_pending_bit_for_addr_set(address, address_is_extended))
        {
            retval = k_nlradio_error_none;
        }
    }
    else
    {
        if (nrf_802154_pending_bit_for_addr_clear(address, address_is_extended))
        {
            retval = k_nlradio_error_none;
        }
    }

    return retval;
}

/**
 * Get the radio filter mode.
 *
 * @retval The current filter mode.
 */
nlradio_filter_mode_t nlradio_get_filter_mode(void)
{
    nlradio_filter_mode_t filter_mode = k_nlradio_filter_mode_normal;

    check_single_task();

    if (nrf_802154_promiscuous_get())
    {
        filter_mode = k_nlradio_filter_mode_full_promiscuous;
    }

    return filter_mode;
}

/**
 * Set the radio filter mode to either normal (destination address filtering),
 * promiscuous (all to a given panid), or monitor mode (all on given channel).
 *
 * @param[in] filter_mode       The radio filter mode.
 *
 * @retval k_nlradio_error_none  If the set filter mode operation succeeded.
 * @retval k_nlradio_error_fail  If the given filter mode could not be set.
 */
int nlradio_set_filter_mode(nlradio_filter_mode_t filter_mode)
{
    bool enable;

    check_single_task();

    switch(filter_mode)
    {
        case k_nlradio_filter_mode_network_promiscuous:
        case k_nlradio_filter_mode_full_promiscuous:
            enable = true;
            break;

        case k_nlradio_filter_mode_normal:
        default:
            enable = false;
            break;
    }

    nrf_802154_promiscuous_set(enable);

    return k_nlradio_error_none;
}

/**
 * Initiate an energy scan on the specified channel for the specified duration.
 *
 * @param[in] channel       The radio channel upon which the energy scan operation shall occur.
 * @param[in] duration_msec The duration in milliseconds for the energy scan.
 * @param[in] cb            A callback that will be called when the energy scan completes.
 *
 * @retval k_nlradio_error_none  If the start energy scan operation succeeded.
 * @retval k_nlradio_error_fail  If the start energy scan operation failed.
 */
int nlradio_start_energy_scan(uint8_t channel, uint32_t duration_msec, escan_complete_cb cb)
{
    int retval = k_nlradio_error_fail;

    check_single_task();

    if (s_radio_state == k_nlradio_state_sleep || s_radio_state == k_nlradio_state_receive)
    {
        uint16_t msTick = NLRADIO_ESCAN_WAIT_MS;

        if (s_radio_state == k_nlradio_state_sleep)
        {
            nlradio_wakeup_internal();
        }

        s_pre_escan_radio_state = s_radio_state;

        s_escan_callback = cb;

        s_radio_state = k_nlradio_state_energy_scan;

        nrf_802154_channel_set(channel);

        while (!nrf_802154_energy_detection(duration_msec) && --msTick)
        {
            nlplatform_delay_ms(1);
        }

        if (msTick)
        {
            retval = k_nlradio_error_none;
        }
        else
        {
            s_radio_state = s_pre_escan_radio_state;
        }
    }

    return retval;
}

/**
 * Get the factory-assigned IEEE EUI-64 for this interface.
 *
 * @param[in]  aInstance             The OpenThread instance structure.
 * @param[out] aIeeeEui64            A pointer to where the factory-assigned IEEE EUI-64 will be placed.
 *
 * @retval k_nlradio_error_none  If the Extended Address set operation succeeded.
 * @retval k_nlradio_error_fail  If the Extended Address set operation failed.
 */
int nlradio_get_ieee_eui64(uint8_t *aIeeeEui64)
{
    check_single_task();

    if (!nlradio_get_nest_ieee_eui64(aIeeeEui64))
    {
        uint64_t factoryAddress = (uint64_t)NRF_FICR->DEVICEID[0] << 32;

        factoryAddress |=  NRF_FICR->DEVICEID[1];

        memcpy(aIeeeEui64, &factoryAddress, sizeof(factoryAddress));
    }

    return k_nlradio_error_none;
}

/**
 * Get radio receive sensitivity.
 *
 * @retval The radio receive sensitivity value in dBm.
 */
int nlradio_get_rx_sensitivity(void)
{
    check_single_task();

    return NLRADIO_RECEIVE_SENSITIVITY;
}


/**
 ***************************************************
 * Nordic Radio Driver ISR Handler function calls. *
 ***************************************************
 */

/**
 * Notify that frame was received.
 *
 * @note Buffer pointed by the p_data pointer is not modified by the radio driver (and can't
 *       be used to receive a frame) until nrf_802154_buffer_free_raw() function is called.
 * @note Buffer pointed by the p_data pointer may be modified by the function handler (and other
 *       modules) until nrf_802154_buffer_free_raw() function is called.
 *
 * p_data
 * v
 * +-----+-----------------------------------------------------------+------------+
 * | PHR | MAC Header and payload                                    | FCS        |
 * +-----+-----------------------------------------------------------+------------+
 *       |                                                                        |
 *       | <---------------------------- PHR -----------------------------------> |
 *
 * @param[in]  p_data  Pointer to the buffer containing received data (PHR + PSDU). First byte in
 *                     the buffer is length of the frame (PHR) and following bytes is the frame
 *                     itself (PSDU). Length byte (PHR) includes FCS. FCS is already verified by
 *                     the hardware and may be modified by the hardware.
 * @param[in]  power   RSSI of received frame.
 * @param[in]  lqi     LQI of received frame.
 */
void nrf_802154_received_raw(uint8_t *p_data, int8_t power, uint8_t lqi)
{
    if (s_radio_state == k_nlradio_state_receive || s_radio_state == k_nlradio_state_transmit)
    {
        if (s_rx_callback != NULL)
        {
            nlradio_rx_error_t rx_error = k_nlradio_rx_error_no_buffer;
            bool all_buffers_used;
            nlradio_packet_t *p;
            const bool from_isr = true;

            if ((p = nlradio_rx_packet_alloc_isr(&all_buffers_used)) != NULL)
            {
                if (all_buffers_used)
                {
                    /* If there is no more rx buffer in the pool, disable the
                     * auto-ack so that radio does not ack any future
                     * received frame.
                     */

                    nrf_802154_auto_ack_set(false);
                }

                rx_error = k_nlradio_rx_error_none;

                p->length = p_data[0];
                p->power = power;
                p->lqi = lqi;
                memcpy(p->psdu, &p_data[1], p->length);
                p->channel = nrf_802154_channel_get();
#ifdef BUILD_FEATURE_RADIO_HEADER_IE
                uint64_t ota_time = (PHR_SIZE + p->length) * PHY_SYMBOLS_PER_OCTET * PHY_US_PER_SYMBOL;
                p->timestamp = nltime_get_system_us() - ota_time;
#endif

                nlradio_rx_packet_list_enqueue_isr(p);
            }

            s_rx_callback(rx_error, from_isr);
        }
    }

    if (s_in_radio_isr)
    {
        nrf_802154_buffer_free_raw(p_data);
    }
    else
    {
        nrf_802154_core_notify_buffer_free(p_data);
    }
}

/**
 * Notify that reception of a frame failed.
 *
 * @param[in]  error  An error code that indicates reason of the failure.
 */
void nrf_802154_receive_failed(nrf_802154_rx_error_t error)
{
    switch (error)
    {
        case NRF_802154_RX_ERROR_INVALID_FRAME:
        case NRF_802154_RX_ERROR_INVALID_FCS:
        case NRF_802154_RX_ERROR_INVALID_DEST_ADDR:
        case NRF_802154_RX_ERROR_RUNTIME:
        case NRF_802154_RX_ERROR_TIMESLOT_ENDED:
        default:
            // TBD - determine which of these errors should be reported to nlopenthread
            // s_rx_callback(k_nlradio_rx_error_cancelled, true);
            break;
    }
}

/**
 * Notify that frame was transmitted.
 *
 * @note If ACK was requested for transmitted frame this function is called after proper ACK is
 *       received. If ACK was not requested this function is called just after transmission is
 *       ended.
 * @note Buffer pointed by the p_ack pointer is not modified by the radio driver (and can't
 *       be used to receive a frame) until nrf_802154_buffer_free_raw() function is
 *       called.
 * @note Buffer pointed by the p_ack pointer may be modified by the function handler (and other
 *       modules) until nrf_802154_buffer_free_raw() function is called.
 *
 * @param[in]  p_frame  Pointer to buffer containing PHR and PSDU of transmitted frame.
 * @param[in]  p_ack    Pointer to received ACK buffer. Fist byte in the buffer is length of the
 *                      frame (PHR) and following bytes are the ACK frame itself (PSDU). Length byte
 *                      (PHR) includes FCS. FCS is already verified by the hardware and may be
 *                      modified by the hardware.
 *                      If ACK was not requested p_ack is set to NULL.
 * @param[in]  power    RSSI of received frame or 0 if ACK was not requested.
 * @param[in]  lqi      LQI of received frame or 0 if ACK was not requested.
 */
void nrf_802154_transmitted_raw(const uint8_t *p_frame, uint8_t *p_ack, int8_t power, uint8_t lqi)
{
    transmit_complete_cb callback;

    // return the radio state back to receive
    s_radio_state = k_nlradio_state_receive;

#ifndef BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM
    // return the radio tx power to the default tx power used for acks
    nlradio_set_tx_power_internal(s_default_tx_power);
#endif //!BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM

    callback = s_tx_complete_callback;
    s_tx_complete_callback = NULL;

    if (callback)
    {
        bool frame_pending;
        const bool from_isr = true;

        if (p_ack == NULL)
        {
            frame_pending = false;
        }
        else
        {
            frame_pending = (p_ack[FRAME_PENDING_OFFSET] & FRAME_PENDING_BIT) ? true : false;
        }

        nlradio_tx_done(k_nlradio_tx_error_none);

#ifdef BUILD_FEATURE_ANTENNA_DIVERSITY
        nlradio_update_avg_ack_rssi(power);
#endif

        callback(k_nlradio_tx_error_none, frame_pending, power, lqi, from_isr);
    }

    if (p_ack != NULL)
    {
        nrf_802154_buffer_free_raw(p_ack);
    }

    (void)p_frame;
}

/**
 * Notify that frame was not transmitted due to busy channel.
 *
 * This function is called if transmission procedure fails.
 *
 * @param[in]  p_frame  Pointer to buffer containing PSDU of frame that was not transmitted.
 * @param[in]  error    Reason of the failure.
 */
void nrf_802154_transmit_failed(const uint8_t *p_frame, nrf_802154_tx_error_t error)
{
    transmit_complete_cb callback;

    // return the radio state back to receive
    s_radio_state = k_nlradio_state_receive;

#ifndef BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM
    // return the radio tx power to the default tx power used for acks
    nlradio_set_tx_power_internal(s_default_tx_power);
#endif //!BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM

    callback = s_tx_complete_callback;
    s_tx_complete_callback = NULL;

    if (callback)
    {
        nlradio_tx_error_t tx_error;
        const bool frame_pending = false;
        const bool from_isr = true;

        switch (error)
        {
            case NRF_802154_TX_ERROR_BUSY_CHANNEL:
            case NRF_802154_TX_ERROR_TIMESLOT_ENDED:
                tx_error = k_nlradio_tx_error_channel_busy;
                break;

            case NRF_802154_TX_ERROR_INVALID_ACK:
                tx_error = k_nlradio_tx_error_no_ack;
                break;

            case NRF_802154_TX_ERROR_NO_ACK:
                tx_error = k_nlradio_tx_error_no_ack;
                break;

            case NRF_802154_TX_ERROR_NO_MEM:
            default:
                tx_error = k_nlradio_tx_error_platform;
                break;
        }

        nlradio_tx_done(tx_error);
        callback(tx_error, frame_pending, kRadioRssiUnknown, 0, from_isr);
    }

    (void)p_frame;
}

/**
 * Notify that Energy Detection procedure finished.
 *
 * @note This function passes EnergyLevel defined in 802.15.4-2006 specification:
 *       0x00 - 0xff proportional to detected energy level (dBm above receiver sensitivity). To
 *       calculate result in dBm use nrf_802154_dbm_from_energy_level_calculate().
 *
 * @param[in]  result  Maximum energy detected during Energy Detection procedure.
 */
void nrf_802154_energy_detected(uint8_t result)
{
    if (s_radio_state == k_nlradio_state_energy_scan)
    {
        if (s_pre_escan_radio_state == k_nlradio_state_receive)
        {
            receive_complete_cb cb = s_rx_callback;

            s_rx_callback = NULL;

            s_radio_state = k_nlradio_state_receive;

            nrf_802154_channel_set(s_channel);

#if defined(BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM)
            nlradio_set_transmit_power();
#else
            nlradio_set_tx_power_internal(s_default_tx_power);
#endif  //BUILD_FEATURE_PRODUCT_HAS_RADIO_FEM

            nrf_802154_receive();

            s_rx_callback = cb;
        }
        else
        {
            nlradio_sleep_internal();

            s_radio_state = k_nlradio_state_sleep;
        }

        if (s_escan_callback)
        {
            const bool from_isr = true;

            s_escan_callback(nrf_802154_dbm_from_energy_level_calculate(result), from_isr);
        }
    }
}

#ifdef BUILD_FEATURE_RADIO_HEADER_IE
/**
 * @brief Notify that transmitting a frame has started.
 *
 * @note Usually, @ref nrf_802154_transmitted is called shortly after this function.
 *       However, if the transmit procedure is interrupted, it might happen that
 *       @ref nrf_802154_transmitted is not called.
 * @note This function should be very short to prevent dropping frames by the driver.
 *
 * @param[in]  p_frame  Pointer to the buffer containing PSDU of the frame being transmitted.
 */
void nrf_802154_tx_started(const uint8_t *aFrame)
{
    if (s_tx_start_callback)
    {
        const bool from_isr = true;

        s_tx_start_callback(s_tx_packet.psdu, from_isr);
        s_tx_start_callback = NULL;
    }
}
#endif

/**
 * Radio ISR
 *
 * Prototype is included here because this is an ISR handler and there is a symbol replacement
 * done by the linker (Radio_IRQHandler= nlradio_isr) so the prototype for nlradio_isr does not
 * exist elsewhere. The compiler will warn of a missing prototype otherwise which will be
 * interpreted as an error for our builds.
 */
void nlradio_isr(void);
void nlradio_isr(void)
{
    s_in_radio_isr = true;
    nrf_802154_radio_irq_handler();
    s_in_radio_isr = false;
}

