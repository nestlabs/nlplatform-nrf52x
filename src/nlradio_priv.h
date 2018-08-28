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

#include <inttypes.h>

#ifndef _NLRADIO_PRIV_H_
#define _NLRADIO_PRIV_H_

#define PAN_ID_SIZE           2    // Size of Pan Id
#define SHORT_ADDRESS_SIZE    2    // Size of Short Mac Address
#define EXTENDED_ADDRESS_SIZE 8    // Size of Extended Mac Address
#define MAX_PACKET_SIZE       127  // Maximal size of radio packet

#define BROADCAST_ADDRESS    ((uint8_t [SHORT_ADDRESS_SIZE]) {0xff, 0xff}) // Broadcast Short Address

/**
 * Header of a IEEE 802.15.4 frame
 */
typedef struct ieee802154_mac_fcf_s
{
    uint16_t frame_type:3;
    uint16_t security_enabled:1;
    uint16_t frame_pending:1;
    uint16_t ack_requested:1;
    uint16_t pan_id_compression:1;
    uint16_t reserved:3;
    uint16_t dst_addr_mode:2;
    uint16_t frame_version:2;
    uint16_t src_addr_mode:2;
} ieee802154_mac_fcf_t; /* 2 bytes */

typedef struct mac_dst_none_src_short_addr_s
{
    uint8_t src_pan_id[PAN_ID_SIZE];
    uint8_t src_addr[SHORT_ADDRESS_SIZE];
} __attribute__((packed)) mac_dst_none_src_short_addr_t;

typedef struct mac_dst_none_src_extended_addr_s
{
    uint8_t src_pan_id[PAN_ID_SIZE];
    uint8_t src_addr[EXTENDED_ADDRESS_SIZE];
} __attribute__((packed)) mac_dst_none_src_extended_addr_t;

typedef struct mac_dst_short_src_none_addr_s
{
    uint8_t dst_pan_id[PAN_ID_SIZE];
    uint8_t dst_addr[SHORT_ADDRESS_SIZE];
} __attribute__((packed)) mac_dst_short_src_none_addr_t;

typedef struct mac_dst_short_src_short_addr_s
{
    uint8_t dst_pan_id[PAN_ID_SIZE];
    uint8_t dst_addr[SHORT_ADDRESS_SIZE];
    uint8_t src_pan_id[PAN_ID_SIZE];
    uint8_t src_addr[SHORT_ADDRESS_SIZE];
} __attribute__((packed)) mac_dst_short_src_short_addr_t;

typedef struct mac_dst_short_src_extended_addr_s
{
    uint8_t dst_pan_id[PAN_ID_SIZE];
    uint8_t dst_addr[SHORT_ADDRESS_SIZE];
    uint8_t src_pan_id[PAN_ID_SIZE];
    uint8_t src_addr[EXTENDED_ADDRESS_SIZE];
} __attribute__((packed)) mac_dst_short_src_extended_addr_t;

typedef struct mac_dst_extended_src_none_addr_s
{
    uint8_t dst_pan_id[PAN_ID_SIZE];
    uint8_t dst_addr[EXTENDED_ADDRESS_SIZE];
} __attribute__((packed)) mac_dst_extended_src_none_addr_t;

typedef struct mac_dst_extended_src_short_addr_s
{
    uint8_t dst_pan_id[PAN_ID_SIZE];
    uint8_t dst_addr[EXTENDED_ADDRESS_SIZE];
    uint8_t src_pan_id[PAN_ID_SIZE];
    uint8_t src_addr[SHORT_ADDRESS_SIZE];
} __attribute__((packed)) mac_dst_extended_src_short_addr_t;

typedef struct mac_dst_extended_src_extended_addr_s
{
    uint8_t dst_pan_id[PAN_ID_SIZE];
    uint8_t dst_addr[EXTENDED_ADDRESS_SIZE];
    uint8_t src_pan_id[PAN_ID_SIZE];
    uint8_t src_addr[EXTENDED_ADDRESS_SIZE];
} __attribute__((packed)) mac_dst_extended_src_extended_addr_t;

typedef struct ieee802154_mac_header_s
{
    ieee802154_mac_fcf_t fcf;
    uint8_t seq_num;
    union
    {
        mac_dst_none_src_short_addr_t        dst_none_src_short_addresses;
        mac_dst_none_src_extended_addr_t     dst_none_src_extended_addresses;
        mac_dst_short_src_none_addr_t        dst_short_src_none_addresses;
        mac_dst_short_src_short_addr_t       dst_short_src_short_addresses;
        mac_dst_short_src_extended_addr_t    dst_short_src_extended_addresses;
        mac_dst_extended_src_none_addr_t     dst_extended_src_none_addresses;
        mac_dst_extended_src_short_addr_t    dst_extended_src_short_addresses;
        mac_dst_extended_src_extended_addr_t dst_extended_src_extended_addresses;
    };
} __attribute__((packed)) ieee802154_mac_header_t;

/**
 * This structure contains an IEEE 802.15.4 radio frame data.
 */
typedef struct ieee802154_packet_s
{
    uint8_t length;                    ///< Length of the PSDU.
    union {
        uint8_t psdu[MAX_PACKET_SIZE]; ///< The PSDU.
        ieee802154_mac_header_t mpdu;
    };
} __attribute__((packed)) ieee802154_packet_t;

/* Possible values for fcf.frame_type */
#define FRAME_TYPE_BEACON            0x00      // Bits containing Beacon frame type
#define FRAME_TYPE_DATA              0x01      // Bits containing Data frame type
#define FRAME_TYPE_ACK               0x02      // Bits containing ACK frame type
#define FRAME_TYPE_COMMAND           0x03      // Bits containing Command frame type

/* Possible values for dst_addr_mode/src_addr_mode: */
#define SHORT_ADDRESSING_MODE    0x2
#define EXTENDED_ADDRESSING_MODE 0x3

#endif // _NLRADIO_PRIV_H_
