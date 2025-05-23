/*
 * Copyright 2025 NXP
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __HAL_UCI_PROTOCOL_H
#define __HAL_UCI_PROTOCOL_H

#include <errno.h>
#include <uci_packets.h>

#ifdef __cplusplus
extern "C" {
#endif

// Not defined in uci_packets.pdl
#define TX_ADAPTIVE_PAYLOAD_POWER 0x7F
#define RESPONDER_SLOT_INDEX 0xA2

#define APP_CONFIG_TLV_DECL(c, value, type, tlv, tlvs_count)                                                           \
    type c##_v = value;                                                                                                \
    tlv[tlvs_count].cfg_id = c;                                                                                        \
    tlv[tlvs_count].v = (uint8_t *)&c##_v;                                                                             \
    tlv[tlvs_count].v_count = sizeof(type);                                                                            \
    tlvs_count++;

#define APP_CONFIG_TLV_PTR(c, value, type, tlv, tlvs_count)                                                            \
    tlv[tlvs_count].cfg_id = c;                                                                                        \
    tlv[tlvs_count].v = (uint8_t *)value;                                                                              \
    tlv[tlvs_count].v_count = sizeof(type);                                                                            \
    tlvs_count++;

#define APP_CONFIG_TLV_ARRAY(c, value, count, tlv, tlvs_count)                                                         \
    tlv[tlvs_count].cfg_id = c;                                                                                        \
    tlv[tlvs_count].v = (uint8_t *)value;                                                                              \
    tlv[tlvs_count].v_count = count;                                                                                   \
    tlvs_count++;

#define APP_CONFIG_TLV(c) c##_tlv

typedef struct UciPacketDecode {
    GroupId gid;
    MessageType mt;
    uint8_t oid;
    int ret;
    Status status;
} UciPacketDecode;

/**
 * @brief Prints the contents of a UCI payload to stdout.
 *
 * This function takes a UCI payload and its length as input, deserializes the payload, and prints the contents to
 *stdout for debugging purposes.
 *
 * @param payload Pointer to the UCI payload.
 * @param length Length of the payload.
 * @return int Status code indicating the result of the operation:
 *         - 0: Success
 *         - EINVAL: Parsing error
 *         - ENOTSUP: Not implemented
 */
int uci_print(const uint8_t *payload, size_t length);

/**
 * @brief Decodes a UCI payload into a UciPacketDecode structure.
 *
 * This function takes a UCI payload and its size as input, decodes the payload, and populates a UciPacketDecode
 * structure with the decoded information.
 *
 * @param payload Pointer to the UCI payload.
 * @param size Size of the payload.
 * @return UciPacketDecode Structure containing the decoded UCI packet information.
 */
UciPacketDecode uci_decode(const uint8_t *payload, size_t size);

#ifdef __cplusplus
}
#endif
#endif /* __HAL_UCI_PROTOCOL_H */