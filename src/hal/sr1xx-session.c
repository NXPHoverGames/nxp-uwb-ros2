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

#include "sr1xx-session.h"
#include "sr1xx-core.h"
#include "sr1xx-thread.h"
#include "uci_protocol.h"
#include <string.h>

#define SHORT_ADDRESS_SIZE 2

int sr1xx_session_init_ranging(sr1xx_dev *dev) {
    int ret;

    SessionInitCmd_view session_init = {0};

    session_init.session_id = dev->session_id.session;
    session_init.session_type = SESSIONTYPE_FIRA_RANGING_SESSION;

    uint8_t buffer[uci_size_SessionInitCmd()];

    ret = uci_serialize_SessionInitCmd(&session_init, buffer);

    buffer[3] = ret - 4; // Fix for serializer not setting payload length in octet 3

    ret = sr1xx_send_cmd(dev, buffer, ret, NULL, 1000);

    return ret;
}

int sr1xx_session_deinit(sr1xx_dev *dev) {
    int ret;

    SessionDeinitCmd_view session_deinit = {0};

    session_deinit.session_token = dev->session_id.session;

    uint8_t buffer[uci_size_SessionDeinitCmd()];

    ret = uci_serialize_SessionDeinitCmd(&session_deinit, buffer);

    buffer[3] = ret - 4; // Fix for serializer not setting payload length in octet 3

    ret = sr1xx_send_cmd(dev, buffer, ret, NULL, 1000);

    return ret;
}

int sr1xx_session_set_app_config_twr(sr1xx_dev *dev) {
    int ret;

    SessionSetAppConfigCmd_view session_app_config_initiator = {0};

    AppConfigTlv cfg[35];
    size_t cnt = 0;

    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_RANGING_ROUND_USAGE, 2, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_STS_CONFIG, STSCONFIG_STATIC, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_MULTI_NODE_MODE, MULTINODEMODE_ONE_TO_ONE, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_CHANNEL_NUMBER, dev->channel_id, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_NUMBER_OF_CONTROLEES, 1, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_SLOT_DURATION, 2400, uint16_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_RANGING_DURATION, dev->ranging_interval, uint32_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_STS_INDEX, 0, uint32_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_MAC_FCS_TYPE, MACFCSTYPE_CRC_16, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_RANGING_ROUND_CONTROL, 3, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_AOA_RESULT_REQ, AOARESULTREQ_AOA_ENABLED, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_SESSION_INFO_NTF_CONFIG, SESSIONINFONTFCONFIG_ENABLE, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_NEAR_PROXIMITY_CONFIG, 0, uint16_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_FAR_PROXIMITY_CONFIG, 20000, uint16_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_RFRAME_CONFIG, RFRAMECONFIG_SP3, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_RSSI_REPORTING, RSSIREPORTING_ENABLE, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_PREAMBLE_CODE_INDEX, 10, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_SFD_ID, 2, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_PSDU_DATA_RATE, PSDUDATARATE_DATA_RATE_6M81, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_PREAMBLE_DURATION, PREAMBLEDURATION_DURATION_64_SYMBOLS, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_RANGING_TIME_STRUCT, RANGINGTIMESTRUCT_BLOCK_BASED_SCHEDULING, uint8_t, cfg,
                        cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_SLOTS_PER_RR, (dev->ranging_interval / 2), uint8_t, cfg, cnt);
    // APP_CONFIG_TLV_DECL(TX_ADAPTIVE_PAYLOAD_POWER, 1, uint8_t, initiator_config, tvls_count);
    // APP_CONFIG_TLV_DECL(RESPONDER_SLOT_INDEX, 1, uint8_t, initiator_config, tvls_count);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_PRF_MODE, PRFMODE_BPRF_MODE, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_SCHEDULE_MODE, SCHEDULEMODE_TIME_SCHEDULED, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_KEY_ROTATION, KEYROTATION_DISABLE, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_KEY_ROTATION_RATE, 0, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_SESSION_PRIORITY, 50, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_MAC_ADDRESS_MODE, 0, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_NUMBER_OF_STS_SEGMENTS, 1, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_MAX_RR_RETRY, 0, uint16_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_HOPPING_MODE, HOPPINGMODE_DISABLE, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_RESULT_REPORT_CONFIG, 1, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_IN_BAND_TERMINATION_ATTEMPT_COUNT, 0, uint8_t, cfg, cnt);

    session_app_config_initiator.session_token = dev->session_id.session;
    session_app_config_initiator.tlvs = cfg;
    session_app_config_initiator.tlvs_count = cnt;

    uint8_t buffer[uci_size_SessionSetAppConfigCmd(&session_app_config_initiator)];

    ret = uci_serialize_SessionSetAppConfigCmd(&session_app_config_initiator, buffer);

    buffer[3] = ret - 4; // Fix for serializer not setting payload length in octet 3

    ret = sr1xx_send_cmd(dev, buffer, ret, NULL, 1000);

    printf("TWR OUT\n");

    return ret;
}

int sr1xx_session_set_app_config_tag(sr1xx_dev *dev) {
    int ret;

    SessionSetAppConfigCmd_view session_app_config_tag = {0};

    AppConfigTlv cfg[20];
    size_t cnt = 0;

    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_DEVICE_ROLE, DEVICEROLE_DT_TAG, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_MULTI_NODE_MODE, MULTINODEMODE_ONE_TO_MANY, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_MAC_ADDRESS_MODE, MACADDRESSINDICATOR_SHORT_ADDRESS, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_SCHEDULE_MODE, SCHEDULEMODE_TIME_SCHEDULED, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_PTR(APPCONFIGTLVTYPE_DEVICE_MAC_ADDRESS, dev->device_mac_address, uint16_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_RANGING_ROUND_USAGE, RANGINGROUNDUSAGE_ON_WAY_RANGING_DL_TDOA, uint8_t, cfg,
                        cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_DEVICE_TYPE, DEVICETYPE_CONTROLEE, uint8_t, cfg, cnt);

    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_RANGING_DURATION, dev->ranging_interval, uint32_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_SLOTS_PER_RR, 10, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_SLOT_DURATION, 1200, uint16_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_RFRAME_CONFIG, RFRAMECONFIG_SP1, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_IN_BAND_TERMINATION_ATTEMPT_COUNT, 0, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_CHANNEL_NUMBER, dev->channel_id, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_NUMBER_OF_CONTROLEES, 1, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_RSSI_REPORTING, RSSIREPORTING_ENABLE, uint8_t, cfg, cnt);

    session_app_config_tag.session_token = dev->session_id.session;
    session_app_config_tag.tlvs = cfg;
    session_app_config_tag.tlvs_count = cnt;

    uint8_t buffer[uci_size_SessionSetAppConfigCmd(&session_app_config_tag)];

    ret = uci_serialize_SessionSetAppConfigCmd(&session_app_config_tag, buffer);

    buffer[3] = ret - 4; // Fix for serializer not setting payload length in octet 3

    ret = sr1xx_send_cmd(dev, buffer, ret, NULL, 1000);

    return ret;
}

int sr1xx_session_set_active_ranging_rounds_tag(sr1xx_dev *dev) {
    int ret;
    size_t index = 0;

    SessionConfigPacket_view config_set_active_ranging_rounds_anchor = {};

    config_set_active_ranging_rounds_anchor.mt = MESSAGETYPE_COMMAND;
    config_set_active_ranging_rounds_anchor.oid = SESSIONCONFIGOPCODEID_UPDATE_DT_TAG_RANGING_ROUNDS;

    const uint8_t SET_ACTIVE_RANGING_ROUNDS_TAG[] = {
        dev->session_id.bytes[0],
        dev->session_id.bytes[1],
        dev->session_id.bytes[2],
        dev->session_id.bytes[3], // session ID
        0x01,                     // Active Rounds
        0x00,
    };

    config_set_active_ranging_rounds_anchor.payload = SET_ACTIVE_RANGING_ROUNDS_TAG;
    config_set_active_ranging_rounds_anchor.payload_size = sizeof(SET_ACTIVE_RANGING_ROUNDS_TAG);

    uint8_t buffer[uci_size_SessionConfigPacket(&config_set_active_ranging_rounds_anchor)];

    ret = uci_serialize_SessionConfigPacket(&config_set_active_ranging_rounds_anchor, buffer);

    buffer[3] = ret - 4; // Fix for serializer not setting payload length in octet 3

    ret = sr1xx_send_cmd(dev, buffer, ret, NULL, 1000);

    return ret;
}

int sr1xx_session_set_app_config_anchor_controller(sr1xx_dev *dev) {
    int ret;

    SessionSetAppConfigCmd_view session_app_config_anchor_controller = {0};

    AppConfigTlv cfg[1];
    size_t cnt = 0;

    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_DL_TDOA_TIME_REFERENCE_ANCHOR, 1, uint8_t, cfg, cnt);
    // APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_DL_TDOA_RANGING_METHOD, 0x01, uint8_t, cfg, cnt); // DS-TWR

    session_app_config_anchor_controller.session_token = dev->session_id.session;
    session_app_config_anchor_controller.tlvs = cfg;
    session_app_config_anchor_controller.tlvs_count = cnt;

    uint8_t buffer[uci_size_SessionSetAppConfigCmd(&session_app_config_anchor_controller)];

    ret = uci_serialize_SessionSetAppConfigCmd(&session_app_config_anchor_controller, buffer);

    buffer[3] = ret - 4; // Fix for serializer not setting payload length in octet 3

    ret = sr1xx_send_cmd(dev, buffer, ret, NULL, 1000);

    return ret;
}

int sr1xx_session_set_active_ranging_rounds_controller_anchor(sr1xx_dev *dev) {
    int ret;
    size_t index = 0;

    SessionConfigPacket_view config_set_active_ranging_rounds_anchor = {};

    config_set_active_ranging_rounds_anchor.mt = MESSAGETYPE_COMMAND;
    config_set_active_ranging_rounds_anchor.oid = SESSIONCONFIGOPCODEID_UPDATE_DT_ANCHOR_RANGING_ROUNDS;

    const uint8_t SET_ACTIVE_RANGING_ROUNDS_ANCHOR[] = {
        dev->session_id.bytes[0],
        dev->session_id.bytes[1],
        dev->session_id.bytes[2],
        dev->session_id.bytes[3], // session ID
        0x01,                     // Active Rounds
    };

    uint8_t rounds_buffer[5 + 4 * 5 + dev->no_of_controlees * 2 * SHORT_ADDRESS_SIZE]; // FIXME short and long address

    memcpy(rounds_buffer, SET_ACTIVE_RANGING_ROUNDS_ANCHOR, sizeof(SET_ACTIVE_RANGING_ROUNDS_ANCHOR));

    index += sizeof(SET_ACTIVE_RANGING_ROUNDS_ANCHOR);

    rounds_buffer[index++] = 0x00; // Round index 0
    rounds_buffer[index++] = 0x01; // Ranging role Initiators

    if (dev->fw_version_major > 0x44) {
        rounds_buffer[index++] = dev->no_of_controlees;

        memcpy(&rounds_buffer[index], dev->dst_mac_address, dev->no_of_controlees * SHORT_ADDRESS_SIZE);

        index += dev->no_of_controlees * SHORT_ADDRESS_SIZE;

        rounds_buffer[index++] = 0x00; // Slot scheduling 0
    }

    config_set_active_ranging_rounds_anchor.payload = rounds_buffer;
    config_set_active_ranging_rounds_anchor.payload_size = index;

    uint8_t buffer[uci_size_SessionConfigPacket(&config_set_active_ranging_rounds_anchor)];

    ret = uci_serialize_SessionConfigPacket(&config_set_active_ranging_rounds_anchor, buffer);

    buffer[3] = ret - 4; // Fix for serializer not setting payload length in octet 3

    ret = sr1xx_send_cmd(dev, buffer, ret, NULL, 1000);

    return ret;
}

int sr1xx_session_set_app_config_anchor_controlee(sr1xx_dev *dev) {
    int ret;

    SessionSetAppConfigCmd_view session_app_config_anchor_controlee = {0};

    AppConfigTlv cfg[1];
    size_t cnt = 0;

    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_DL_TDOA_RESPONDER_TOF, 0x01, uint8_t, cfg, cnt);
    // APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_DL_TDOA_RANGING_METHOD, 0x01, uint8_t, cfg, cnt); // DS-TWRWR

    session_app_config_anchor_controlee.session_token = dev->session_id.session;
    session_app_config_anchor_controlee.tlvs = cfg;
    session_app_config_anchor_controlee.tlvs_count = cnt;

    uint8_t buffer[uci_size_SessionSetAppConfigCmd(&session_app_config_anchor_controlee)];

    ret = uci_serialize_SessionSetAppConfigCmd(&session_app_config_anchor_controlee, buffer);

    buffer[3] = ret - 4; // Fix for serializer not setting payload length in octet 3

    ret = sr1xx_send_cmd(dev, buffer, ret, NULL, 1000);

    return ret;
}

int sr1xx_session_set_active_ranging_rounds_controlee_anchor(sr1xx_dev *dev) {
    int ret;
    size_t index = 0;

    SessionConfigPacket_view config_set_active_ranging_rounds_anchor = {};

    config_set_active_ranging_rounds_anchor.mt = MESSAGETYPE_COMMAND;
    config_set_active_ranging_rounds_anchor.oid = SESSIONCONFIGOPCODEID_UPDATE_DT_ANCHOR_RANGING_ROUNDS;

    const uint8_t SET_ACTIVE_RANGING_ROUNDS_ANCHOR[] = {
        dev->session_id.bytes[0],
        dev->session_id.bytes[1],
        dev->session_id.bytes[2],
        dev->session_id.bytes[3], // session ID
        0x01,                     // Active Rounds
    };

    uint8_t rounds_buffer[5 + 4 * 5 + dev->no_of_controlees * 2 * SHORT_ADDRESS_SIZE]; // FIXME short and long address

    memcpy(rounds_buffer, SET_ACTIVE_RANGING_ROUNDS_ANCHOR, sizeof(SET_ACTIVE_RANGING_ROUNDS_ANCHOR));

    index += sizeof(SET_ACTIVE_RANGING_ROUNDS_ANCHOR);

    rounds_buffer[index++] = 0x00; // Round index 0
    rounds_buffer[index++] = 0x00; // Ranging role Responder

    config_set_active_ranging_rounds_anchor.payload = rounds_buffer;
    config_set_active_ranging_rounds_anchor.payload_size = index;

    uint8_t buffer[uci_size_SessionConfigPacket(&config_set_active_ranging_rounds_anchor)];

    ret = uci_serialize_SessionConfigPacket(&config_set_active_ranging_rounds_anchor, buffer);

    buffer[3] = ret - 4; // Fix for serializer not setting payload length in octet 3

    ret = sr1xx_send_cmd(dev, buffer, ret, NULL, 1000);

    return ret;
}

int sr1xx_session_set_dl_tdoa_anchor_config(sr1xx_dev *dev) {
    int ret;

    SessionSetAppConfigCmd_view dl_tdoa_anchor_config = {0};

    AppConfigTlv cfg[22];
    size_t cnt = 0;

    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_DEVICE_ROLE, DEVICEROLE_DT_ANCHOR, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_MULTI_NODE_MODE, MULTINODEMODE_ONE_TO_MANY, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_MAC_ADDRESS_MODE, MACADDRESSINDICATOR_SHORT_ADDRESS, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_SCHEDULE_MODE, SCHEDULEMODE_TIME_SCHEDULED, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_PTR(APPCONFIGTLVTYPE_DEVICE_MAC_ADDRESS, dev->device_mac_address, uint16_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_RANGING_ROUND_USAGE, RANGINGROUNDUSAGE_ON_WAY_RANGING_DL_TDOA, uint8_t, cfg,
                        cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_DEVICE_TYPE, DEVICETYPE_CONTROLLER, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_RSSI_REPORTING, RSSIREPORTING_ENABLE, uint8_t, cfg, cnt);

    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_RANGING_DURATION, dev->ranging_interval, uint32_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_SLOTS_PER_RR, 10, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_SLOT_DURATION, 1200, uint16_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_RFRAME_CONFIG, RFRAMECONFIG_SP1, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_IN_BAND_TERMINATION_ATTEMPT_COUNT, 0, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_CHANNEL_NUMBER, dev->channel_id, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_DL_TDOA_ANCHOR_CFO, 1, uint8_t, cfg, cnt);
    if (dev->location_set) {
        // TODO this will most likely change with newever firmware versions
        // Currently byte[4] is bugged out causing the y position least significant byte to be 0
        APP_CONFIG_TLV_ARRAY(APPCONFIGTLVTYPE_DL_TDOA_ANCHOR_LOCATION, dev->location, 13, cfg, cnt);
    } else {
        APP_CONFIG_TLV_ARRAY(APPCONFIGTLVTYPE_DL_TDOA_ANCHOR_LOCATION, dev->location, 1, cfg, cnt);
    }
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_DL_TDOA_TX_ACTIVE_RANGING_ROUNDS, 1, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_NUMBER_OF_CONTROLEES, dev->no_of_controlees, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_STS_CONFIG, STSCONFIG_STATIC, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_DL_TDOA_HOP_COUNT, 1, uint8_t, cfg, cnt);
    if (dev->mac_is_extended) {
        APP_CONFIG_TLV_ARRAY(APPCONFIGTLVTYPE_DST_MAC_ADDRESS, dev->dst_mac_address, dev->no_of_controlees * 8, cfg,
                             cnt);
    } else {
        APP_CONFIG_TLV_ARRAY(APPCONFIGTLVTYPE_DST_MAC_ADDRESS, dev->dst_mac_address, dev->no_of_controlees * 2, cfg,
                             cnt);
    }
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_DL_TDOA_TX_TIMESTAMP_CONF, 0x03, uint8_t, cfg, cnt);

    dl_tdoa_anchor_config.session_token = dev->session_id.session;
    dl_tdoa_anchor_config.tlvs = cfg;
    dl_tdoa_anchor_config.tlvs_count = cnt;

    uint8_t buffer[uci_size_SessionSetAppConfigCmd(&dl_tdoa_anchor_config)];

    ret = uci_serialize_SessionSetAppConfigCmd(&dl_tdoa_anchor_config, buffer);

    buffer[3] = ret - 4; // Fix for serializer not setting payload length in octet 3

    ret = sr1xx_send_cmd(dev, buffer, ret, NULL, 1000);

    return ret;
}

int sr1xx_session_set_app_config_nxp(sr1xx_dev *dev) {
    int ret;

    if (dev->fw_version_major > 0x44) {
        const uint8_t UWB_SESSION_SET_APP_CONFIG_NXP[] = {
            0x2F, 0x00, 0x00, 0x2E, dev->session_id.bytes[0], dev->session_id.bytes[1], dev->session_id.bytes[2],
            dev->session_id.bytes[3],
            0x0C, // Number of parameters
            // 0x00, 0x01, 0x01,                        // MAC_PAYLOAD_ENCRYPTION
            0x02, 0x02, 0x01, 0x01,             // ANTENNAS_CONFIGURATION_TX
            0x03, 0x04, 0x01, 0x02, 0x01, 0x02, // ANTENNAS_CONFIGURATION_RX for 3D, Type2BP
            // 0x60, 0x02, 0x11, 0x00,                  // CIR_CAPTURE_MODE
            // 0x61, 0x01, 0x00,                        // RX_ANTENNA_POLARIZATION_OPTION
            0x62, 0x01, 0x03, // SESSION_SYNC_ATTEMPTS
            0x63, 0x01, 0x03, // SESSION_SHED_ATTEMPTS
            0x64, 0x01, 0x00, // SCHED_STATUS_NTF
            0x65, 0x01, 0x00, // TX_POWER_DELTA_FCC
            0x66, 0x01, 0x00, // TEST_KDF_FEATURE
            0x67, 0x01, 0x00, // TX_POWER_TEMP_COMPENSATION
            // 0x68, 0x01, 0x03,                        // WIFI_COEX_MAX_TOLERANCE_COUNT
            // 0x69, 0x01, 0x00,                        // ADAPTIVE_HOPPING_THRESHOLD
            // 0x6E, 0x01, 0x00,                        // AUTHENTICITY_TAG
            // 0x6F, 0x02, 0x1E, 0x14,                  // RX_NBIC_CONFIG
            0x70, 0x01, 0x03, // MAC_CFG
            // 0x71, 0x01, 0x00                         // SESSION_INBAND_DATA_TX_BLOCKS
            // 0x72, 0x01, 0x00                         // SESSION_INBAND_DATA_RX_BLOCKS
            0x7F, 0x01, 0x01,      // TX_ADAPTIVE_PAYLOAD_POWER
            0x84, 0x01, 0x00,      // ENABLE_FOV
            0x85, 0x02, 0x01, 0x30 // AZIMUTH_FIELD_OF_VIEW
        };

        ret = sr1xx_send_cmd(dev, UWB_SESSION_SET_APP_CONFIG_NXP, sizeof(UWB_SESSION_SET_APP_CONFIG_NXP), NULL, 1000);
    } else {
        const uint8_t UWB_SESSION_SET_APP_CONFIG_NXP[] = {
            0x21, 0x03, 0x00, 0x31, dev->session_id.bytes[0], dev->session_id.bytes[1], dev->session_id.bytes[2],
            dev->session_id.bytes[3],
            0x0A, // Number of parameters
            0xE3, 0x01, 0x01,
            0x76, // CIR_CAPTURE_MODE
                  // ###0xE3, 0x02, 0x01, 0x01,                          // MAC_PAYLOAD_ENCRYPTION
                  // ###0xE3, 0x03, 0x01, 0x01,                          // RX_ANTENNA_POLARIZATION_OPTION
            0xE3, 0x05, 0x01, 0x03, // SESSION_SYNC_ATTEMPTS
            0xE3, 0x06, 0x01, 0x03, // SESSION_SHED_ATTEMPTS
            0xE3, 0x07, 0x01, 0x00, // SCHED_STATUS_NTF
            0xE3, 0x08, 0x01, 0x00, // TX_POWER_DELTA_FCC
            0xE3, 0x09, 0x01, 0x00, // TEST_KDF_FEATURE
            0xE3, 0x0B, 0x01,
            0x00, // TX_POWER_TEMP_COMPENSATION
                  // ###0xE3, 0x0C, 0x01, 0x03,                          // WIFI_COEX_MAX_TOLERANCE_COUNT
                  // ###0xE3, 0x0D, 0x01, 0x00,                          // ADAPTIVE_HOPPING_THRESHOLD
                  // ###0xE3, 0x13, 0x01, 0x00,                          // AUTHENTICITY_TAG
                  // ###0xE3, 0x14, 0x02, 0x1E, 0x14,                    // RX_NBIC_CONFIG
            0xE3, 0x15, 0x01,
            0x03, // MAC_CFG
                  // ###0xE3, 0x16, 0x01, 0x00                           // SESSION_INBAND_DATA_TX_BLOCKS
                  // ###0xE3, 0x17, 0x01, 0x00                           // SESSION_INBAND_DATA_RX_BLOCKS
                  // ###0xE3, 0x18, 0x01, 0x00                           // SUSPEND_RANGING
                  // ###0xE3, 0x19, 0x01, 0x00                           // RX_ANTENNA_SELECTION_RFM
                  // ###0xE3, 0x1A, 0x01, 0x00                           // DATA_TRANSFER_MODE
            0xE3, 0x1B, 0x02, 0x01, 0x00,            // ANTENNAS_CONFIGURATION_TX, supported from FW32
            0xE3, 0x1C, 0x04, 0x00, 0x02, 0x01, 0x02 // ANTENNAS_CONFIGURATION_RX, supported from FW32, for 3D AoA
            //   0xE3, 0x1C, 0x04, 0x00, 0x02, 0x01, 0x00         // ANTENNAS_CONFIGURATION_RX, supported from FW32, for
            //   2D AoA
        };

        ret = sr1xx_send_cmd(dev, UWB_SESSION_SET_APP_CONFIG_NXP, sizeof(UWB_SESSION_SET_APP_CONFIG_NXP), NULL, 1000);
    }

    return ret;
}

int sr1xx_session_set_initiator_config(sr1xx_dev *dev) {
    int ret;

    SessionSetAppConfigCmd_view session_set_initiator_config = {0};

    AppConfigTlv cfg[4];
    size_t cnt = 0;

    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_DEVICE_TYPE, DEVICETYPE_CONTROLLER, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_PTR(APPCONFIGTLVTYPE_DEVICE_MAC_ADDRESS, dev->device_mac_address, uint16_t, cfg, cnt);
    APP_CONFIG_TLV_PTR(APPCONFIGTLVTYPE_DST_MAC_ADDRESS, dev->dst_mac_address, uint16_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_DEVICE_ROLE, DEVICEROLE_INITIATOR, uint8_t, cfg, cnt);

    session_set_initiator_config.session_token = dev->session_id.session;
    session_set_initiator_config.tlvs = cfg;
    session_set_initiator_config.tlvs_count = cnt;

    uint8_t buffer[uci_size_SessionSetAppConfigCmd(&session_set_initiator_config)];

    ret = uci_serialize_SessionSetAppConfigCmd(&session_set_initiator_config, buffer);

    buffer[3] = ret - 4; // Fix for serializer not setting payload length in octet 3

    ret = sr1xx_send_cmd(dev, buffer, ret, NULL, 1000);

    return ret;
}

int sr1xx_session_set_responder_config(sr1xx_dev *dev) {
    int ret;

    SessionSetAppConfigCmd_view session_set_responder_config = {0};

    AppConfigTlv cfg[4];
    size_t cnt = 0;

    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_DEVICE_TYPE, DEVICETYPE_CONTROLEE, uint8_t, cfg, cnt);
    APP_CONFIG_TLV_PTR(APPCONFIGTLVTYPE_DEVICE_MAC_ADDRESS, dev->device_mac_address, uint16_t, cfg, cnt);
    APP_CONFIG_TLV_PTR(APPCONFIGTLVTYPE_DST_MAC_ADDRESS, dev->dst_mac_address, uint16_t, cfg, cnt);
    APP_CONFIG_TLV_DECL(APPCONFIGTLVTYPE_DEVICE_ROLE, DEVICEROLE_RESPONDER, uint8_t, cfg, cnt);

    session_set_responder_config.session_token = dev->session_id.session;
    session_set_responder_config.tlvs = cfg;
    session_set_responder_config.tlvs_count = cnt;

    uint8_t buffer[uci_size_SessionSetAppConfigCmd(&session_set_responder_config)];

    ret = uci_serialize_SessionSetAppConfigCmd(&session_set_responder_config, buffer);

    buffer[3] = ret - 4; // Fix for serializer not setting payload length in octet 3

    ret = sr1xx_send_cmd(dev, buffer, ret, NULL, 1000);

    return ret;
}
