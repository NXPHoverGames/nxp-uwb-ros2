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

#include "uci_protocol.h"

#define VENDOR_E_CORE_DEVICE_INIT_OID 0x0
#define VENDOR_F_APP_CONFIG_RSP_OID 0x0
#define VENDOR_F_DEVICE_CALIBRATION_RSP_OID 0x21

int uci_print_core(ControlPacket_view *parent, const uint8_t *payload, size_t size, size_t *index) {
    int ret = 0;
    bool parse_success;
    CorePacket_view core_packet = {0};
    parse_success = uci_parse_CorePacket_view(parent, payload, size, index, &core_packet);

    if (!parse_success) {
        printf("Error parsing CorePacket\n");
        return -EINVAL;
    }

    uci_print_CorePacket_view(&core_packet);

    switch (core_packet.oid) {
    case COREOPCODEID_DEVICE_RESET: {
        CoreDeviceResetRsp_view device_reset_response = {0};
        parse_success = uci_parse_CoreDeviceResetRsp_view(&core_packet, payload, size, index, &device_reset_response);
        if (parse_success) {
            uci_print_CoreDeviceResetRsp_view(&device_reset_response);
        }
    } break;

    case COREOPCODEID_DEVICE_STATUS: {
        CoreDeviceStatusNtf_view device_status_ntf = {0};
        parse_success = uci_parse_CoreDeviceStatusNtf_view(&core_packet, payload, size, index, &device_status_ntf);
        if (parse_success) {
            uci_print_CoreDeviceStatusNtf_view(&device_status_ntf);
        }
    } break;

    case COREOPCODEID_GET_DEVICE_INFO: {
        CoreGetDeviceInfoRsp_view device_info_response = {0};
        parse_success = uci_parse_CoreGetDeviceInfoRsp_view(&core_packet, payload, size, index, &device_info_response);
        if (parse_success) {
            uci_print_CoreGetDeviceInfoRsp_view(&device_info_response);
        }
        uci_free_CoreGetDeviceInfoRsp_view(&device_info_response);
    } break;

    case COREOPCODEID_GET_CAPS_INFO: {
        CoreGetCapsInfoRsp_view device_caps_info = {0};
        parse_success = uci_parse_CoreGetCapsInfoRsp_view(&core_packet, payload, size, index, &device_caps_info);
        if (parse_success) {
            uci_print_CoreGetCapsInfoRsp_view(&device_caps_info);
        }
        uci_free_CoreGetCapsInfoRsp_view(&device_caps_info);
    } break;

    case COREOPCODEID_SET_CONFIG: {
        CoreSetConfigRsp_view device_set_config_response = {0};
        parse_success =
            uci_parse_CoreSetConfigRsp_view(&core_packet, payload, size, index, &device_set_config_response);
        if (parse_success) {
            uci_print_CoreSetConfigRsp_view(&device_set_config_response);
        }
        uci_free_CoreSetConfigRsp_view(&device_set_config_response);
    } break;

    case COREOPCODEID_GET_CONFIG: {
        CoreGetConfigRsp_view device_get_config_response = {0};
        parse_success =
            uci_parse_CoreGetConfigRsp_view(&core_packet, payload, size, index, &device_get_config_response);
        if (parse_success) {
            uci_print_CoreGetConfigRsp_view(&device_get_config_response);
        }
        uci_free_CoreGetConfigRsp_view(&device_get_config_response);
    } break;

    case COREOPCODEID_GENERIC_ERROR: {
        CoreGenericErrorNtf_view device_generic_error_ntf = {0};
        parse_success =
            uci_parse_CoreGenericErrorNtf_view(&core_packet, payload, size, index, &device_generic_error_ntf);
        if (parse_success) {
            uci_print_CoreGenericErrorNtf_view(&device_generic_error_ntf);
        }
    } break;

    case COREOPCODEID_QUERY_UWBS_TIMESTAMP: {
        CoreQueryTimeStampRsp_view device_query_uwbs_timestamp_response = {0};
        parse_success = uci_parse_CoreQueryTimeStampRsp_view(&core_packet, payload, size, index,
                                                             &device_query_uwbs_timestamp_response);
        if (parse_success) {
            uci_print_CoreQueryTimeStampRsp_view(&device_query_uwbs_timestamp_response);
        }
    } break;

    default:
        printf("CorePacket %s Decoder Not implemented\n", CoreOpcodeIdText(core_packet.oid));
        ret = -ENOTSUP;
    }

    if (!parse_success) {
        printf("Error parsing CorePacket %s\n", CoreOpcodeIdText(core_packet.oid));
        ret = -EINVAL;
    }

    return 0;
}

int uci_print_vendor_config_e(ControlPacket_view *parent, const uint8_t *payload, size_t size, size_t *index) {
    int ret = 0;
    uint8_t oid = payload[*index];

    printf("Attempting vendor decode GID %X OID %X MT %s\n", parent->gid, oid, MessageTypeText(parent->mt));

    if (parent->mt == MESSAGETYPE_RESPONSE) {
        switch (oid) {

        case VENDOR_E_CORE_DEVICE_INIT_OID: {
            Status status = payload[*index + 1];
            printf("Response Status %s\n", StatusText(status));
        } break;

        default:
            printf("VENDOR E response %i Decoder Not implemented\n", oid);
            ret = -EINVAL;
        }
    } else {
        ret = -ENOTSUP;
    }

    return ret;
}

int uci_print_vendor_config_f(ControlPacket_view *parent, const uint8_t *payload, size_t size, size_t *index) {
    int ret = 0;
    bool parse_success;
    uint8_t oid = payload[*index];

    printf("Attempting vendor decode GID %X OID %X MT %s\n", parent->gid, oid, MessageTypeText(parent->mt));

    if (parent->mt == MESSAGETYPE_RESPONSE) {
        switch (oid) {

        case VENDOR_F_APP_CONFIG_RSP_OID: {
            parent->gid = GROUPID_SESSION_CONFIG; // SET_VENDOR_APP_CONFIG_RSP decodes the same as SESSION_CONFIG

            SessionConfigPacket_view session_config_packet = {0};
            parse_success = uci_parse_SessionConfigPacket_view(parent, payload, size, index, &session_config_packet);

            if (!parse_success) {
                printf("Error parsing SessionConfigPacket\n");
                parse_success = false;
            } else {
                session_config_packet.oid = SESSIONCONFIGOPCODEID_SET_APP_CONFIG;
                SessionSetAppConfigRsp_view session_setconfig = {0};
                parse_success = uci_parse_SessionSetAppConfigRsp_view(&session_config_packet, payload, size, index,
                                                                      &session_setconfig);
                if (parse_success) {
                    uci_print_SessionSetAppConfigRsp_view(&session_setconfig);
                }
                uci_free_SessionSetAppConfigRsp_view(&session_setconfig);
            }
        } break;

        case VENDOR_F_DEVICE_CALIBRATION_RSP_OID: {
            Status status = payload[*index + 1];
            printf("Calib set status %s\n", StatusText(status));
        } break;

        default:
            printf("VENDOR F response %i Decoder Not implemented\n", oid);
            ret = -ENOTSUP;
        }
    }

    if (!parse_success) {
        ret = -EINVAL;
    }

    return ret;
}

int uci_print_session_config(ControlPacket_view *parent, const uint8_t *payload, size_t size, size_t *index) {
    int ret = 0;
    bool parse_success;
    SessionConfigPacket_view session_config_packet = {0};
    parse_success = uci_parse_SessionConfigPacket_view(parent, payload, size, index, &session_config_packet);

    if (!parse_success) {
        printf("Error parsing SessionConfigPacket\n");
        return -EINVAL;
    }

    uci_print_SessionConfigPacket_view(&session_config_packet);

    if (session_config_packet.mt == MESSAGETYPE_RESPONSE) {
        switch (session_config_packet.oid) {
        case SESSIONCONFIGOPCODEID_INIT: {
            SessionInitRsp_view session_init = {0};
            parse_success = uci_parse_SessionInitRsp_view(&session_config_packet, payload, size, index, &session_init);
            if (parse_success) {
                uci_print_SessionInitRsp_view(&session_init);
            }
        } break;

        case SESSIONCONFIGOPCODEID_SET_APP_CONFIG: {
            SessionSetAppConfigRsp_view session_setconfig = {0};
            parse_success =
                uci_parse_SessionSetAppConfigRsp_view(&session_config_packet, payload, size, index, &session_setconfig);
            if (parse_success) {
                uci_print_SessionSetAppConfigRsp_view(&session_setconfig);
            }
            uci_free_SessionSetAppConfigRsp_view(&session_setconfig);
        } break;

        case SESSIONCONFIGOPCODEID_UPDATE_CONTROLLER_MULTICAST_LIST: {
            SessionUpdateControllerMulticastListRsp_view session_update_multicast = {0};
            parse_success = uci_parse_SessionUpdateControllerMulticastListRsp_view(
                &session_config_packet, payload, size, index, &session_update_multicast);
            if (parse_success) {
                uci_print_SessionUpdateControllerMulticastListRsp_view(&session_update_multicast);
            }
        } break;

        case SESSIONCONFIGOPCODEID_UPDATE_DT_ANCHOR_RANGING_ROUNDS: {
            SessionUpdateDtAnchorRangingRoundsRsp_view session_update_ranging_rounds = {0};
            parse_success = uci_parse_SessionUpdateDtAnchorRangingRoundsRsp_view(&session_config_packet, payload, size,
                                                                                 index, &session_update_ranging_rounds);
            if (parse_success) {
                uci_print_SessionUpdateDtAnchorRangingRoundsRsp_view(&session_update_ranging_rounds);
                for (int i = 0; i < session_config_packet.payload_size; i++) {
                    printf("%X ", session_config_packet.payload[i]);
                }
                printf("\n");
                if (session_config_packet.payload_size > 0) {
                    printf("status: %s\n", StatusText(session_config_packet.payload[0]));
                }
            }
        } break;

        case SESSIONCONFIGOPCODEID_UPDATE_DT_TAG_RANGING_ROUNDS: {
            SessionUpdateDtTagRangingRoundsRsp_view session_update_ranging_rounds = {0};
            parse_success = uci_parse_SessionUpdateDtTagRangingRoundsRsp_view(&session_config_packet, payload, size,
                                                                              index, &session_update_ranging_rounds);
            if (parse_success) {
                uci_print_SessionUpdateDtTagRangingRoundsRsp_view(&session_update_ranging_rounds);
            }
        } break;

        default:
            printf("SESSION_CONFIG response %s Decoder Not implemented\n",
                   SessionConfigOpcodeIdText(session_config_packet.oid));
            ret = -ENOTSUP;
        }
    } else {
        switch (session_config_packet.oid) {

        case SESSIONCONFIGOPCODEID_INIT: {
            SessionInitCmd_view session_init = {0};
            parse_success = uci_parse_SessionInitCmd_view(&session_config_packet, payload, size, index, &session_init);
            if (parse_success) {
                uci_print_SessionInitCmd_view(&session_init);
            }
        } break;

        case SESSIONCONFIGOPCODEID_SET_APP_CONFIG: {
            SessionSetAppConfigCmd_view session_setconfig = {0};
            parse_success =
                uci_parse_SessionSetAppConfigCmd_view(&session_config_packet, payload, size, index, &session_setconfig);
            if (parse_success) {
                uci_print_SessionSetAppConfigCmd_view(&session_setconfig);
            }
            uci_free_SessionSetAppConfigCmd_view(&session_setconfig);
        } break;

        case SESSIONCONFIGOPCODEID_STATUS: {
            SessionStatusNtf_view session_status = {0};
            parse_success =
                uci_parse_SessionStatusNtf_view(&session_config_packet, payload, size, index, &session_status);
            if (parse_success) {
                uci_print_SessionStatusNtf_view(&session_status);
            }
        } break;

        default:
            printf("SESSION_CONFIG %s %s Decoder Not implemented\n", MessageTypeText(session_config_packet.oid),
                   SessionConfigOpcodeIdText(session_config_packet.oid));
            ret = -ENOTSUP;
        }
    }

    if (!parse_success) {
        printf("Error parsing SessionConfigPacket %s\n", SessionConfigOpcodeIdText(session_config_packet.oid));
        ret = -EINVAL;
    }

    return parse_success;
}

bool uci_decode_session_config(UciPacketDecode *decode, ControlPacket_view *parent, const uint8_t *payload, size_t size,
                               size_t *index) {
    bool parse_success;
    SessionConfigPacket_view session_config_packet = {0};
    parse_success = uci_parse_SessionConfigPacket_view(parent, payload, size, index, &session_config_packet);

    if (!parse_success) {
        decode->ret = -EINVAL;
        return false;
    }

    decode->oid = session_config_packet.oid;

    if (session_config_packet.mt == MESSAGETYPE_RESPONSE) {
        switch (session_config_packet.oid) {
        case SESSIONCONFIGOPCODEID_INIT: {
            SessionInitRsp_view session_init = {0};
            parse_success = uci_parse_SessionInitRsp_view(&session_config_packet, payload, size, index, &session_init);
            if (parse_success) {
                decode->status = session_init.status;
            }
        } break;

        case SESSIONCONFIGOPCODEID_SET_APP_CONFIG: {
            SessionSetAppConfigRsp_view session_setconfig = {0};
            parse_success =
                uci_parse_SessionSetAppConfigRsp_view(&session_config_packet, payload, size, index, &session_setconfig);
            if (parse_success) {
                decode->status = session_setconfig.status;
            }
            uci_free_SessionSetAppConfigRsp_view(&session_setconfig);
        } break;

        case SESSIONCONFIGOPCODEID_UPDATE_CONTROLLER_MULTICAST_LIST: {
            SessionUpdateControllerMulticastListRsp_view session_update_multicast = {0};
            parse_success = uci_parse_SessionUpdateControllerMulticastListRsp_view(
                &session_config_packet, payload, size, index, &session_update_multicast);
            if (parse_success) {
                decode->status = session_update_multicast.status;
            }
        } break;

        case SESSIONCONFIGOPCODEID_UPDATE_DT_ANCHOR_RANGING_ROUNDS: {
            SessionUpdateDtAnchorRangingRoundsRsp_view session_update_ranging_rounds = {0};
            parse_success = uci_parse_SessionUpdateDtAnchorRangingRoundsRsp_view(&session_config_packet, payload, size,
                                                                                 index, &session_update_ranging_rounds);
            if (parse_success) {
                if (session_config_packet.payload_size > 0) {
                    decode->status = session_config_packet.payload[0];
                }
                // FIXME decode status
            }
        } break;

        case SESSIONCONFIGOPCODEID_UPDATE_DT_TAG_RANGING_ROUNDS: {
            SessionUpdateDtTagRangingRoundsRsp_view session_update_ranging_rounds = {0};
            parse_success = uci_parse_SessionUpdateDtTagRangingRoundsRsp_view(&session_config_packet, payload, size,
                                                                              index, &session_update_ranging_rounds);
            if (parse_success) {
                decode->status = session_update_ranging_rounds.status;
            }
        } break;

        default:
            printf("SESSION_CONFIG response %s Decoder Not implemented\n",
                   SessionConfigOpcodeIdText(session_config_packet.oid));
        }
    }

    if (!parse_success) {
        printf("Error parsing SessionConfigPacket %s\n", SessionConfigOpcodeIdText(session_config_packet.oid));
    }

    return parse_success;
}

int uci_print_session_control(ControlPacket_view *parent, const uint8_t *payload, size_t size, size_t *index) {
    int ret = 0;
    bool parse_success;
    SessionControlPacket_view session_control_packet = {0};
    parse_success = uci_parse_SessionControlPacket_view(parent, payload, size, index, &session_control_packet);

    if (!parse_success) {
        printf("Error parsing SessionControlPacket\n");
        return -EINVAL;
    }

    uci_print_SessionControlPacket_view(&session_control_packet);

    if (session_control_packet.mt == MESSAGETYPE_RESPONSE) {
        switch (session_control_packet.oid) {
        case SESSIONCONTROLOPCODEID_START: {
            SessionStartRsp_view session_start = {0};
            parse_success =
                uci_parse_SessionStartRsp_view(&session_control_packet, payload, size, index, &session_start);
            if (parse_success) {
                uci_print_SessionStartRsp_view(&session_start);
            }
        } break;

        default:
            printf("SessionControlPacket response %s Decoder Not implemented\n",
                   SessionControlOpcodeIdText(session_control_packet.oid));
            ret = -ENOTSUP;
        }
    } else if (session_control_packet.mt == MESSAGETYPE_COMMAND) {
        switch (session_control_packet.oid) {

        case SESSIONCONTROLOPCODEID_START: {
            SessionStartCmd_view session_start = {0};
            parse_success =
                uci_parse_SessionStartCmd_view(&session_control_packet, payload, size, index, &session_start);
            if (parse_success) {
                uci_print_SessionStartCmd_view(&session_start);
            }
        } break;

        default:
            printf("SessionControlPacket response %s Decoder Not implemented\n",
                   SessionControlOpcodeIdText(session_control_packet.oid));
            ret = -ENOTSUP;
        }
    } else {
        switch (session_control_packet.oid) {

        case SESSIONCONTROLOPCODEID_START: {
            SessionInfoNtf_view session_info = {0};
            parse_success = uci_parse_SessionInfoNtf_view(&session_control_packet, payload, size, index, &session_info);
            if (parse_success) {
                uci_print_SessionInfoNtf_view(&session_info);
                if (session_info.ranging_measurement_type == RANGINGMEASUREMENTTYPE_TWO_WAY) {
                    ShortMacTwoWaySessionInfoNtf_view twoway_session = {0};
                    parse_success = uci_parse_ShortMacTwoWaySessionInfoNtf_view(&session_info, payload, size, index,
                                                                                &twoway_session);
                    if (parse_success) {
                        uci_print_ShortMacTwoWaySessionInfoNtf_view(&twoway_session);
                    }

                    uci_free_ShortMacTwoWaySessionInfoNtf_view(&twoway_session);
                } else if (session_info.ranging_measurement_type == RANGINGMEASUREMENTTYPE_DL_TDOA) {
                    ShortMacDlTDoASessionInfoNtf_view tdoa_session = {0};
                    parse_success =
                        uci_parse_ShortMacDlTDoASessionInfoNtf_view(&session_info, payload, size, index, &tdoa_session);
                    if (parse_success) {
                        uci_print_ShortMacDlTDoASessionInfoNtf_view(&tdoa_session);
                    }
                    uci_free_ShortMacDlTDoASessionInfoNtf_view(&tdoa_session);
                }
            }
        } break;

        default:
            printf("SessionControlPacket response %s Decoder Not implemented\n",
                   SessionControlOpcodeIdText(session_control_packet.oid));
            ret = -ENOTSUP;
        }
    }

    if (!parse_success) {
        printf("Error parsing SessionControlPacket %s\n", SessionControlOpcodeIdText(session_control_packet.oid));
        ret = -EINVAL;
    }

    return ret;
}

int uci_print_control(const uint8_t *payload, size_t length) {
    int ret = 0;
    size_t index = 0;
    ControlPacket_view control_packet = {0};

    if (uci_parse_ControlPacket_view(payload, length, &index, &control_packet)) {

        switch (control_packet.gid) {
        case GROUPID_CORE:
            ret = uci_print_core(&control_packet, payload, length, &index);
            break;

        case GROUPID_SESSION_CONFIG:
            ret = uci_print_session_config(&control_packet, payload, length, &index);
            break;

        case GROUPID_SESSION_CONTROL:
            ret = uci_print_session_control(&control_packet, payload, length, &index);
            break;

        case GROUPID_VENDOR_RESERVED_E:
            ret = uci_print_vendor_config_e(&control_packet, payload, length, &index);
            break;

        case GROUPID_VENDOR_RESERVED_F:
            ret = uci_print_vendor_config_f(&control_packet, payload, length, &index);
            break;

        default:
            printf("%s Decoder Not implemented\n", GroupIdText(control_packet.gid));
            ret = -ENOTSUP;
        }
    } else {
        ret = -EINVAL;
    }
    return ret;
}

int uci_print_data(const uint8_t *payload, size_t length) {
    int ret = 0;
    size_t index = 0;
    DataPacket_view data_packet = {0};
    if (uci_parse_DataPacket_view(payload, length, &index, &data_packet)) {
        uci_print_DataPacket_view(&data_packet);
    } else {
        ret = -EINVAL;
    }

    return ret;
}

int uci_print(const uint8_t *payload, size_t length) {
    int ret = 0;

    printf("-Decoding-UCI-Packet------------------------------------------\n");

    size_t index = 0;

    CommonPacketHeader_view common_header = {0};
    if (uci_parse_CommonPacketHeader_view(payload, length, &index, &common_header)) {

        switch (common_header.mt) {
        case MESSAGETYPE_DATA: {
            ret = uci_print_data(payload, length);
        } break;

        case MESSAGETYPE_COMMAND:
        case MESSAGETYPE_RESPONSE:
        case MESSAGETYPE_NOTIFICATION:
            ret = uci_print_control(payload, length);
            break;

        default:
            printf("Error can't determine Messsage type\n");
            ret = -ENOTSUP;
        }
    } else {
        ret = -EINVAL;
    }

    return ret;
}

void uci_decode_control(UciPacketDecode *decode, const uint8_t *payload, size_t size) {
    size_t index = 0;
    ControlPacket_view control_packet = {0};

    if (!uci_parse_ControlPacket_view(payload, size, &index, &control_packet)) {
        decode->ret = -EINVAL;
    }

    decode->gid = control_packet.gid;

    switch (control_packet.gid) {
    case GROUPID_CORE: {
        CorePacket_view core_packet = {0};
        if (!uci_parse_CorePacket_view(&control_packet, payload, size, &index, &core_packet)) {
            decode->ret = -EINVAL;
        }

        if (core_packet.oid == COREOPCODEID_GENERIC_ERROR) {
            CoreGenericErrorNtf_view device_generic_error_ntf = {0};
            decode->status = device_generic_error_ntf.status;
            if (!uci_parse_CoreGenericErrorNtf_view(&core_packet, payload, size, &index, &device_generic_error_ntf)) {
                decode->ret = -EINVAL;
            }

            if (device_generic_error_ntf.status == STATUS_UCI_MESSAGE_RETRY) {
                decode->ret = -EAGAIN;
            } else if (device_generic_error_ntf.status != STATUS_OK) {
                printf("Generic status %s", StatusText(device_generic_error_ntf.status));
                decode->ret = -EINVAL;
            }
        }

        decode->oid = core_packet.oid;
    } break;

    case GROUPID_SESSION_CONFIG:
        uci_decode_session_config(decode, &control_packet, payload, size, &index);
        break;

    case GROUPID_SESSION_CONTROL:
        SessionControlPacket_view session_control_packet = {0};
        if (!uci_parse_SessionControlPacket_view(&control_packet, payload, size, &index, &session_control_packet)) {
            decode->ret = -EINVAL;
        }
        decode->oid = session_control_packet.oid;
        break;

    case GROUPID_VENDOR_RESERVED_A:
    case GROUPID_VENDOR_RESERVED_E:
    case GROUPID_VENDOR_RESERVED_F:
        decode->oid = 0;
        break;

    default:
        printf("%s Encoder Not implemented\n", GroupIdText(control_packet.gid));
        decode->ret = -EINVAL;
    }
}

UciPacketDecode uci_decode(const uint8_t *payload, size_t size) {
    size_t index = 0;
    UciPacketDecode decode = {0};

    CommonPacketHeader_view common_header = {0};
    if (!uci_parse_CommonPacketHeader_view(payload, size, &index, &common_header)) {
        decode.ret = -EINVAL;
    }

    // TODO PBF

    decode.mt = common_header.mt;

    switch (common_header.mt) {
    case MESSAGETYPE_DATA: {
        // uci_print_data(payload, size);
        printf("DATA not implemented\n");
        decode.oid = 0;
        // decode.ret = -EINVAL; // TODO
    } break;

    case MESSAGETYPE_RESPONSE:
        uci_decode_control(&decode, payload, size);
        break;

    case MESSAGETYPE_COMMAND:
    case MESSAGETYPE_NOTIFICATION:
        uci_decode_control(&decode, payload, size);
        break;

    default:
        printf("Error can't determine Messsage type\n");
        decode.ret = -EINVAL;
    }

    return decode;
}