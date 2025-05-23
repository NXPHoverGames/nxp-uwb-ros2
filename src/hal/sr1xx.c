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

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "sr1xx-core.h"
#include "sr1xx-thread.h"
#include "sr1xx.h"
#include "uci_protocol.h"

#define TAG_DEVICE_NAME 0x00
#define TAG_FIRMWARE_VERSION 0x01
#define TAG_UCI_VERSION 0x02
#define TAG_CHIP_ID 0x03

int sr1xx_coreInit(sr1xx_dev *dev) {
    int ret;

    ret = sr1xx_thread_init(dev);

    if (ret < 0) {
        printf("sr1xx_thread_init ERROR: %i \n", ret);
        return ret;
    }

    // Init startup registors
    // phNxpUciHal_set_board_config

    uint8_t buffer[] = {0x2E, 0x00, 0x00, 0x02, 0x73, 0x04};

    ret = sr1xx_send_cmd(dev, buffer, sizeof(buffer), NULL, 1000);

    if (ret < 0) {
        printf("Set boardconfig ERROR: %i \n", ret);
        return ret;
    } else {
        printf("Set boardconfig \n");
    }

    ret = sr1xx_core_reset(dev);

    if (ret < 0) {
        printf("sr1xx_core_reset ERROR: %i \n", ret);
        return ret;
    } else {
        printf("Core reset\n");
    }

    uint8_t getCoreDeviceInfoConfig[4];
    uci_serialize_CoreGetDeviceInfoCmd(NULL, getCoreDeviceInfoConfig);

    static uint8_t uci_response[UCI_MAX_DATA_LEN + UCI_PKT_HDR_LEN]; // Note can overflow

    ret = sr1xx_send_cmd(dev, getCoreDeviceInfoConfig, sizeof(getCoreDeviceInfoConfig), uci_response, 1000);

    if (ret < 0) {
        printf("CoreGetDeviceInfoCmd ERROR: %i \n", ret);
        return ret;
    } else {
        ControlPacket_view control_packet = {0};
        CorePacket_view core_packet = {0};
        CoreGetDeviceInfoRsp_view dev_info = {0};
        size_t index = 0;

        if (!uci_parse_ControlPacket_view(uci_response, ret, &index, &control_packet)) {
            return -EINVAL;
        }

        if (!uci_parse_CorePacket_view(&control_packet, uci_response, ret, &index, &core_packet)) {
            return -EINVAL;
        }

        if (!uci_parse_CoreGetDeviceInfoRsp_view(&core_packet, uci_response, ret, &index, &dev_info)) {
            return -EINVAL;
        }

        if (dev_info.uci_version == 2) {
            bool tag_name = false;
            bool tag_version = false;
            bool tag_uci_version = false;
            bool tag_chip_id = false;
            for (int i = 0; i < dev_info.vendor_spec_info_count; i++) {
                if (!tag_name && dev_info.vendor_spec_info[i] == TAG_DEVICE_NAME) {
                    int j;
                    for (j = 0; j < dev_info.vendor_spec_info[i + 1]; j++) {
                        printf("%c", dev_info.vendor_spec_info[j + i]);
                    }
                    printf("\n");
                    tag_name = true;
                    i += j;
                }
                if (!tag_version && dev_info.vendor_spec_info[i] == TAG_FIRMWARE_VERSION) {
                    if (dev_info.vendor_spec_info[i + 1] == 3) {
                        printf("v%X.%X.%X\n", dev_info.vendor_spec_info[i + 2], dev_info.vendor_spec_info[i + 3],
                               dev_info.vendor_spec_info[i + 4]);
                        tag_version = true;
                        dev->fw_version_major = dev_info.vendor_spec_info[i + 2];
                        dev->fw_version_minor = dev_info.vendor_spec_info[i + 3];
                        dev->fw_version_patch = dev_info.vendor_spec_info[i + 4];
                        i += 4;
                    }
                }
                if (!tag_uci_version && dev_info.vendor_spec_info[i] == TAG_UCI_VERSION) {
                    if (dev_info.vendor_spec_info[i + 1] == 3) {
                        printf("UCI %X.%X.%X\n", dev_info.vendor_spec_info[i + 2], dev_info.vendor_spec_info[i + 3],
                               dev_info.vendor_spec_info[i + 4]);
                        tag_uci_version = true;
                        dev->uci_version_major = dev_info.vendor_spec_info[i + 2];
                        dev->uci_version_minor = dev_info.vendor_spec_info[i + 3];
                        dev->uci_version_patch = dev_info.vendor_spec_info[i + 4];
                        i += 4;
                    }
                }
                if (!tag_chip_id && dev_info.vendor_spec_info[i] == TAG_CHIP_ID) {
                    if (dev_info.vendor_spec_info[i + 1] == 16) {
                        tag_version = true;
                        memcpy(dev->chip_id, &dev_info.vendor_spec_info[i + 2], 16);
                        printf("%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x\n",
                               dev->chip_id[0], dev->chip_id[1], dev->chip_id[2], dev->chip_id[3], dev->chip_id[4],
                               dev->chip_id[5], dev->chip_id[6], dev->chip_id[7], dev->chip_id[8], dev->chip_id[9],
                               dev->chip_id[10], dev->chip_id[11], dev->chip_id[12], dev->chip_id[13], dev->chip_id[14],
                               dev->chip_id[15]);
                        i += 16;
                    }
                }
            }

        } else {
            printf("CoreGetDeviceInfoCmd\n");
            uci_print(uci_response, sizeof(uci_response));
        }
    }

    ret = sr1xx_core_set_config(dev);

    if (ret < 0) {
        printf("sr1xx_core_set_config ERROR: %i \n", ret);
        return ret;
    } else {
        printf("Set coreconfig \n");
    }

    ret = sr1xx_core_vendor_command(dev);

    if (ret < 0) {
        printf("sr1xx_core_vendor_command ERROR: %i \n", ret);
        return ret;
    } else {
        printf("Set vendor_command \n");
    }

    return ret;
}

int sr1xx_calibInit(sr1xx_dev *dev) {

    int ret;

#ifdef OTP
    printf("sr1xx_core_read_calib_xtal_cap\n");
    ret = sr1xx_core_read_calib_xtal_cap(dev);

    if (ret < 0) {
        printf("sr1xx_core_read_calib_xtal_cap ERROR: %i \n", ret);
        return ret;
    }

    printf("sr1xx_core_read_calib_tx_power\n");
    ret = sr1xx_core_read_calib_tx_power(dev);

    if (ret < 0) {
        printf("sr1xx_core_read_calib_tx_power ERROR: %i \n", ret);
        return ret;
    }
#endif

    printf("sr1xx_core_set_antenna_define\n");
    ret = sr1xx_core_set_antenna_define(dev);

    if (ret < 0) {
        printf("sr1xx_core_set_antenna_define ERROR: %i \n", ret);
        return ret;
    }

    printf("sr1xx_core_set_calibration_rf_clk_accuracy_calib\n");
    ret = sr1xx_core_set_calibration_rf_clk_accuracy_calib(dev);

    if (ret < 0) {
        printf("sr1xx_core_set_calibration_rf_clk_accuracy_calib ERROR: %i \n", ret);
        return ret;
    }

    printf("sr1xx_core_set_rx_ant_delay_calib\n");
    ret = sr1xx_core_set_rx_ant_delay_calib(dev);

    if (ret < 0) {
        printf("sr1xx_core_set_rx_ant_delay_calib ERROR: %i \n", ret);
        return ret;
    }

    printf("sr1xx_core_set_pdoa_offset_calib\n");
    ret = sr1xx_core_set_pdoa_offset_calib(dev);

    if (ret < 0) {
        printf("sr1xx_core_set_pdoa_offset_calib ERROR: %i \n", ret);
        return ret;
    }

    printf("sr1xx_core_set_aoa_threshold_pdoa\n");
    ret = sr1xx_core_set_aoa_threshold_pdoa(dev);

    if (ret < 0) {
        printf("sr1xx_core_set_aoa_threshold_pdoa ERROR: %i \n", ret);
        return ret;
    }

    printf("sr1xx_core_set_aoa_antennas_pdoa_calib_pair2\n");
    ret = sr1xx_core_set_aoa_antennas_pdoa_calib_pair2(dev);

    if (ret < 0) {
        printf("r1xx_core_set_aoa_antennas_pdoa_calib_pair2 ERROR: %i \n", ret);
        return ret;
    }

    printf("sr1xx_core_set_aoa_antennas_pdoa_calib_pair1\n");
    ret = sr1xx_core_set_aoa_antennas_pdoa_calib_pair1(dev);

    if (ret < 0) {
        printf("sr1xx_core_set_aoa_antennas_pdoa_calib_pair1 ERROR: %i \n", ret);
        return ret;
    }

#if 0
    ret = sr1xx_core_set_set_calibration_pdoa_manufact_zero_offset_calib(dev);

    if (ret < 0) {
        printf("sr1xx_core_set_set_calibration_pdoa_manufact_zero_offset_calib ERROR: %i \n",ret);
    }
#endif

    return ret;
}