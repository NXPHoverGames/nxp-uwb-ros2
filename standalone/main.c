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

#include "hal/uci_protocol.h"
#include "sr1xx_driver.h"
#include <unistd.h>

#define DEFAULT_DEVICE_MAC {0x11, 0x11};
#define DEFAULT_DST_MAC {0x22, 0x22};
#define DEFAULT_TAG_MAC {0x33, 0x33};
#define DEFAULT_TAG_DST_MAC {0xff, 0xff};

static const uint8_t default_device_mac_address[8] = DEFAULT_DEVICE_MAC;
static const uint8_t default_dest_mac_address[8] = DEFAULT_DST_MAC;
static const uint8_t default_tag_mac_address[8] = DEFAULT_TAG_MAC;
static const uint8_t default_tag_dst_mac_address[8] = DEFAULT_TAG_DST_MAC;

char *role = TWR_INITIATOR;
MacAddressIndicator mac_indicator = MACADDRESSINDICATOR_SHORT_ADDRESS;

uint8_t sr1xx_device_mac_address[8] = DEFAULT_DEVICE_MAC;
uint8_t sr1xx_dst_mac_address[8] = DEFAULT_DST_MAC;
uint16_t *sr1xx_dst_mac_addresses;

uint8_t channel_id = 0x09;
const uint8_t session_id[4] = {0x01, 0x00, 0x00, 0x00};
const uint32_t ranging_interval = 20;
uint8_t no_of_controlees = 0;

sr1xx_dev dev;

void print_mac(char name[], uint8_t mac_address[]) {
    printf("%s = [", name);
    for (int i = 0; i < strlen(mac_address); i++) {
        printf(" %02X", mac_address[i] & 0xff);
    }
    printf(" ]\n");
}

void set_source_dest_mac(int argc, char *argv[]) {
    if (argc == 4) {
        if (strlen(argv[2]) <= 4) {
            *(uint16_t *)sr1xx_device_mac_address = strtol(argv[2], NULL, 16);
        } else {
            memcpy(sr1xx_device_mac_address, default_device_mac_address, sizeof(default_device_mac_address));
        }
        if (strlen(argv[3]) <= 4) {
            *(uint16_t *)sr1xx_dst_mac_address = strtol(argv[3], NULL, 16);
        } else {
            memcpy(sr1xx_dst_mac_address, default_dest_mac_address, sizeof(default_dest_mac_address));
        }
    } else {
        memcpy(sr1xx_device_mac_address, default_device_mac_address, sizeof(default_device_mac_address));
        memcpy(sr1xx_dst_mac_address, default_dest_mac_address, sizeof(default_dest_mac_address));
    }
}

void set_controller_dest_mac(int argc, char *argv[]) {
    bool mac_is_extended = false;

    if (argc >= 4) {
        if (strlen(argv[2]) <= 4) {
            *(uint16_t *)sr1xx_device_mac_address = strtol(argv[2], NULL, 16);
        }

        sr1xx_dst_mac_addresses = malloc(2 * (argc - 3));

        for (int i = 3; i < argc; i++) {
            if (strlen(argv[i]) <= 4) {
                sr1xx_dst_mac_addresses[no_of_controlees] = strtol(argv[i], NULL, 16);
            }
            no_of_controlees++;
        }

    } else {
        memcpy(sr1xx_device_mac_address, default_device_mac_address, sizeof(default_device_mac_address));
        memcpy(sr1xx_dst_mac_address, default_dest_mac_address, sizeof(default_dest_mac_address));
    }
}

void set_source_dest_mac_swap(int argc, char *argv[]) {
    if (argc == 4) {
        if (strlen(argv[2]) <= 4) {
            *(uint16_t *)sr1xx_device_mac_address = strtol(argv[2], NULL, 16);
        } else {
            memcpy(sr1xx_device_mac_address, default_dest_mac_address, sizeof(default_dest_mac_address));
        }
        if (strlen(argv[3]) <= 4) {
            *(uint16_t *)sr1xx_dst_mac_address = strtol(argv[3], NULL, 16);
        } else {
            memcpy(sr1xx_dst_mac_address, default_device_mac_address, sizeof(default_device_mac_address));
        }
    } else {
        // Swap predefined mac addresses
        memcpy(sr1xx_device_mac_address, default_dest_mac_address, sizeof(default_dest_mac_address));
        memcpy(sr1xx_dst_mac_address, default_device_mac_address, sizeof(default_device_mac_address));
    }
}

void set_tag_mac(int argc, char *argv[]) {
    if (argc == 3) {
        if (strlen(argv[2]) <= 4) {
            *(uint16_t *)sr1xx_device_mac_address = strtol(argv[2], NULL, 16);
        } else {
            memcpy(sr1xx_device_mac_address, default_tag_mac_address, sizeof(default_tag_mac_address));
        }
    } else {
        memcpy(sr1xx_device_mac_address, default_tag_mac_address, sizeof(default_tag_mac_address));
    }
    memcpy(sr1xx_dst_mac_address, default_tag_dst_mac_address, sizeof(default_tag_dst_mac_address));
}

void rx_callback(uint8_t *payload, size_t size, void *user_data) {
    UNUSED(user_data);
    printf("Callback\n");
}

int main(int argc, char *argv[]) {
    int ret;

    if (argc >= 2) {
        if (strcmp(argv[1], "i") == 0) {
            role = TWR_INITIATOR;
            set_source_dest_mac(argc, argv);
            no_of_controlees = 1;
        } else if (strcmp(argv[1], "r") == 0) {
            role = TWR_RESPONDER;
            set_source_dest_mac_swap(argc, argv);
            no_of_controlees = 1;
        } else if (strcmp(argv[1], "t") == 0) {
            role = TDOA_DL_TAG;
            set_tag_mac(argc, argv);
            no_of_controlees = 1;
        } else if (strcmp(argv[1], "1") == 0) {
            role = TDOA_DL_ANCHOR_CONTROLLER;
            set_controller_dest_mac(argc, argv);
        } else if (strcmp(argv[1], "2") == 0) {
            role = TDOA_DL_ANCHOR_CONTROLEE;
            set_source_dest_mac_swap(argc, argv);
            no_of_controlees = 1;
        } else {
            printf("No valid role selected -> role = TWR_INITIATOR\n");
            role = TWR_INITIATOR;
        }
    } else {
        printf("No valid role selected -> role = TWR_INITIATOR\n");
        role = TWR_INITIATOR;
    }

    if (role == TDOA_DL_ANCHOR_CONTROLLER) {
        printf("role = %s, device [%X],", role, *(uint16_t *)sr1xx_device_mac_address);
        for (int i = 0; i < no_of_controlees; i++) {
            printf(" dst [%X]", sr1xx_dst_mac_addresses[i]);
        }
        printf("\n");
    } else {
        printf("role = %s, device [%X], dst [%X]\n", role, *(uint16_t *)sr1xx_device_mac_address,
               *(uint16_t *)sr1xx_dst_mac_address);
    }
    sleep(2);

    if (role == TDOA_DL_ANCHOR_CONTROLLER) {
        ret = sr1xx_driver_init(&dev, channel_id, *(uint32_t *)&session_id[0], mac_indicator, sr1xx_device_mac_address,
                                (uint8_t *)sr1xx_dst_mac_addresses, no_of_controlees, ranging_interval);
    } else {
        ret = sr1xx_driver_init(&dev, channel_id, *(uint32_t *)&session_id[0], mac_indicator, sr1xx_device_mac_address,
                                sr1xx_dst_mac_address, no_of_controlees, ranging_interval);
    }

    if (ret < 0) {
        printf("ERROR Initializing SR1XX %i\n", ret);
        return -1;
    }

    ret = sr1xx_driver_start(&dev, role);

    if (ret < 0) {
        printf("ERROR Starting SR1XX %i\n", ret);
        return -1;
    }

    sr1xx_driver_register_callback(&dev, rx_callback, NULL);

    while (1) {
        // 2nd RX thread reads commands
        sleep(1);
    }

    return 0;
}
