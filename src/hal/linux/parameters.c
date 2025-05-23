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

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "parameters.h"

#define NUM_KEYS sizeof(_keys) / sizeof(keyMap)

enum value_type {
    BOOL_VAL = 0x00,
    INT_VAL = 0x01,
    FLOAT_VAL = 0x02,
    STRING_VAL = 0x03,
};
typedef uint8_t value_type;

union value {
    char *s;
    int i;
    float f;
} data;

typedef struct {
    const char *key;
    int offset;
    value_type type;
    int length;
    union value default_value;
} keyMap;

typedef struct {
    char nxp_uwb_device_node[260];
    char nxp_uwb_prod_fw_filename[260];
    char nxp_uwb_dev_fw_filename[260];
    int uwb_fw_download_log;
} params;

params _params;

const keyMap _keys[4] = {
    {"NAME_NXP_UWB_DEVICE_NODE",      offsetof(params, nxp_uwb_device_node),      STRING_VAL, sizeof(_params.nxp_uwb_device_node),
     ""                                                                                                                              },
    {"NAME_NXP_UWB_PROD_FW_FILENAME", offsetof(params, nxp_uwb_prod_fw_filename), STRING_VAL,
     sizeof(_params.nxp_uwb_prod_fw_filename),                                                                                     ""},
    {"NAME_NXP_UWB_DEV_FW_FILENAME",  offsetof(params, nxp_uwb_dev_fw_filename),  STRING_VAL,
     sizeof(_params.nxp_uwb_dev_fw_filename),                                                                                      ""},
    {"NAME_UWB_FW_DOWNLOAD_LOG",      offsetof(params, uwb_fw_download_log),      INT_VAL,    sizeof(_params.uwb_fw_download_log),
     0                                                                                                                               },
};

void initParams() {
    // Load defaults to RAM
    for (int i = 0; i < NUM_KEYS; i++) {
        if (_keys[i].type == STRING_VAL) {
            memcpy(((uint8_t *)(&_params) + _keys[i].offset), _keys[i].default_value.s, _keys[i].length);
        } else {
            memcpy(((uint8_t *)(&_params) + _keys[i].offset), &_keys[i].default_value.i, _keys[i].length);
        }
    }
}

int getParamStrValue(const char *key, char *value, unsigned long len) {
    for (int i = 0; i < NUM_KEYS; i++) {
        if (strncmp(_keys[i].key, key, len) == 0 && _keys[i].type == STRING_VAL && _keys[i].length <= len) {
            memcpy(value, ((uint8_t *)(&_params) + _keys[i].offset), sizeof(value));
            return 1;
        }
    }
    return 0;
}

int getParamNumValue(const char *key, unsigned long *value, unsigned long len) {
    for (int i = 0; i < NUM_KEYS; i++) {
        if (strncmp(_keys[i].key, key, len) == 0 && _keys[i].type == INT_VAL && _keys[i].length <= len) {
            memcpy(value, ((uint8_t *)(&_params) + _keys[i].offset), sizeof(value));
            return 1;
        }
    }
    return 0;
}