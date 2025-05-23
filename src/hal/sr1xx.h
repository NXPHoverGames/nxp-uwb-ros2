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

#ifndef _SR1XX_H
#define _SR1XX_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/*********************************************************************************************************************/
/*   GLOBAL DEFINES */
/*********************************************************************************************************************/

#define UCI_MAX_DATA_LEN 4200
#define UCI_PKT_HDR_LEN 0x04
#define NORMAL_MODE_HEADER_LEN 4

#define SRXXX_MAGIC 0xEA
#define SRXXX_SET_PWR _IOW(SRXXX_MAGIC, 0x01, long)
#define SRXXX_SET_FWD _IOW(SRXXX_MAGIC, 0x02, long)

#define UNUSED(x) (void)(x)

typedef uint8_t bool_t; /* boolean data type */

typedef union sr1xx_session_id {
    uint8_t bytes[4];
    uint32_t session;
} sr1xx_session_id;

/**
 * @brief Callback function type for handling received data in the SR1XX driver.
 *
 * This function pointer type defines the signature for callbacks that handle 
 * incoming data from the SR1XX driver. A function matching this signature can 
 * be registered using `sr1xx_driver_register_callback`.
 *
 * @param payload A pointer to the received data payload.
 * @param size The size of the received data payload in bytes.
 * @param user_data A pointer to user-defined data provided during the 
 *                  registration of the callback via 
 *                  `sr1xx_driver_register_callback`.
 */
typedef void (*sr1xx_recv_callback)(uint8_t *payload, size_t size, void *user_data);

/* SR1XX Device structure */
typedef struct sr1xx_dev {
    int devHandle;

    bool_t thread_running; /* Thread running if set to 1, else set to 0 */
    sr1xx_recv_callback recv_callback;
    void *user_data;

    bool_t fw_downloaded;
    bool_t mac_is_extended;
    bool_t initialized;

    uint8_t device_mac_address[8];
    uint8_t *dst_mac_address;

    uint32_t ranging_interval;
    uint32_t no_of_controlees;

    uint8_t channel_id;
    sr1xx_session_id session_id;

    uint8_t fw_version_major;
    uint8_t fw_version_minor;
    uint8_t fw_version_patch;

    uint8_t uci_version_major;
    uint8_t uci_version_minor;
    uint8_t uci_version_patch;

    uint8_t chip_id[16];

    bool_t location_set;
    uint8_t location[13];

} sr1xx_dev;

int sr1xx_coreInit(sr1xx_dev *dev);
int sr1xx_calibInit(sr1xx_dev *dev);

#endif /* _SR1XX_H */