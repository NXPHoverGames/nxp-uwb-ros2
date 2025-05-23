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

#ifndef _SR1XX_DRIVER_H
#define _SR1XX_DRIVER_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hal/sr1xx.h"

#define TWR_INITIATOR "twr_initiator"
#define TWR_RESPONDER "twr_responder"
#define TDOA_DL_TAG "tdoa_dl_tag"
#define TDOA_DL_ANCHOR_CONTROLLER "tdoa_dl_anchor_controller"
#define TDOA_DL_ANCHOR_CONTROLEE "tdoa_dl_anchor_controlee"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the SR1XX driver with specified configuration parameters.
 *
 * This function sets up the SR1XX driver for operation by configuring it with 
 * the provided channel, session, and MAC address details, along with the 
 * number of controlees and the ranging interval.
 *
 * @param dev A pointer to the sr1xx_dev structure representing the device.
 * @param channel_id The ID of the communication channel to be used.
 * @param session_id The session identifier for the driver initialization.
 * @param mac_is_extended A boolean indicating whether the MAC address format 
 *                        is extended (true) or standard (false).
 * @param device_mac_address A pointer to the MAC address of the device.
 * @param dst_mac_address A pointer to the destination MAC address.
 * @param no_of_controlees The number of controlees to be supported in this session.
 * @param ranging_interval The interval for ranging operations in milliseconds.
 * 
 * @return Returns 0 on success, or a negative value indicating an error code 
 *         if the initialization fails.
 */
int sr1xx_driver_init(sr1xx_dev *dev, uint8_t channel_id, uint32_t session_id, bool_t mac_is_extended,
                      uint8_t *device_mac_address, uint8_t *dst_mac_address, uint8_t no_of_controlees,
                      uint32_t ranging_interval);

/**
 * @brief Starts SR1XX ranging with the specified role.
 *
 * This function starts ranging the SR1XX driver for the given device, 
 * setting it to operate in the specified role.
 *
 * @param dev A pointer to the sr1xx_dev structure representing the device.
 * @param role A string specifying the role to configure the driver.
 *        The following roles are supported:
 *          - twr_initiator
 *          - twr_responder
 *          - tdoa_dl_tag
 *          - tdoa_dl_anchor_controller
 *          - tdoa_dl_anchor_controlee
 *
 * @return Returns 0 on success or a negative value indicating an error code 
 *         if the driver fails to start.
 */
int sr1xx_driver_start(sr1xx_dev *dev, const char *role);

/**
 * @brief Sets the anchor position for the specified device.
 *
 * This function configures the anchor position of the given device
 * by specifying the coordinates in a 3D space.
 *
 * @param dev Pointer to the sr1xx_dev device structure.
 * @param x   The x-coordinate of the anchor position in meters.
 * @param y   The y-coordinate of the anchor position in meters.
 * @param z   The z-coordinate of the anchor position in meters.
 *
 * @return int
 *   - 0 on success.
 *   - A negative value on failure indicating the error code.
 *
 * @note Ensure the device pointer is valid before calling this function.
 */
int sr1xx_set_anchor_position(sr1xx_dev *dev, double x, double y, double z);


/**
 * @brief Registers a callback function for receiving data with the SR1XX driver.
 *
 * This function allows the user to register a callback that will be invoked
 * when data is received by the SR1XX driver. The callback is associated with
 * the given device and can pass user-defined data to the callback function.
 *
 * @param dev A pointer to the sr1xx_dev structure representing the device.
 * @param recv_callback The function pointer to the callback that will be
 *                      called upon data reception. The callback should match
 *                      the `sr1xx_recv_callback` signature.
 * @param user_data A pointer to user-defined data that will be passed to the
 *                  callback when it is invoked.
 *
 * @return Returns 0 on success, or a negative value indicating an error code
 *         if the initialization fails.
 */
int sr1xx_driver_register_callback(sr1xx_dev *dev, sr1xx_recv_callback recv_callback, void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* _SR1XX_DRIVER_H */