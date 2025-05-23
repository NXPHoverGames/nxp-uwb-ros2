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

#ifndef __HAL_SR1XX_RX_THREAD_H
#define __HAL_SR1XX_RX_THREAD_H

#include "sr1xx.h"
#include <uci_packets.h>
#include <errno.h>

int sr1xx_send_cmd(sr1xx_dev *dev, const uint8_t *payload, size_t size, uint8_t *response, int ms);
int sr1xx_wait_for_session_token();
int sr1xx_thread_init(sr1xx_dev *dev);

#endif /* __HAL_SR1XX_RX_THREAD_H */