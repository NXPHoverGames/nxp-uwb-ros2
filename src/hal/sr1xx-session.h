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

#ifndef __HAL_SR1XX_SESSION_H
#define __HAL_SR1XX_SESSION_H

#include "sr1xx.h"
#include <errno.h>

int sr1xx_session_init_ranging(sr1xx_dev *dev);
int sr1xx_session_deinit(sr1xx_dev *dev);

int sr1xx_session_set_app_config_twr(sr1xx_dev *dev);
int sr1xx_session_set_app_config_responder(sr1xx_dev *dev);
int sr1xx_session_set_app_config_nxp(sr1xx_dev *dev);
int sr1xx_session_set_initiator_config(sr1xx_dev *dev);
int sr1xx_session_set_responder_config(sr1xx_dev *dev);
int sr1xx_session_set_dl_tdoa_anchor_config(sr1xx_dev *dev);
int sr1xx_session_set_app_config_tag(sr1xx_dev *dev);
int sr1xx_session_set_active_ranging_rounds_tag(sr1xx_dev *dev);
int sr1xx_session_set_app_config_anchor_controller(sr1xx_dev *dev);
int sr1xx_session_set_active_ranging_rounds_controller_anchor(sr1xx_dev *dev);
int sr1xx_session_set_app_config_anchor_controlee(sr1xx_dev *dev);
int sr1xx_session_set_active_ranging_rounds_controlee_anchor(sr1xx_dev *dev);
#endif /* __HAL_SR1XX_SESSION_H */