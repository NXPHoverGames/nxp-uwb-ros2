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

#ifndef __HAL_SR1XX_CORE_H
#define __HAL_SR1XX_CORE_H

#include "sr1xx.h"
#include <errno.h>

int sr1xx_core_reset(sr1xx_dev *dev);
int sr1xx_core_set_config(sr1xx_dev *dev);
int sr1xx_core_vendor_command(sr1xx_dev *dev);
int sr1xx_core_set_antenna_define(sr1xx_dev *dev);
int sr1xx_core_set_power_calibration(sr1xx_dev *dev);
int sr1xx_core_set_rx_ant_delay_calib(sr1xx_dev *dev);
int sr1xx_core_set_pdoa_offset_calib(sr1xx_dev *dev);
int sr1xx_core_set_aoa_antennas_pdoa_calib_pair1(sr1xx_dev *dev);
int sr1xx_core_set_aoa_antennas_pdoa_calib_pair2(sr1xx_dev *dev);
int sr1xx_core_set_aoa_threshold_pdoa(sr1xx_dev *dev);
int sr1xx_core_set_set_calibration_pdoa_manufact_zero_offset_calib(sr1xx_dev *dev);
int sr1xx_core_set_calibration_rf_clk_accuracy_calib(sr1xx_dev *dev);

#endif /* __HAL_SR1XX_CORE_H */