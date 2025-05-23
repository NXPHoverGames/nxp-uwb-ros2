/*
 * Copyright (C) 2011-2012 Broadcom Corporation
 * Copyright 2018-2025 NXP
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

#ifndef PARAMETERS_H
#define PARAMETERS_H

// FIXME PARAM SYS

#define NAME_NXP_UWB_DEVICE_NODE "NXP_UWB_DEVICE_NODE"
#define NAME_NXP_UWB_PROD_FW_FILENAME "NXP_UWB_PROD_FW_FILENAME"
#define NAME_NXP_UWB_DEV_FW_FILENAME "NXP_UWB_DEV_FW_FILENAME"
#define NAME_NXP_UWB_FW_FILENAME "NXP_UWB_FW_FILENAME"
#define NAME_NXP_UWB_EXT_APP_DEFAULT_CONFIG "NXP_UWB_EXT_APP_DEFAULT_CONFIG"
#define NAME_NXP_UWB_EXT_APP_SR1XX_T_CONFIG "NXP_UWB_EXT_APP_SR1XX_T_CONFIG"
#define NAME_NXP_UWB_EXT_APP_SR1XX_S_CONFIG "NXP_UWB_EXT_APP_SR1XX_S_CONFIG"
#define NAME_NXP_UWB_EXT_APP_DEFAULT_CONFIG "NXP_UWB_EXT_APP_DEFAULT_CONFIG"
#define NAME_UWB_USER_FW_BOOT_MODE_CONFIG "UWB_USER_FW_BOOT_MODE_CONFIG"
#define NAME_NXP_COUNTRY_CODE_CONFIG "NXP_COUNTRY_CODE_CONFIG"
#define NAME_UWB_FW_DOWNLOAD_LOG "UWB_FW_DOWNLOAD_LOG"

void initParams();
int getParamStrValue(const char *key, char *value, unsigned long len);
int getParamNumValue(const char *key, unsigned long *value, unsigned long len);

#endif // PARAMETERS_H