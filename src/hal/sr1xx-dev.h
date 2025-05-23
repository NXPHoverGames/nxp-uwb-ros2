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

#ifndef __HAL_SR1XX_DEV_H
#define __HAL_SR1XX_DEV_H

#include "sr1xx.h"

int sr1xx_dev_open(sr1xx_dev *dev);
void sr1xx_dev_close(sr1xx_dev *dev);
int sr1xx_dev_write(sr1xx_dev *dev, const uint8_t *payload, uint16_t size);
int sr1xx_dev_read(sr1xx_dev *dev, uint8_t *payload, uint16_t size);

#endif /* __HAL_SR1XX_DEV_H */