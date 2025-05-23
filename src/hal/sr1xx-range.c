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

#include "sr1xx-range.h"
#include "sr1xx-core.h"
#include "sr1xx-session.h"
#include "sr1xx-thread.h"
#include <string.h>

int sr1xx_uwb_range_start(sr1xx_dev *dev) {
    int ret;

    SessionStartCmd_view session_start = {0};
    session_start.session_id = dev->session_id.session;

    uint8_t buffer[uci_size_SessionStartCmd()];

    ret = uci_serialize_SessionStartCmd(&session_start, buffer);
    buffer[3] = 4; // Fix for serializer not setting payload length in octet 3

    ret = sr1xx_send_cmd(dev, buffer, ret, NULL, 1000);

    return ret;
}

int sr1xx_uwb_range_stop(sr1xx_dev *dev) {
    int ret;

    SessionStopCmd_view session_stop = {0};
    session_stop.session_id = dev->session_id.session;

    uint8_t buffer[uci_size_SessionStopCmd()];

    ret = uci_serialize_SessionStopCmd(&session_stop, buffer);
    buffer[3] = 4; // Fix for serializer not setting payload length in octet 3

    ret = sr1xx_send_cmd(dev, buffer, ret, NULL, 1000);

    return ret;
}
