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

#include <errno.h>
#include <fcntl.h>
#include <hal/sr1xx-dev.h>
#include <hal/sr1xx.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "parameters.h" // Needed for sr1xx-fwd
#include "sr1xx-fwd.h"

static bool_t fw_dwnld_mode = false;

int sr1xx_dev_open(sr1xx_dev *dev) {

    int ret;
    int retries = 6;
    ret = open("/dev/sr1xx", O_RDWR);

    if (ret != -1) {

        dev->devHandle = ret;

        if (!dev->fw_downloaded) {

            /* Rudamentary port of the HBCI firmware mechanism from AOSP
             * Needs to be adapted te become platform agnostic
             */

            initParams();
            setDeviceHandle(dev->devHandle);

            do {
                if (retries % 2 == 0) {
                    // Power cycle, to get the SR150 in the right for uploading FW
                    ioctl(dev->devHandle, SRXXX_SET_PWR, 0);
                    usleep(2000);
                    ioctl(dev->devHandle, SRXXX_SET_PWR, 1);
                    usleep(2000);
                }
                ret = phNxpUciHal_fw_download();
            } while (ret != 0 && retries-- > 0);

            if (ret != 0) {
                dev->fw_downloaded = false;
                printf("FW Download ERROR: %i)\n", ret);
                return -1;
            }

            printf("FW Download status %d\n", ret); // UCI_STATUS_OK 0x00

            dev->fw_downloaded = true;

            sleep(2);

            /* FIXME actually wait for
            -Decoding-UCI-Packet------------------------------------------
            oid: DEVICE_STATUS
            device_state: Unknown DeviceState
            */
        }
    }

    return ret;
}

void sr1xx_dev_close(sr1xx_dev *dev) {
    close(dev->devHandle);
}

int phTmlUwb_spi_write(sr1xx_dev *dev, uint8_t *pBuffer, int nNbBytesToWrite) {
    int ret;
    int numWrote = 0;

    ret = write(dev->devHandle, pBuffer, nNbBytesToWrite);
    if (ret > 0) {
        // printf("_spi_write()_1 ret : %x\n", ret);
        numWrote = ret;
    } else {
        printf("_spi_write()_1 failed : %d\n", ret);
        return -1;
    }
    return numWrote;
}

int sr1xx_dev_write(sr1xx_dev *dev, const uint8_t *payload, uint16_t size) {

    if ((size > UCI_MAX_DATA_LEN) || (size < UCI_PKT_HDR_LEN)) {
        return -EINVAL;
    }

    return phTmlUwb_spi_write(dev, (uint8_t *)payload, (uint16_t)size);
}

int sr1xx_dev_read(sr1xx_dev *dev, uint8_t *payload, uint16_t size) {
    int ret_Read;
    uint16_t totalBtyesToRead = 0;

    UNUSED(size);

    totalBtyesToRead = NORMAL_MODE_HEADER_LEN;
    /*Just requested 3 bytes header here but driver will get the header + payload and returns*/
    ret_Read = read(dev->devHandle, payload, totalBtyesToRead);
    if (ret_Read < 0) {
        printf("_spi_read() error: %d", ret_Read);
        return -1;
    } else if ((fw_dwnld_mode) && ((0xFF == payload[0]) || ((0x00 == payload[0]) && (0x00 == payload[3])))) {
        printf("_spi_read() error: Invalid UCI packet");
        /* To Avoid spurious interrupt after FW download */
        return 0;
    }

    return ret_Read;
}