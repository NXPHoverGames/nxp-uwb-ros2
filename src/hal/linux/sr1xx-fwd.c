/*
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
/*************************************************************************************/
/*   INCLUDES                                                                        */
/*************************************************************************************/
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <hal/sr1xx.h>

#include "parameters.h"
#include "sr1xx-fwd.h"

#define FILEPATH_MAXLEN 500

static uint8_t chip_id = 0x00;
static uint8_t deviceLcInfo = 0x00;
static uint8_t is_fw_download_log_enabled = 0x00;

const char *default_prod_fw = "sr150_fw.bin";
const char *default_dev_fw = "sr150_fw_dev.bin";
char default_fw_path[FILEPATH_MAXLEN] = "/lib/firmware/nxp/";

char *configured_fw_name = NULL;

phHbci_MosiApdu_t gphHbci_MosiApdu;
phHbci_MisoApdu_t gphHbci_MisoApdu;
phPalSr100_Config_t tPalConfig;
Options_t gOpts;

bool gPasswd, gImg;
uint8_t gDummyMiso[PHHBCI_MAX_LEN_PAYLOAD_MISO];

static uint8_t gphHbci_ImgHelios[256 * 1024] __attribute__((aligned(4)));

// Debug printf
void phNxpUciHal_print_packet(const char *pString, const uint8_t *p_data, uint16_t len) {
    uint32_t i;
    char print_buffer[len * 3 + 1];

    memset(print_buffer, 0, sizeof(print_buffer));
    for (i = 0; i < len; i++) {
        snprintf(&print_buffer[i * 2], 3, "%02X", p_data[i]);
    }
    if (0 == memcmp(pString, "SEND", 0x04)) {
        printf("len = %3d > %s\n", len, print_buffer);
    } else if (0 == memcmp(pString, "RECV", 0x04)) {
        printf("len = %3d > %s\n", len, print_buffer);
    }

    return;
}

/*************************************************************************************/
/*   LOCAL FUNCTIONS                                                                 */
/*************************************************************************************/
static void setOpts(void) {
    gOpts.link = Link_Default;
    gOpts.mode = Mode_Default;
    gOpts.capture = Capture_Default;
    gOpts.imgFile = NULL;
    gOpts.fImg = NULL;
    gOpts.mosiFile = (char *)"Mosi.bin";
    gOpts.fMosi = NULL;
    gOpts.misoFile = (char *)"Miso.bin";
    gOpts.fMiso = NULL;

    gPasswd = false;
    gImg = false;
}

static int init(void) {
    const uint16_t fw_max_len = 260;
    configured_fw_name = (char *)malloc(fw_max_len * sizeof(char));
    const char *pDefaultFwFileName = NULL;
    int maxSrcLen = (FILEPATH_MAXLEN - strlen(default_fw_path)) - 1;
    if (configured_fw_name == NULL) {
        printf("malloc of configured_fw_name failed ");
        return 1;
    }
    if ((deviceLcInfo == PHHBCI_HELIOS_PROD_KEY_1) || (deviceLcInfo == PHHBCI_HELIOS_PROD_KEY_2)) {
        pDefaultFwFileName = default_prod_fw;
        if (!getParamStrValue(NAME_NXP_UWB_PROD_FW_FILENAME, configured_fw_name, fw_max_len)) {
            printf("Invalid Prod Fw  name keeping the default name: %s", pDefaultFwFileName);
            strncat(default_fw_path, pDefaultFwFileName, maxSrcLen);
        } else {
            printf("configured_fw_name : %s", configured_fw_name);
            strncat(default_fw_path, configured_fw_name, maxSrcLen);
        }
    } else if (deviceLcInfo == PHHBCI_HELIOS_DEV_KEY) {
        pDefaultFwFileName = default_dev_fw;
        if (!getParamStrValue(NAME_NXP_UWB_DEV_FW_FILENAME, configured_fw_name, fw_max_len)) {
            printf("Invalid Dev Fw  name keeping the default name: %s", pDefaultFwFileName);
            strncat(default_fw_path, pDefaultFwFileName, maxSrcLen);
        } else {
            printf("configured_fw_name : %s", configured_fw_name);
            strncat(default_fw_path, configured_fw_name, maxSrcLen);
        }
    } else {
        printf("Invalid DeviceLCInfo : 0x%x\n", deviceLcInfo);
        return 1;
    }

    printf("Referring FW path..........: %s", default_fw_path);
    if (gImg && (NULL == (gOpts.fImg = fopen(default_fw_path, "rb")))) {
        printf("ERROR: Cannot open %s file for reading!\n", gOpts.imgFile);
        return 1;
    } else {
        printf("FW FILE OPEN SUCCESS....\n");
        gImg = true;
        // gOpts.capture = Capture_Apdu_With_Dummy_Miso;
    }

    if (Capture_Off != gOpts.capture) {
        printf("Not Capture_Off.....\n");
        if (NULL == (gOpts.fMosi = fopen(gOpts.mosiFile, "wb"))) {
            printf("ERROR: Cannot open %s file for writing!\n", gOpts.mosiFile);
            return 1;
        }

        if (NULL == (gOpts.fMiso = fopen(gOpts.misoFile, "wb"))) {
            printf("ERROR: Cannot open %s file for writing!\n", gOpts.misoFile);
            return 1;
        }

        if (Capture_Apdu_With_Dummy_Miso == gOpts.capture) {
            memset(gDummyMiso, 0xEE, sizeof(gDummyMiso));
        }
    }

    return 0;
}

static void cleanup(void) {
    ioctl((intptr_t)tPalConfig.pDevHandle, SRXXX_SET_FWD, 0);
    if (configured_fw_name != NULL) {
        free(configured_fw_name);
        memset(default_fw_path, '\0', sizeof(char) * 256);
        strcpy(default_fw_path, "/lib/firmware/nxp/");
    }
    if (NULL != gOpts.fImg) {
        printf("CLSOIGN\n");
        fclose(gOpts.fImg);
    }

    if (NULL != gOpts.fMosi) {
        fclose(gOpts.fMosi);
    }

    if (NULL != gOpts.fMiso) {
        fclose(gOpts.fMiso);
    }
}
phHbci_Status_t phHbci_GetStatus(void) {
    printf("phHbci_GetStatus Enter\n");
    phHbci_Status_t ret = phHbci_Failure;

    gphHbci_MosiApdu.len = 0;

    if (phHbci_Success != (ret = phHbci_PutApdu((uint8_t *)&gphHbci_MosiApdu, PHHBCI_LEN_HDR))) {
        return ret;
    }
    if (phHbci_Success != (ret = phHbci_GetApdu((uint8_t *)&gphHbci_MisoApdu, PHHBCI_LEN_HDR))) {
        return ret;
    }

    return phHbci_Success;
}

phHbci_Status_t phHbci_GeneralStatus(phHbci_General_Command_t mode) {
    printf("phHbci_GeneralStatus Enter\n");
    switch (gphHbci_MisoApdu.cls) {
    case (uint8_t)phHbci_Class_General | (uint8_t)phHbci_SubClass_Answer:
        switch (gphHbci_MisoApdu.ins) {
        case phHbci_General_Ans_HBCI_Ready:
            if (!mode) {
                return phHbci_Success;
            }

            printf("ERROR: Unexpected General Status 0x%02x In Mode 0x%02x\n", gphHbci_MisoApdu.ins, mode);
            break;

        case phHbci_General_Ans_Mode_Patch_ROM_Ready:
            if (phHbci_General_Cmd_Mode_Patch_ROM == mode) {
                return phHbci_Success;
            }

            printf("ERROR: Unexpected General Status 0x%02x In Mode 0x%02x\n", gphHbci_MisoApdu.ins, mode);
            break;

        case phHbci_General_Ans_Mode_HIF_Image_Ready:
            if (phHbci_General_Cmd_Mode_HIF_Image == mode) {
                return phHbci_Success;
            }

            printf("ERROR: Unexpected General Status 0x%02x In Mode 0x%02x\n", gphHbci_MisoApdu.ins, mode);
            break;

        case phHbci_General_Ans_HBCI_Fail:
        case phHbci_General_Ans_Boot_Autoload_Fail:
        case phHbci_General_Ans_Boot_GPIOConf_CRC_Fail:
        case phHbci_General_Ans_Boot_TRIM_CRC_Fail:
        case phHbci_General_Ans_Boot_GPIOTRIM_CRC_Fail:
            printf("ERROR: HBCI Interface Failed With 0x%02x\n", gphHbci_MisoApdu.ins);
            break;

        case phHbci_General_Ans_Mode_Patch_ROM_Fail:
            printf("ERROR: Patch ROM Mode Failed!\n");
            break;

        case phHbci_General_Ans_Mode_HIF_Image_Fail:
            printf("ERROR: HIF Image Mode Failed!\n");
            break;

        default:
            printf("ERROR: Unknown General Status 0x%02x\n", gphHbci_MisoApdu.ins);
            break;
        }
        break;

    case (uint8_t)phHbci_Class_General | (uint8_t)phHbci_SubClass_Ack:
        switch (gphHbci_MisoApdu.ins) {
        case phHbci_Invlaid_Class:
            printf("ERROR: Invalid Class Error From Slave!\n");
            break;

        case phHbci_Invalid_Instruction:
            printf("ERROR: Invalid Instruction Error From Slave!\n");
            break;

        default:
            printf("ERROR: Unexpected Instruction From Slave 0x%02x\n", gphHbci_MisoApdu.ins);
            break;
        }
        break;

    default:
        printf("ERROR: Unknown General Class 0x%02x\n", gphHbci_MisoApdu.cls);
        break;
    }

    return phHbci_Failure;
}

phHbci_Status_t phHbci_QueryInfo(uint8_t *pInfo, uint32_t *pInfoSz, uint32_t maxSz, bool matchMaxSz) {
    printf("phHbci_QueryInfo Enter\n");
    uint8_t expCls, expIns;
    uint16_t lrc, dataSz, payloadSz, segment;
    phHbci_Status_t ret = phHbci_Failure;

    if (maxSz > PHHBCI_MAX_LEN_DATA_MISO) {
        printf("ERROR: Info Size Cannot Be Greater Than %u Bytes!\n", PHHBCI_MAX_LEN_DATA_MISO);
        return phHbci_Failure;
    }

    expCls = (gphHbci_MosiApdu.cls & (uint8_t)PHHBCI_CLASS_MASK) | phHbci_SubClass_Answer;
    expIns = gphHbci_MosiApdu.ins;

    gphHbci_MosiApdu.len = 0;

    if (phHbci_Success != (ret = phHbci_PutApdu((uint8_t *)&gphHbci_MosiApdu, PHHBCI_LEN_HDR))) {
        return ret;
    }

    if (phHbci_Success != (ret = phHbci_GetApdu((uint8_t *)&gphHbci_MisoApdu, PHHBCI_LEN_HDR))) {
        return ret;
    }

    payloadSz = gphHbci_MisoApdu.len;
    segment = payloadSz & PHHBCI_APDU_SEG_FLAG;

    if (!segment) {
        lrc = payloadSz ? PHHBCI_LEN_LRC : 0;
        dataSz = payloadSz - lrc;

        if (!dataSz) {
            printf("ERROR: No Info From Slave!\n");
            return phHbci_Failure;
        }
    }

    gphHbci_MosiApdu.cls = (uint8_t)phHbci_Class_General | (uint8_t)phHbci_SubClass_Ack;
    gphHbci_MosiApdu.ins = (uint8_t)phHbci_Valid_APDU;

    if (gphHbci_MisoApdu.cls != expCls) {
        printf("ERROR: Invalid Class - Exp 0x%02x, Got 0x%02x\n", expCls, gphHbci_MisoApdu.cls);
        gphHbci_MosiApdu.ins = phHbci_Invlaid_Class;
    } else if (gphHbci_MisoApdu.ins != expIns) {
        printf("ERROR: Invalid Instruction - Exp 0x%02x, Got 0x%02x\n", expIns, gphHbci_MisoApdu.ins);
        gphHbci_MosiApdu.ins = phHbci_Invalid_Instruction;
    } else if (segment) {
        printf("ERROR: Invalid Payload Length!\n");
        gphHbci_MosiApdu.ins = phHbci_Invalid_Segment_Length;
    } else if (dataSz > maxSz) {
        printf("ERROR: Total Size (%u) Greater Than Max. Size (%u)!\n", dataSz, maxSz);
        gphHbci_MosiApdu.ins = phHbci_Invalid_Segment_Length;
    } else if (matchMaxSz && (dataSz != maxSz)) {
        printf("ERROR: Total Size (%u) Not Equal To Expected Size (%u)!\n", dataSz, maxSz);
        gphHbci_MosiApdu.ins = phHbci_Invalid_Segment_Length;
    }

    if (phHbci_Success != (ret = phHbci_PutApdu((uint8_t *)&gphHbci_MosiApdu, PHHBCI_LEN_HDR))) {
        return ret;
    }

    if (gphHbci_MosiApdu.ins & PHHBCI_ERROR_STATUS_MASK) {
        return phHbci_Failure;
    }

    if (phHbci_Success != (ret = phHbci_GetApdu((uint8_t *)gphHbci_MisoApdu.payload, payloadSz))) {
        return ret;
    }

    if (gphHbci_MisoApdu.payload[dataSz] != phHbci_CalcLrc((uint8_t *)&gphHbci_MisoApdu, PHHBCI_LEN_HDR + dataSz)) {
        printf("ERROR: Invalid LRC!\n");
        return phHbci_Failure;
    }

    memcpy(&pInfo[*pInfoSz], gphHbci_MisoApdu.payload, dataSz);
    *pInfoSz += dataSz;

    return phHbci_Success;
}

phHbci_Status_t phHbci_GetGeneralInfo(uint8_t *pInfo, uint32_t *pInfoSz) {
    printf("phHbci_GetGeneralInfo\n");
    if (gphHbci_MosiApdu.cls != ((uint8_t)phHbci_Class_General | (uint8_t)phHbci_SubClass_Query)) {
        printf("ERROR: Invalid General Info Class = 0x%02x\n", gphHbci_MosiApdu.cls);
        return phHbci_Failure;
    }

    switch (gphHbci_MosiApdu.ins) {
    case phHbci_General_Qry_Chip_ID:
        return phHbci_QueryInfo(pInfo, pInfoSz, PHHBCI_HELIOS_CHIP_ID_SZ, true);

    case phHbci_General_Qry_Helios_ID:
        return phHbci_QueryInfo(pInfo, pInfoSz, PHHBCI_HELIOS_ID_SZ, true);

        // case phHbci_General_Qry_CA_Root_Pub_Key:
        //     return phHbci_QueryInfo(pInfo, pInfoSz, PHHBCI_HELIOS_CA_ROOT_PUB_KEY_SZ, true);

    case phHbci_General_Qry_NXP_Pub_Key:
        return phHbci_QueryInfo(pInfo, pInfoSz, PHHBCI_HELIOS_NXP_PUB_KEY_SZ, true);

    case phHbci_General_Qry_ROM_Version:
        return phHbci_QueryInfo(pInfo, pInfoSz, PHHBCI_HELIOS_ROM_VERSION_SZ, true);

    case phHbci_General_Qry_OTP_AutoLoad_Info:
        return phHbci_QueryInfo(pInfo, pInfoSz, PHHBCI_HELIOS_OTP_AUTOLOAD_INFO_SZ, true);
    default:
        printf("ERROR: Undefined General Query = 0x%02x\n", gphHbci_MosiApdu.ins);
        return phHbci_Failure;
    }

    return phHbci_Success;
}

phHbci_Status_t phHbci_GetInfo(uint8_t *pInfo, uint32_t *pInfoSz) {
    printf("phHbci_GetInfo Enter\n");
    switch (gphHbci_MosiApdu.cls) {
    case (uint8_t)phHbci_Class_General | (uint8_t)phHbci_SubClass_Query:
        return phHbci_GetGeneralInfo(pInfo, pInfoSz);
        break;

    default:
        printf("ERROR: No Info Defined For Class = 0x%02x\n", gphHbci_MosiApdu.cls);
        return phHbci_Failure;
    }

    return phHbci_Success;
}

phHbci_Status_t phHbci_PutCommand(uint8_t *pImg, uint32_t imgSz) {
    printf("phHbci_PutCommand Enter\n");
    uint8_t ackCls, ackIns;
    uint16_t lrc, dataSz, payloadSz;
    phHbci_Status_t ret = phHbci_Failure;

    ackCls = (uint8_t)phHbci_Class_General | (uint8_t)phHbci_SubClass_Ack;
    ackIns = (uint8_t)phHbci_Valid_APDU;

    do {
        // printf("fwd while loop...imgSz: %d\n",imgSz);
        if (imgSz > PHHBCI_MAX_LEN_DATA_MOSI) {
            dataSz = PHHBCI_MAX_LEN_DATA_MOSI;
            payloadSz = PHHBCI_APDU_SEG_FLAG;
        } else {
            lrc = imgSz ? PHHBCI_LEN_LRC : 0;
            dataSz = imgSz;
            payloadSz = dataSz + lrc;
        }
        // printf("dataSz : %d\n",dataSz);
        gphHbci_MosiApdu.len = payloadSz;
        usleep(1);

        if (phHbci_Success != (ret = phHbci_PutApdu((uint8_t *)&gphHbci_MosiApdu, PHHBCI_LEN_HDR))) {
            return ret;
        }
        usleep(1);
        if (phHbci_Success != (ret = phHbci_GetApdu((uint8_t *)&gphHbci_MisoApdu, PHHBCI_LEN_HDR))) {
            return ret;
        }

        if ((gphHbci_MisoApdu.cls != ackCls) || (gphHbci_MisoApdu.ins != ackIns)) {
            printf("ERROR: NACK (CLS = 0x%02x, INS = 0x%02x)\n", gphHbci_MisoApdu.cls, gphHbci_MisoApdu.ins);
            return phHbci_Failure;
        }

        if (dataSz) {
            // printf("dataSz is not zero......");
            memcpy(gphHbci_MosiApdu.payload, pImg, dataSz);
            gphHbci_MosiApdu.payload[dataSz] = phHbci_CalcLrc((uint8_t *)&gphHbci_MosiApdu, PHHBCI_LEN_HDR + dataSz);

            pImg += dataSz;
            imgSz -= dataSz;
            payloadSz = dataSz + PHHBCI_LEN_LRC;
            if (chip_id == PHHBCI_FW_B2_VERSION) {
                usleep(250);
            }

            if (phHbci_Success != (ret = phHbci_PutApdu((uint8_t *)gphHbci_MosiApdu.payload, payloadSz))) {
                return ret;
            }

            if (phHbci_Success != (ret = phHbci_GetApdu((uint8_t *)&gphHbci_MisoApdu, PHHBCI_LEN_HDR))) {
                return ret;
            }

            if ((gphHbci_MisoApdu.cls != ackCls) || (gphHbci_MisoApdu.ins != ackIns)) {
                printf("ERROR: NACK (CLS = 0x%02x, INS = 0x%02x)\n", gphHbci_MisoApdu.cls, gphHbci_MisoApdu.ins);
                return phHbci_Failure;
            }
        }
    } while (imgSz);

    return phHbci_Success;
}

static phHbci_Status_t phHbci_MasterPatchROM(uint8_t *pImg, uint32_t imgSz) {
    printf("phHbci_MasterPatchROM enter");
    phHbci_Status_t ret = phHbci_Failure;

    gphHbci_MosiApdu.cls = (uint8_t)phHbci_Class_General | (uint8_t)phHbci_SubClass_Query;
    gphHbci_MosiApdu.ins = (uint8_t)phHbci_General_Qry_Status;

    while (1) {
        if (phHbci_Success != (ret = phHbci_GetStatus())) {
            return ret;
        }

        switch (gphHbci_MisoApdu.cls) {
        case (uint8_t)phHbci_Class_General | (uint8_t)phHbci_SubClass_Answer:
        case (uint8_t)phHbci_Class_General | (uint8_t)phHbci_SubClass_Ack:
            if (phHbci_Success != (ret = phHbci_GeneralStatus(phHbci_General_Cmd_Mode_Patch_ROM))) {
                return ret;
            }

            gphHbci_MosiApdu.cls = (uint8_t)phHbci_Class_Patch_ROM | (uint8_t)phHbci_SubClass_Command;
            gphHbci_MosiApdu.ins = (uint8_t)phHbci_Patch_ROM_Cmd_Download_Patch;

            /* Reset GPIO event flag */
            // cppResetGPIOEvent();

            if (phHbci_Success != (ret = phHbci_PutCommand(pImg, imgSz))) {
                return ret;
            }

            /* Wait for GPIO event */
            /*if (0 > cppWaitForGPIOEvent(PHHBCI_GPIO_TIMEOUT_MS))
            {
                printf("ERROR: GPIO notification timeout!\n");
            }*/

            gphHbci_MosiApdu.cls = (uint8_t)phHbci_Class_Patch_ROM | (uint8_t)phHbci_SubClass_Query;
            gphHbci_MosiApdu.ins = (uint8_t)phHbci_Patch_ROM_Qry_Patch_Status;
            break;

        case (uint8_t)phHbci_Class_Patch_ROM | (uint8_t)phHbci_SubClass_Answer:
            switch (gphHbci_MisoApdu.ins) {
            case phHbci_Patch_ROM_Ans_Patch_Success:
                printf("Patch ROM Transfer Complete.\n");
                ret = phHbci_Success;
                break;

            case phHbci_Patch_ROM_Ans_File_Too_Large:
            case phHbci_Patch_ROM_Ans_Invalid_Patch_File_Marker:
            case phHbci_Patch_ROM_Ans_Too_Many_Patch_Table_Entries:
            case phHbci_Patch_ROM_Ans_Invalid_Patch_Code_Size:
            case phHbci_Patch_ROM_Ans_Invalid_Global_Patch_Marker:
            case phHbci_Patch_ROM_Ans_Invalid_Signature_Size:
            case phHbci_Patch_ROM_Ans_Invalid_Signature:
                printf("EROOR: Patch ROM Transfer Failed With 0x%02x!\n", gphHbci_MisoApdu.ins);
                ret = phHbci_Failure;
                break;

            default:
                printf("ERROR: Unknown Patch ROM Status 0x%02x\n", gphHbci_MisoApdu.ins);
                ret = phHbci_Failure;
                break;
            }
            return ret;

        default:
            printf("ERROR: Unknown Class 0x%02x\n", gphHbci_MisoApdu.cls);
            return phHbci_Failure;
        }
    }

    return phHbci_Success;
}

static phHbci_Status_t phHbci_MasterHIFImage(uint8_t *pImg, uint32_t imgSz) {
    printf("phHbci_MasterHIFImage enter");
    phHbci_Status_t ret = phHbci_Failure;

    gphHbci_MosiApdu.cls = (uint8_t)phHbci_Class_General | (uint8_t)phHbci_SubClass_Query;
    gphHbci_MosiApdu.ins = (uint8_t)phHbci_General_Qry_Status;

    while (1) {
        if (phHbci_Success != (ret = phHbci_GetStatus())) {
            return ret;
        }

        switch (gphHbci_MisoApdu.cls) {
        case (uint8_t)phHbci_Class_General | (uint8_t)phHbci_SubClass_Answer:
        case (uint8_t)phHbci_Class_General | (uint8_t)phHbci_SubClass_Ack:
            if (phHbci_Success != (ret = phHbci_GeneralStatus(phHbci_General_Cmd_Mode_HIF_Image))) {
                return ret;
            }

            gphHbci_MosiApdu.cls = (uint8_t)phHbci_Class_HIF_Image | (uint8_t)phHbci_SubClass_Command;
            gphHbci_MosiApdu.ins = (uint8_t)phHbci_HIF_Image_Cmd_Download_Image;

            /* Reset GPIO event flag */
            // cppResetGPIOEvent();

            if (phHbci_Success != (ret = phHbci_PutCommand(pImg, imgSz))) {
                return ret;
            }

            /* Wait for GPIO event */
            /* if (0 > cppWaitForGPIOEvent(PHHBCI_GPIO_TIMEOUT_MS))
             {
                 printf("ERROR: GPIO notification timeout!\n");
             }*/
            usleep(100000);

            gphHbci_MosiApdu.cls = (uint8_t)phHbci_Class_HIF_Image | (uint8_t)phHbci_SubClass_Query;
            gphHbci_MosiApdu.ins = (uint8_t)phHbci_HIF_Image_Qry_Image_Status;
            break;

        case (uint8_t)phHbci_Class_HIF_Image | (uint8_t)phHbci_SubClass_Answer:
            switch (gphHbci_MisoApdu.ins) {
            case phHbci_HIF_Image_Ans_Image_Success:
                printf("HIF Image Transfer Complete.\n");
                /*Check FW download throughput measurement*/
                // ioctl((intptr_t)tPalConfig.pDevHandle, SRXXX_GET_THROUGHPUT, 0);
                return phHbci_Success;

            case phHbci_HIF_Image_Ans_Header_Too_Large:
            case phHbci_HIF_Image_Ans_Header_Parse_Error:
            case phHbci_HIF_Image_Ans_Invalid_Cipher_Type_Crypto:
            case phHbci_HIF_Image_Ans_Invalid_Cipher_Type_Hash:
            case phHbci_HIF_Image_Ans_Invalid_Cipher_Type_Curve:
            case phHbci_HIF_Image_Ans_Invalid_ECC_Key_Length:
            case phHbci_HIF_Image_Ans_Invalid_Payload_Description:
            case phHbci_HIF_Image_Ans_Invalid_Firmware_Version:
            case phHbci_HIF_Image_Ans_Invalid_ECID_Mask:
            case phHbci_HIF_Image_Ans_Invalid_ECID_Value:
            case phHbci_HIF_Image_Ans_Invalid_Encrypted_Payload_Hash:
            case phHbci_HIF_Image_Ans_Invalid_Header_Signature:
            case phHbci_HIF_Image_Ans_Install_Settings_Too_Large:
            case phHbci_HIF_Image_Ans_Install_Settings_Parse_Error:
            case phHbci_HIF_Image_Ans_Payload_Too_Large:
            case phHbci_HIF_Image_Ans_Quickboot_Settings_Parse_Error:
            case phHbci_HIF_Image_Ans_Invalid_Static_Hash:
            case phHbci_HIF_Image_Ans_Invalid_Dynamic_Hash:
            case phHbci_HIF_Image_Ans_Execution_Settings_Parse_Error:
            case phHbci_HIF_Image_Ans_Key_Read_Error:
                printf("EROOR: HIF Image Transfer Failed With 0x%02x!\n", gphHbci_MisoApdu.ins);
                return phHbci_Failure;

            default:
                printf("ERROR: Unknown HIF Status 0x%02x\n", gphHbci_MisoApdu.ins);
                return phHbci_Failure;
            }
            break;

        default:
            printf("ERROR: Unknown Class 0x%02x\n", gphHbci_MisoApdu.cls);
            return phHbci_Failure;
        }
    }

    return phHbci_Success;
}

/*********************************************************************************************************************/
/*   GLOBAL FUNCTIONS                                                                                                */
/*********************************************************************************************************************/
phHbci_Status_t phHbci_Master(phHbci_General_Command_t mode, uint8_t *pImg, uint32_t imgSz) {
    printf("phHbci_Master Enter\n");
    //    uint8_t             info[PHHBCI_MAX_LEN_DATA_MISO];
    //    uint32_t            infoSz = 0;
    phHbci_Status_t ret = phHbci_Failure;

    gphHbci_MosiApdu.cls = (uint8_t)phHbci_Class_General | (uint8_t)phHbci_SubClass_Query;
    gphHbci_MosiApdu.ins = (uint8_t)phHbci_General_Qry_Status;

    if (phHbci_Success != (ret = phHbci_GetStatus())) {
        return ret;
    }

    if (phHbci_Success != (ret = phHbci_GeneralStatus((phHbci_General_Command_t)0))) {
        return ret;
    }

    gphHbci_MosiApdu.cls = (uint8_t)phHbci_Class_General | (uint8_t)phHbci_SubClass_Command;
    gphHbci_MosiApdu.ins = (uint8_t)mode;
    printf("STARTING FW DOWNLOAD.....\n");
    if (phHbci_Success != (ret = phHbci_PutCommand(pImg, 0))) {
        return ret;
    }

    switch (mode) {
    case phHbci_General_Cmd_Mode_Patch_ROM:
        return phHbci_MasterPatchROM(pImg, imgSz);

    case phHbci_General_Cmd_Mode_HIF_Image:
        return phHbci_MasterHIFImage(pImg, imgSz);

    default:
        printf("ERROR: Undefined mode 0x%02x\n", mode);
        break;
    }

    return phHbci_Failure;
}

/*********************************************************************************************************************/
/*   GLOBAL FUNCTIONS                                                                                                */
/*********************************************************************************************************************/
uint8_t phHbci_CalcLrc(uint8_t *pBuf, uint16_t bufSz) {
    // printf("phHbci_CalcLrc....\n");
    uint8_t lrc = 0;
    uint16_t i;

    if (!pBuf || !bufSz)
        return lrc;

    /* ISO 1155:1978 Information processing -- Use of longitudinal parity to detect errors in information messages */
    for (i = 0; i < bufSz; i++) {
        lrc += *pBuf++;
    }

    lrc ^= 0xFF;
    lrc += 1;

    return lrc;
}
/*********************************************************************************************************************/
/*   GLOBAL FUNCTIONS                                                                                                */
/*********************************************************************************************************************/
phHbci_Status_t phHbci_GetApdu(uint8_t *pApdu, uint16_t sz) {
    //    printf("phHbci_GetApdu Enter\n");
    uint16_t ret;
    int ret_Read;
    if (sz == 0 || sz > PHHBCI_MAX_LEN_PAYLOAD_MISO) {
        printf("ERROR: phHbci_GetApdu data len is 0 or greater than max palyload length supported\n");
        return phHbci_Failure;
    }
    ret_Read = read((intptr_t)tPalConfig.pDevHandle, (void *)pApdu, (sz));
    if (ret_Read < 0) {
        printf("ERROR: Get APDU %u bytes failed with error %i!\n", sz, errno);
        return phHbci_Failure;
    }

    if (is_fw_download_log_enabled == 0x01)
        phNxpUciHal_print_packet("RECV", pApdu, ret_Read);

    switch (gOpts.capture) {
    case Capture_Apdu_With_Dummy_Miso:
        if (sz != (ret = fwrite(gDummyMiso, sizeof(uint8_t), sz, gOpts.fMosi))) {
            printf("ERROR: %s dummy write returned %d, expected %d\n", gOpts.mosiFile, ret, sz);
        }
        if (sz != (ret = fwrite(pApdu, sizeof(uint8_t), sz, gOpts.fMiso))) {
            printf("ERROR: %s write returned %d, expected %d\n", gOpts.misoFile, ret, sz);
        }
        break;
    case Capture_Apdu:
        if (sz != (ret = fwrite(pApdu, sizeof(uint8_t), sz, gOpts.fMiso))) {
            printf("ERROR: %s write returned %d, expected %d\n", gOpts.misoFile, ret, sz);
        }
        break;

    case Capture_Off:
    default:
        break;
    }
    // printf("Rx --> 0X%x 0X%x 0X%x 0X%x.\n", pApdu[0], pApdu[1], pApdu[2],pApdu[3]);
    return phHbci_Success;
}

phHbci_Status_t phHbci_PutApdu(uint8_t *pApdu, uint16_t sz) {
    // printf("phHbci_PutApdu Enter\n");
    int ret;
    int numWrote = 0;
    if (is_fw_download_log_enabled == 0x01)
        phNxpUciHal_print_packet("SEND", pApdu, sz);

    ret = write((intptr_t)tPalConfig.pDevHandle, pApdu, sz);
    if (ret > 0) {
        numWrote += ret;
    } else if (ret == 0) {
        printf("_spi_write() EOF");
        return (phHbci_Status_t)1;
    } else {
        printf("_spi_write() errno : %x", ret);
        return (phHbci_Status_t)1;
    }
    switch (gOpts.capture) {
    case Capture_Apdu_With_Dummy_Miso:
    case Capture_Apdu:
        printf("Write dummy apdu......\n");
        ret = write((intptr_t)tPalConfig.pDevHandle, pApdu, sz);
        if (ret > 0) {
            numWrote += ret;
        } else if (ret == 0) {
            printf("_spi_write() EOF");
            return (phHbci_Status_t)1;
        } else {
            printf("_spi_write() errno : %x", ret);
            return (phHbci_Status_t)1;
        }
        /*if (sz != (ret = fwrite(pApdu, sizeof(uint8_t), sz, gOpts.fMosi)))
        {
            printf("ERROR: %s write returned %d, expected %d\n", gOpts.mosiFile, ret, sz);
        }*/
        break;

    case Capture_Off:
    default:
        break;
    }
    // printf("Tx-->0X%x 0X%x 0X%x 0X%x......\n", pApdu[0], pApdu[1], pApdu[2],pApdu[3]);
    return phHbci_Success;
}

phHbci_Status_t phHbci_GetChipIdInfo() {
    phHbci_Status_t ret = phHbci_Failure;
    uint8_t FwdExtndLenIndication = 0, totalBtyesToReadMsb = 0;
    uint16_t totalBtyesToRead = 0;
    uint8_t hbciData[PHHBCI_MAX_LEN_PAYLOAD_MISO];

    gphHbci_MosiApdu.cls = (uint8_t)phHbci_Class_General | (uint8_t)phHbci_SubClass_Query;
    gphHbci_MosiApdu.ins = (uint8_t)phHbci_General_Qry_Chip_ID;

    gphHbci_MosiApdu.len = 0;

    if (phHbci_Success != (ret = phHbci_PutApdu((uint8_t *)&gphHbci_MosiApdu, PHHBCI_LEN_HDR))) {
        return ret;
    }
    usleep(1);
    if (phHbci_Success != (ret = phHbci_GetApdu((uint8_t *)&hbciData[0], PHHBCI_LEN_HDR))) {
        return ret;
    }
    FwdExtndLenIndication = ((hbciData[PHHBCI_MODE_LEN_MSB_OFFSET] & 0xF0) >> 4);
    totalBtyesToReadMsb = (hbciData[PHHBCI_MODE_LEN_MSB_OFFSET] & 0x0F);
    totalBtyesToRead = (hbciData[PHHBCI_MODE_LEN_LSB_OFFSET] | (totalBtyesToReadMsb << 8));

    if (totalBtyesToRead == 0) {
        printf("ERROR: hbci data len is 0\n");
        return phHbci_Failure;
    }
    gphHbci_MosiApdu.cls = (uint8_t)phHbci_SubClass_Ack;
    gphHbci_MosiApdu.ins = (uint8_t)phHbci_SubClass_Query;
    gphHbci_MosiApdu.len = 0;

    if (phHbci_Success != (ret = phHbci_PutApdu((uint8_t *)&gphHbci_MosiApdu, PHHBCI_LEN_HDR))) {
        return ret;
    }
    usleep(1);

    if (phHbci_Success != (ret = phHbci_GetApdu((uint8_t *)&hbciData[0], totalBtyesToRead))) {
        return ret;
    }
    chip_id = hbciData[PHHBCI_MODE_CHIP_ID_OFFSET];
    printf("Recived ChipId = 0x%02x\n", chip_id);

    return phHbci_Success;
}

/******************************************************************************
 * Function         phHbci_GetDeviceLcInfo
 *
 * Description      This function is called to get the OTP Autoload Info
 * Returns          return 0 on success and -1
 *
 ******************************************************************************/
phHbci_Status_t phHbci_GetDeviceLcInfo() {
    phHbci_Status_t ret = phHbci_Failure;
    uint8_t FwdExtndLenIndication = 0, totalBtyesToReadMsb = 0;
    uint16_t totalBtyesToRead = 0;
    uint8_t hbciData[PHHBCI_MAX_LEN_PAYLOAD_MISO];

    gphHbci_MosiApdu.cls = (uint8_t)phHbci_Class_General | (uint8_t)phHbci_SubClass_Query;
    gphHbci_MosiApdu.ins = (uint8_t)phHbci_General_Qry_OTP_AutoLoad_Info;

    gphHbci_MosiApdu.len = 0;

    if (phHbci_Success != (ret = phHbci_PutApdu((uint8_t *)&gphHbci_MosiApdu, PHHBCI_LEN_HDR))) {
        return ret;
    }
    usleep(1);
    if (phHbci_Success != (ret = phHbci_GetApdu((uint8_t *)&hbciData[0], PHHBCI_LEN_HDR))) {
        return ret;
    }

    FwdExtndLenIndication = ((hbciData[PHHBCI_MODE_LEN_MSB_OFFSET] & 0xF0) >> 4);
    totalBtyesToReadMsb = (hbciData[PHHBCI_MODE_LEN_MSB_OFFSET] & 0x0F);
    totalBtyesToRead = (uint16_t)(hbciData[PHHBCI_MODE_LEN_LSB_OFFSET] | (totalBtyesToReadMsb << 8));

    if (totalBtyesToRead == 0) {
        printf("ERROR: hbci data len is 0\n");
        return phHbci_Failure;
    }
    gphHbci_MosiApdu.cls = (uint8_t)phHbci_SubClass_Ack;
    gphHbci_MosiApdu.ins = (uint8_t)phHbci_SubClass_Query;
    gphHbci_MosiApdu.len = 0;

    if (phHbci_Success != (ret = phHbci_PutApdu((uint8_t *)&gphHbci_MosiApdu, PHHBCI_LEN_HDR))) {
        return ret;
    }
    usleep(1);
    if (phHbci_Success != (ret = phHbci_GetApdu((uint8_t *)&hbciData[0], totalBtyesToRead))) {
        return ret;
    }

    deviceLcInfo = hbciData[PHHBCI_MODE_DEV_LIFE_CYCLE_INFO_OFFSET];
    printf("Recived devLifeCycleId = 0x%02x\n", deviceLcInfo);

    return phHbci_Success;
}

/******************************************************************************
 * Function         phNxpUciHal_fw_download
 *
 * Description      This function is called by jni when wired mode is
 *                  performed.First SR100 driver will give the access
 *                  permission whether wired mode is allowed or not
 *                  arg (0):
 * Returns          return 0 on success and -1 on fail, On success
 *                  update the acutual state of operation in arg pointer
 *
 ******************************************************************************/
int phNxpUciHal_fw_download() {
    uint8_t *pImg = gphHbci_ImgHelios;
    uint32_t imgSz = 0, maxSz, err = 0;
    unsigned long num = 0;
    phHbci_General_Command_t cmd;
    printf("phNxpUciHal_fw_download enter and FW download started.....\n");
    setOpts();

    if (ioctl((intptr_t)tPalConfig.pDevHandle, SRXXX_SET_FWD, 1) < 0) {
        printf("SRXXX_SET_FWD error \n");
        return 1;
    }
    /* Always display chip id information */
    is_fw_download_log_enabled = true;
    if (phHbci_Success != phHbci_GetDeviceLcInfo()) {
        printf("phHbci_GetDeviceLcInfo Failure!\n");
        return 1;
    }

    if (phHbci_Success != phHbci_GetChipIdInfo()) {
        printf("phHbci_GetChipIdInfo Failure!\n");
        return 1;
    }
    is_fw_download_log_enabled = false;

    if (getParamNumValue(NAME_UWB_FW_DOWNLOAD_LOG, &num, sizeof(num))) {
        is_fw_download_log_enabled = (uint8_t)num;
        printf("NAME_UWB_FW_DOWNLOAD_LOG: 0x%02x\n", is_fw_download_log_enabled);
    } else {
        printf("NAME_UWB_FW_DOWNLOAD_LOG: failed 0x%02x\n", is_fw_download_log_enabled);
    }
    if (init()) {
        printf("INIT Failed.....\n");
        cleanup();
        return 1;
    }

    switch (gOpts.mode) {
    case Mode_Patch_ROM:
        cmd = phHbci_General_Cmd_Mode_Patch_ROM;
        maxSz = PHHBCI_PATCHROM_MAX_IMAGE_SZ;
        break;

    case Mode_HIF_Image:
        cmd = phHbci_General_Cmd_Mode_HIF_Image;
        maxSz = PHHIF_MAX_IMAGE_SZ;
        break;

    default:
        printf("ERROR: Undefined Master Mode = %u\n", gOpts.mode);
        return 1;
    }
    if (gImg) {
        if (gOpts.fImg == NULL) {
            printf("Opening %s\n", default_fw_path);
            gOpts.fImg = fopen(default_fw_path, "rb");
            printf("Error %d \n", errno);
        }
        if (gOpts.fImg == NULL) {
            printf("Firmware file does not exist:");
            return phHbci_File_Not_found;
        }
        fseek(gOpts.fImg, 0, SEEK_END);
        imgSz = (uint32_t)ftell(gOpts.fImg);
        printf("FWD file size ftell returns: %d\n", imgSz);
        if (!imgSz || (maxSz < imgSz)) {
            printf("ERROR: %s image size (%d) not supported!\n", gOpts.imgFile, imgSz);
            cleanup();
            return 1;
        }

        rewind(gOpts.fImg);
    }
    printf("FWD file size: %d\n", imgSz);
    if (!gImg || (imgSz == fread(pImg, sizeof(uint8_t), imgSz, gOpts.fImg))) {
        if (cmd == phHbci_General_Cmd_Mode_HIF_Image) {
            printf("HIF Image mode.\n");
        }
        err = phHbci_Master(cmd, pImg, imgSz);
        if (phHbci_Success != err) {
            printf("Failure!\n");
            err = 1;
        }
    } else {
        printf("ERROR: Image read failed!\n");
        err = 1;
    }

    cleanup();
    return err;
}

void setDeviceHandle(int pDevHandle) {
    printf("Set the device handle!\n");

    if (pDevHandle == 0) {
        printf("device handle is NULL!\n");
    } else {
        tPalConfig.pDevHandle = (void *)((intptr_t)pDevHandle);
    }
}
