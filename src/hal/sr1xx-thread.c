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

#include <pthread.h>
#include <sys/time.h>

#include "sr1xx-dev.h"
#include "sr1xx-thread.h"
#include "sr1xx.h"
#include "uci_protocol.h"

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

pthread_mutex_t mutex_token = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond_token = PTHREAD_COND_INITIALIZER;

int read_ret;
UciPacketDecode packet_encode;
uint8_t *response_buffer;

pthread_t rx_thread;

static uint8_t uci_receive[UCI_MAX_DATA_LEN + UCI_PKT_HDR_LEN]; // Note can overflow

int sr1xx_send_cmd(sr1xx_dev *dev, const uint8_t *payload, size_t size, uint8_t *response, int ms) {
    int ret;
    int send_ret;
    struct timeval now;
    struct timespec timeout;
    long future_us;
    ControlPacket_view control_packet = {0};
    size_t index = 0;
    UciPacketDecode packet_decode = {0};

    if (!uci_parse_ControlPacket_view(payload, size, &index, &control_packet)) {
        return -EINVAL;
    }

    if (control_packet.mt != MESSAGETYPE_COMMAND) {
        return -EINVAL;
    }

    pthread_mutex_lock(&mutex);

    packet_encode = uci_decode(payload, size);
    printf("Expecting OID %X\n", packet_encode.oid);
    read_ret = 0;

    if (packet_encode.ret < 0) {
        pthread_mutex_unlock(&mutex);
        printf("Encode failure\n");
        return packet_encode.ret;
    }

    if (response) {
        response_buffer = response;
    } else {
        response_buffer = NULL;
    }

    do {
        gettimeofday(&now, NULL);
        /* microsecond precision */
        future_us = now.tv_usec + (int)ms * 1000;
        timeout.tv_nsec = (future_us % 1000000) * 1000;
        timeout.tv_sec = now.tv_sec + future_us / 1000000;

        send_ret = sr1xx_dev_write(dev, payload, size);

        if (send_ret > 0) {
            ret = pthread_cond_timedwait(&cond, &mutex, &timeout);
        }

    } while (read_ret == -EAGAIN);

    if (read_ret > 0) {
        packet_decode = uci_decode(uci_receive, read_ret);
        ret = read_ret;
    }

    pthread_mutex_unlock(&mutex);

    if (packet_decode.status != STATUS_OK) {
        return -EINVAL;
    }

    if (ret == -ETIMEDOUT) {
        printf("TIMEOUT !!!!!!!!!!!!!!\n");
    }

    return ret;
}

int sr1xx_wait_for_session_token() {
    int ret;
    struct timeval now;
    struct timespec timeout;

    pthread_mutex_lock(&mutex_token);

    gettimeofday(&now, NULL);
    /* microsecond precision */
    long future_us = now.tv_usec + (int)500 * 1000;
    timeout.tv_nsec = (future_us % 1000000) * 1000;
    timeout.tv_sec = now.tv_sec + future_us / 1000000;

    ret = pthread_cond_timedwait(&cond_token, &mutex_token, &timeout);
    pthread_mutex_unlock(&mutex_token);
    return ret;
}

static void sr1xx_processs_session_ntf(sr1xx_dev *dev, size_t length) {
    size_t index = 0;
    bool parse_success;
    ControlPacket_view control_packet = {0};
    SessionConfigPacket_view session_config_packet = {0};
    SessionStatusNtf_view session_status = {0};

    parse_success = uci_parse_ControlPacket_view(uci_receive, length, &index, &control_packet);

    if (!parse_success) {
        printf("Error parsing ControlPacket\n");
    }

    parse_success =
        uci_parse_SessionConfigPacket_view(&control_packet, uci_receive, length, &index, &session_config_packet);

    if (!parse_success) {
        printf("Error parsing SessionConfigPacket\n");
    }

    parse_success =
        uci_parse_SessionStatusNtf_view(&session_config_packet, uci_receive, length, &index, &session_status);

    if (!parse_success) {
        printf("Error parsing SessionStatusNtf\n");
    }

    if (session_status.reason_code == 0) {
        printf("Update Token\n");
        pthread_mutex_lock(&mutex_token);
        dev->session_id.session = session_status.session_token;
        pthread_cond_signal(&cond_token);
        pthread_mutex_unlock(&mutex_token);
    } else {
        printf("SessionStatusNtf error Reason %s\n", ReasonCodeText(session_status.reason_code));
    }
}

void *sr1xx_rx_thread(void *arg) {
    int ret;
    sr1xx_dev *dev = (sr1xx_dev *)arg;
    UciPacketDecode packet_decode;

    dev->thread_running = true;

    while (1) {
        ret = sr1xx_dev_read(dev, uci_receive, sizeof(uci_receive));

        if (ret > 0) {
            uci_print(uci_receive, ret); // TODO make this configurable
            packet_decode = uci_decode(uci_receive, ret);

            pthread_mutex_lock(&mutex);
            if (packet_decode.ret == -EAGAIN) {
                read_ret = -EAGAIN;
                pthread_cond_signal(&cond);
            } else if (packet_decode.gid == packet_encode.gid && packet_decode.oid == packet_encode.oid) {
                if (response_buffer) {
                    memcpy(response_buffer, uci_receive, ret);
                }
                read_ret = ret;
                pthread_cond_signal(&cond);
            } else if (packet_decode.gid == GROUPID_SESSION_CONFIG &&
                       packet_decode.oid == SESSIONCONFIGOPCODEID_STATUS) {
                sr1xx_processs_session_ntf(dev, ret);
            }
            pthread_mutex_unlock(&mutex);

            if (dev->recv_callback) {
                dev->recv_callback(uci_receive, ret, dev->user_data);
            }

        } else {
            printf("Receive error %i\n", ret);
        }
    }

    // TODO create exit condition
    dev->thread_running = false;
}

int sr1xx_thread_init(sr1xx_dev *dev) {
    pthread_create(&rx_thread, NULL, &sr1xx_rx_thread, dev);
}
