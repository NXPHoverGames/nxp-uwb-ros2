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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <hal/uci_protocol.h>
#include <sr1xx_driver.h>

#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "uwb_msgs/msg/ultra_wide_band_ranging.hpp"

using namespace std::chrono_literals;

const std::vector<int64_t> default_mac_address = {0x1111};
const std::vector<int64_t> default_dst_mac_address = {0x2222};
const std::vector<double> default_position = {};
const bool default_mac_is_extended = false;
const int64_t default_channel_id = 0x9; // Modifying the channel, requires to update calib in sr1xx-core.c
const int64_t default_session_id = 1;
const int64_t default_ranging_interval = 50; // in ms, default 20 Hz

class RangingPublisher : public rclcpp::Node {
  public:
    /**
     * @brief Constructor for the ROS2 RangingPublisher class, initializes the node and sets up parameters for the SR1XX
     * driver.
     *
     * The SR1XX driver is responsible for handling Ultra-Wideband (UWB) communication, specifically designed for NXP's
     * SR1XX series SoCs. It manages SPI communication, interrupts, and UCI packet processing.
     *
     * The `sr1xx_driver_register_callback` function registers a callback function (`sr1xx_driver_recv`) that processes
     * incoming measurements and produces data for the ROS2 publisher. This publisher then publishes
     * `uwb_msgs::msg::UltraWideBandRanging` messages.
     */
    RangingPublisher() : Node("sr1xx_node"), count_(0) {
        /**
         * @brief Parameters for configuring the UWB device.
         *
         * | Parameter          | Description                          | Default Value     |
         * |--------------------|--------------------------------------|-------------------|
         * | role               | Role of the UWB device               | twr_initiator     |
         * | device_mac_address | MAC address of the device            | [0x1111]          |
         * | dst_mac_address    | Destination MAC addresses            | [0x2222]          |
         * | mac_is_extended    | Whether MAC address is extended      | false             |
         * | channel_id         | UWB channel ID                       | 0x9               |
         * | session_id         | UWB session ID                       | 1                 |
         * | ranging_interval   | Ranging interval in milliseconds     | 50                |
         * | position           | Device position (used for anchors)   | []                |
         */

        this->declare_parameter("role", TWR_INITIATOR);
        this->declare_parameter("device_mac_address", default_mac_address);
        this->declare_parameter("dst_mac_address", default_dst_mac_address);
        this->declare_parameter("mac_is_extended", default_mac_is_extended);
        this->declare_parameter("channel_id", default_channel_id);
        this->declare_parameter("session_id", default_session_id);
        this->declare_parameter("ranging_interval", default_ranging_interval);
        this->declare_parameter<std::vector<double>>("position", default_position);

        int ret;
        std::string role = this->get_parameter("role").as_string();
        std::vector<int64_t> device_mac_address = this->get_parameter("device_mac_address").as_integer_array();
        std::vector<int64_t> dst_mac_address = this->get_parameter("dst_mac_address").as_integer_array();
        bool mac_is_extended = this->get_parameter("mac_is_extended").as_bool();
        uint8_t channel_id = (uint8_t)this->get_parameter("channel_id").as_int();
        uint32_t session_id = (uint32_t)this->get_parameter("session_id").as_int();
        uint32_t ranging_interval = (uint32_t)this->get_parameter("ranging_interval").as_int();
        RCLCPP_INFO(this->get_logger(), "Interval %i", ranging_interval);

        if (mac_is_extended) {
            ret = sr1xx_driver_init(&dev, channel_id, session_id, mac_is_extended, (uint8_t *)device_mac_address.data(),
                                    (uint8_t *)dst_mac_address.data(), dst_mac_address.size(), ranging_interval);

            for (auto i = device_mac_address.begin(); i != device_mac_address.end(); ++i) {
                RCLCPP_INFO(this->get_logger(), "extended Device mac 0x%04x", *i);
            }

            RCLCPP_INFO(this->get_logger(), "Number of controlees %i", dst_mac_address.size());

            for (auto i = dst_mac_address.begin(); i != dst_mac_address.end(); ++i) {
                RCLCPP_INFO(this->get_logger(), "extended Dest. mac 0x%04x", *i);
            }

        } else {
            std::vector<uint16_t> short_device_mac_address(device_mac_address.begin(), device_mac_address.end());
            std::vector<uint16_t> short_dst_mac_address(dst_mac_address.begin(), dst_mac_address.end());

            ret = sr1xx_driver_init(&dev, channel_id, session_id, mac_is_extended,
                                    (uint8_t *)short_device_mac_address.data(), (uint8_t *)short_dst_mac_address.data(),
                                    short_dst_mac_address.size(), ranging_interval);

            for (auto i = device_mac_address.begin(); i != device_mac_address.end(); ++i) {
                RCLCPP_INFO(this->get_logger(), "short Device mac 0x%04x", *i);
            }

            RCLCPP_INFO(this->get_logger(), "Number of controlees %i", short_dst_mac_address.size());

            for (auto i = dst_mac_address.begin(); i != dst_mac_address.end(); ++i) {
                RCLCPP_INFO(this->get_logger(), "short Dest. mac 0x%04x", *i);
            }
        }

        if (this->has_parameter("position")) {
            std::vector<double> position;

            // Try to get the parameter value
            if (this->get_parameter("position", position)) {
                if (position.size() == 3) {
                    RCLCPP_INFO(this->get_logger(), "Position parameter is set: x=%f, y=%f, z=%f", position[0],
                                position[1], position[2]);
                    sr1xx_set_anchor_position(&dev, position[0], position[1], position[2]);
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to get the 'position' parameter.");
            }
        }

        if (ret >= 0) {
            /*
             * Suported UWB Roles:
             * twr_initiator - Initiates Two-Way Ranging (TWR) measurements.
             * twr_responder - Responds to Two-Way Ranging (TWR) measurements.
             * tdoa_dl_tag - Tag used in Downlink Time Difference of Arrival (TDoA DL).
             * tdoa_dl_anchor_controller - Controller anchor in TDoA DL.
             * tdoa_dl_anchor_controlee - Controlee anchor in TDoA DL.
             */

            RCLCPP_INFO(this->get_logger(), "Start SR1XX in role %s", role.c_str());
            ret = sr1xx_driver_start(&dev, role.c_str());

            if (ret >= 0) {
                RCLCPP_INFO(this->get_logger(), "Loaded firmware v%X.%X.%X UCI %X.%X.%X", dev.fw_version_major,
                            dev.fw_version_minor, dev.fw_version_patch, dev.uci_version_major, dev.uci_version_minor,
                            dev.uci_version_patch);

                std::stringstream ss;

                // Concatenate the role and the hex-encoded XOR of the UWB UUID as frame_id
                ss << role << "_" << std::hex << (*(uint64_t *)&dev.chip_id[0] ^ *(uint64_t *)&dev.chip_id[4]);
                this->frame_id = ss.str();
                RCLCPP_INFO(this->get_logger(), "Frame id %s", this->frame_id.c_str());
            }
        }

        if (ret < 0) {
            RCLCPP_INFO(this->get_logger(), "sr1xx_driver_init ERROR: %i", ret);
            RCLCPP_INFO(this->get_logger(), "Stopping the node");
            exit(0);
        }

        publisher_ = this->create_publisher<uwb_msgs::msg::UltraWideBandRanging>(role, 10);

        sr1xx_driver_register_callback(&dev, sr1xx_driver_recv, this);
    }

  private:
    /**
     * @brief Decodes Two-Way Ranging (TWR) measurements from a given session and populates the UltraWideBandRanging
     * ROS2 message with the decoded data.
     *
     * @param session Pointer to the session information containing TWR measurements from the sr1xx radio.
     * @param msg Reference to the UltraWideBandRanging ROS2 message to be populated with decoded TWR data.
     */
    void uci_decode_two_way(ShortMacTwoWaySessionInfoNtf_view *session, uwb_msgs::msg::UltraWideBandRanging &msg) {
        msg.measurements += session->two_way_ranging_measurements_count;

        // TODO ExtendedAddressTwoWayRangingMeasurement

        for (int i = 0; i < session->two_way_ranging_measurements_count; i++) {
            auto twr = uwb_msgs::msg::TwoWayRanging();
            ShortAddressTwoWayRangingMeasurement *measurement = &session->two_way_ranging_measurements[i];

            twr.nlos = measurement->nlos;
            twr.mac_address = measurement->mac_address;
            twr.status = static_cast<uint8_t>(measurement->status);
            twr.distance = measurement->distance;
            twr.aoa_azimuth = measurement->aoa_azimuth;
            twr.aoa_azimuth_fom = measurement->aoa_azimuth_fom;
            twr.aoa_elevation = measurement->aoa_elevation;
            twr.aoa_elevation_fom = measurement->aoa_elevation_fom;
            twr.aoa_destination_azimuth = measurement->aoa_destination_azimuth;
            twr.aoa_destination_azimuth_fom = measurement->aoa_destination_azimuth_fom;
            twr.aoa_destination_elevation = measurement->aoa_destination_elevation;
            twr.aoa_destination_elevation_fom = measurement->aoa_destination_elevation_fom;
            twr.slot_index = measurement->slot_index;
            twr.rssi = measurement->rssi;
            msg.two_way_data.push_back(twr);
        }

        // Decode NXP vendor-specific data from the session.

        uint8_t *vendor_data = session->vendor_data;

        if (dev.fw_version_major > 0x44) {
            uint16_t vendor_size = *(uint16_t *)(&vendor_data[0]); // Read data length field 2 octets
            vendor_data += 2;                                      // increment vendor data pointer

            if (session->vendor_data_count != vendor_size + 2) {
                RCLCPP_ERROR(this->get_logger(), "TWR vendor size mismatch expected %d got %d", vendor_size + 2,
                             session->vendor_data_count);
                return;
            }

            for (int i = 0; i < msg.measurements; i++) {
                auto rx_entry = uwb_msgs::msg::RxAntennaEntry();

                uint8_t rx_info_aoa = vendor_data[3];
                uint8_t rx_info_debug = vendor_data[5 + rx_info_aoa];
                uint8_t aoa_meas_offset = 6 + rx_info_aoa + rx_info_debug;
                uint8_t debug_meas_offset = aoa_meas_offset + rx_info_aoa * 6;

                for (int j = 0; j < rx_info_aoa; j++) {
                    auto rx_entry = uwb_msgs::msg::RxAntennaEntry();

                    rx_entry.field_presence_mask = uwb_msgs::msg::RxAntennaEntry::PRESENCE_ANGLE_OF_ARRIVAL |
                                                   uwb_msgs::msg::RxAntennaEntry::PRESENCE_PDOA |
                                                   uwb_msgs::msg::RxAntennaEntry::PRESENCE_CIR_INDEX_ESTIMATE |
                                                   uwb_msgs::msg::RxAntennaEntry::PRESENCE_FIRST_PATH_SNR |
                                                   uwb_msgs::msg::RxAntennaEntry::PRESENCE_MAIN_PATH_SNR |
                                                   uwb_msgs::msg::RxAntennaEntry::PRESENCE_FIRST_PATH_INDEX_CIR |
                                                   uwb_msgs::msg::RxAntennaEntry::PRESENCE_MAIN_PATH_INDEX_CIR;

                    rx_entry.rx_antenna_ids.push_back(1);
                    rx_entry.rx_antenna_ids.push_back(vendor_data[6 + rx_info_aoa + j]);

                    rx_entry.angle_of_arrival = (vendor_data[aoa_meas_offset + 1] << 8) | vendor_data[aoa_meas_offset];
                    rx_entry.pdoa = (vendor_data[aoa_meas_offset + 3] << 8) | vendor_data[aoa_meas_offset + 2];
                    rx_entry.cir_index_estimate =
                        (vendor_data[aoa_meas_offset + 5] << 8) | vendor_data[aoa_meas_offset + 4];

                    rx_entry.first_path_snr = vendor_data[debug_meas_offset];
                    rx_entry.main_path_snr = vendor_data[debug_meas_offset + 1];
                    rx_entry.first_path_index_cir =
                        (vendor_data[debug_meas_offset + 3] << 8) | vendor_data[debug_meas_offset + 2];
                    rx_entry.main_path_index_cir =
                        (vendor_data[debug_meas_offset + 5] << 8) | vendor_data[debug_meas_offset + 4];

                    msg.two_way_data[i].rx_entries.push_back(rx_entry);

                    aoa_meas_offset += 6;
                    debug_meas_offset += 6;
                }

                if (i == 1) {
                    // TODO calc offset i > 1
                    break;
                }
            }

        } else { // Firmware version <= 44
            for (int i = 0; i < msg.measurements; i++) {

                int pdoa_meas_offset = 10;

                for (int j = 0; j < vendor_data[9]; j++) {
                    auto rx_entry = uwb_msgs::msg::RxAntennaEntry();

                    rx_entry.field_presence_mask = uwb_msgs::msg::RxAntennaEntry::PRESENCE_PDOA |
                                                   uwb_msgs::msg::RxAntennaEntry::PRESENCE_CIR_INDEX_ESTIMATE;

                    rx_entry.rx_antenna_ids.push_back(1);
                    rx_entry.rx_antenna_ids.push_back(j + 2);

                    rx_entry.pdoa = (vendor_data[pdoa_meas_offset + 1] << 8) | vendor_data[pdoa_meas_offset];
                    rx_entry.cir_index_estimate =
                        (vendor_data[pdoa_meas_offset + 3] << 8) | vendor_data[pdoa_meas_offset + 2];

                    msg.two_way_data[i].rx_entries.push_back(rx_entry);

                    pdoa_meas_offset += 4;
                }

                if (i == 1) {
                    // TODO calc offset i > 1
                    break;
                }
            }
        }
    }

    /**
     * @brief Decodes Time Difference of Arrival (TDoA) measurements from a given session and populates the
     * UltraWideBandRanging ROS2 message with the decoded data.
     *
     * @param session Pointer to the session information containing TDoA measurements from the sr1xx radio.
     * @param msg Reference to the UltraWideBandRanging ROS2 message to be populated with decoded TDoA data.
     */
    void uci_decode_tdoa(ShortMacDlTDoASessionInfoNtf_view *session, uwb_msgs::msg::UltraWideBandRanging &msg) {
        msg.measurements += session->dl_tdoa_measurements_count;

        for (int i = 0; i < session->dl_tdoa_measurements_count; i++) {
            auto tdoa = uwb_msgs::msg::DownLinkTimeDifferenceOfArrival();
            ShortAddressDlTdoaRangingMeasurement *tdoa_ranging = &session->dl_tdoa_measurements[i];
            tdoa.mac_address = tdoa_ranging->mac_address;
            tdoa.status = static_cast<uint8_t>(tdoa_ranging->measurement.status);
            tdoa.message_type = tdoa_ranging->measurement.message_type;
            tdoa.tx_timestamp_type = tdoa_ranging->measurement.tx_timestamp_type;
            tdoa.block_index = tdoa_ranging->measurement.block_index;
            tdoa.round_index = tdoa_ranging->measurement.round_index;
            tdoa.nlos = tdoa_ranging->measurement.nlos;
            tdoa.aoa_azimuth = tdoa_ranging->measurement.aoa_azimuth;
            tdoa.aoa_azimuth_fom = tdoa_ranging->measurement.aoa_azimuth_fom;
            tdoa.aoa_elevation = tdoa_ranging->measurement.aoa_elevation;
            tdoa.aoa_elevation_fom = tdoa_ranging->measurement.aoa_elevation_fom;
            tdoa.rssi = tdoa_ranging->measurement.rssi;

            if (tdoa_ranging->measurement.tx_timestamp_length == 0) {
                tdoa.tx_timestamp = tdoa_ranging->measurement.tx_timestamp_40;
            } else {
                tdoa.tx_timestamp = tdoa_ranging->measurement.tx_timestamp_64;
            }

            if (tdoa_ranging->measurement.rx_timestamp_length == 0) {
                tdoa.rx_timestamp = tdoa_ranging->measurement.rx_timestamp_40;
            } else {
                tdoa.rx_timestamp = tdoa_ranging->measurement.rx_timestamp_64;
            }

            tdoa.anchor_cfo = tdoa_ranging->measurement.anchor_cfo;
            tdoa.cfo = tdoa_ranging->measurement.cfo;
            tdoa.initiator_reply_time = tdoa_ranging->measurement.initiator_reply_time;
            tdoa.responder_reply_time = tdoa_ranging->measurement.responder_reply_time;
            tdoa.initiator_responder_tof = tdoa_ranging->measurement.initiator_responder_tof;

            // TODO dt_anchor_location Wgs84

            if (tdoa_ranging->measurement.relative_location_count == 1) {
                auto location = geometry_msgs::msg::Vector3();

                // Convert Q28.0 and Q24.0 millimeters to meters for the ROS2 Vector3 msg

                if (tdoa_ranging->measurement.relative_location->x & 0x8000000) { // Signed, thus negative
                    location.x = (int32_t)(tdoa_ranging->measurement.relative_location->x | 0xF0000000) / 1000.0f;
                } else {
                    location.x = tdoa_ranging->measurement.relative_location->x / 1000.0f;
                }

                if (tdoa_ranging->measurement.relative_location->y & 0x8000000) { // Signed, thus negative
                    location.y = (int32_t)(tdoa_ranging->measurement.relative_location->y | 0xF0000000) / 1000.0f;
                } else {
                    location.y = tdoa_ranging->measurement.relative_location->y / 1000.0f;
                }

                if (tdoa_ranging->measurement.relative_location->z & 0x800000) { // Signed, thus negative
                    location.z = (int32_t)(tdoa_ranging->measurement.relative_location->z | 0xFF000000) / 1000.0f;
                } else {
                    location.z = tdoa_ranging->measurement.relative_location->z / 1000.0f;
                }

                tdoa.relative_location.push_back(location);
            }

            // TODO active ranging rounds

            msg.dl_tdoa_data.push_back(tdoa);
        }
    }

    /**
     * @brief Decodes session information from a given payload and populates the UltraWideBandRanging ROS2 message with
     * the decoded data.
     *
     * @param payload Pointer to the payload containing session information from the sr1xx radio.
     * @param size Size of the payload.
     * @param msg Reference to the UltraWideBandRanging ROS2 message to be populated with decoded session data.
     */
    void uci_decode_session(uint8_t *payload, size_t size, uwb_msgs::msg::UltraWideBandRanging &msg) {
        size_t index = 0;

        ControlPacket_view control_packet = {};
        if (!uci_parse_ControlPacket_view(payload, size, &index, &control_packet)) {
            return;
        }

        if (control_packet.gid == GROUPID_SESSION_CONTROL) {
            SessionControlPacket_view session_control_packet = {};
            if (!uci_parse_SessionControlPacket_view(&control_packet, payload, size, &index, &session_control_packet)) {
                return;
            }

            if (session_control_packet.mt == MESSAGETYPE_NOTIFICATION) {
                SessionInfoNtf_view session_info = {};
                if (!uci_parse_SessionInfoNtf_view(&session_control_packet, payload, size, &index, &session_info)) {
                    return;
                }

                msg.ranging_type = static_cast<uint8_t>(session_info.ranging_measurement_type);
                msg.session_id = session_info.session_token;
                msg.session_sequence = session_info.sequence_number;
                msg.ranging_interval = session_info.current_ranging_interval;
                msg.mac_address_indicator = static_cast<uint8_t>(session_info.mac_address_indicator);

                msg.authenticated = false; // TODO authentication

                switch (session_info.ranging_measurement_type) {
                case RANGINGMEASUREMENTTYPE_TWO_WAY: {
                    ShortMacTwoWaySessionInfoNtf_view twoway_session = {};
                    if (!uci_parse_ShortMacTwoWaySessionInfoNtf_view(&session_info, payload, size, &index,
                                                                     &twoway_session)) {
                        return;
                    }
                    uci_decode_two_way(&twoway_session, msg);
                    uci_free_ShortMacTwoWaySessionInfoNtf_view(&twoway_session);
                    break;
                }

                case RANGINGMEASUREMENTTYPE_DL_TDOA: {
                    ShortMacDlTDoASessionInfoNtf_view tdoa_session = {};
                    if (!uci_parse_ShortMacDlTDoASessionInfoNtf_view(&session_info, payload, size, &index,
                                                                     &tdoa_session)) {
                        return;
                    }
                    uci_decode_tdoa(&tdoa_session, msg);
                    uci_free_ShortMacDlTDoASessionInfoNtf_view(&tdoa_session);
                    break;
                }

                default:
                    RCLCPP_ERROR(this->get_logger(), "%s Not implemented",
                                 RangingMeasurementTypeText(session_info.ranging_measurement_type));
                }
            }
        }
    }

    /**
     * @brief Parses the UCI payload and publishes the UltraWideBandRanging ROS2 message if the payload contains a
     * session control notification.
     *
     * @param payload Pointer to the payload containing UCI data from the sr1xx radio.
     * @param size Size of the payload.
     */
    void sr1xx_parse_uci(uint8_t *payload, size_t size) {

        UciPacketDecode decode = uci_decode(payload, size);

        if (decode.ret == -EINVAL) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing UCI");
            return;
        }

        if (decode.mt == MESSAGETYPE_NOTIFICATION && decode.gid == GROUPID_SESSION_CONTROL) {
            auto message = uwb_msgs::msg::UltraWideBandRanging();

            message.header.stamp = this->get_clock()->now();
            message.header.frame_id = this->frame_id;

            uci_decode_session(payload, size, message);

            RCLCPP_INFO(this->get_logger(), "Publishing UWB");
            publisher_->publish(message);
        }
    }

    /**
     * @brief Callback function for receiving UCI payloads from the SR1XX driver.
     *
     * This function is called for each received measurement and processes the payload by invoking the `sr1xx_parse_uci`
     * method of the `RangingPublisher` class.
     *
     * @param payload Pointer to the payload containing UCI data from the sr1xx radio.
     * @param size Size of the payload.
     * @param user_data Pointer to user data, expected to be a `RangingPublisher` instance.
     */
    static void sr1xx_driver_recv(uint8_t *payload, size_t size, void *user_data) {
        static_cast<RangingPublisher *>(user_data)->sr1xx_parse_uci(payload, size);
    }

    sr1xx_dev dev = {};
    rclcpp::Publisher<uwb_msgs::msg::UltraWideBandRanging>::SharedPtr publisher_;
    size_t count_;
    std::string frame_id;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RangingPublisher>());
    rclcpp::shutdown();
    return 0;
}
