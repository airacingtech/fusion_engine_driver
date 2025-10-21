// Copyright 2025 AI Racing Tech
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef FUSION_ENGINE_DRIVER__CORE__FUSION_ENGINE_NODE_HPP_
#define FUSION_ENGINE_DRIVER__CORE__FUSION_ENGINE_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <unordered_map>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "fusion_engine_interface.hpp"
#include "errors.hpp"

// #include <septentrio_gnss_driver/msg/aim_plus_status.hpp>
// #include <septentrio_gnss_driver/msg/att_cov_euler.hpp>
// #include <septentrio_gnss_driver/msg/att_euler.hpp>
// #include <septentrio_gnss_driver/msg/base_vector_cart.hpp>
// #include <septentrio_gnss_driver/msg/base_vector_geod.hpp>
// #include <septentrio_gnss_driver/msg/block_header.hpp>
// #include <septentrio_gnss_driver/msg/gal_auth_status.hpp>
// #include <septentrio_gnss_driver/msg/meas_epoch.hpp>
// #include <septentrio_gnss_driver/msg/meas_epoch_channel_type1.hpp>
// #include <septentrio_gnss_driver/msg/meas_epoch_channel_type2.hpp>
// #include <septentrio_gnss_driver/msg/pos_cov_cartesian.hpp>
// #include <septentrio_gnss_driver/msg/pos_cov_geodetic.hpp>
// #include <septentrio_gnss_driver/msg/pvt_cartesian.hpp>
// #include <septentrio_gnss_driver/msg/pvt_geodetic.hpp>
// #include <septentrio_gnss_driver/msg/receiver_time.hpp>
// #include <septentrio_gnss_driver/msg/rf_band.hpp>
// #include <septentrio_gnss_driver/msg/rf_status.hpp>
// #include <septentrio_gnss_driver/msg/vector_info_cart.hpp>
// #include <septentrio_gnss_driver/msg/vector_info_geod.hpp>
// #include <septentrio_gnss_driver/msg/vel_cov_cartesian.hpp>
// #include <septentrio_gnss_driver/msg/vel_cov_geodetic.hpp>
// #include <septentrio_gnss_driver/msg/ext_sensor_meas.hpp>
// #include <septentrio_gnss_driver/msg/imu_setup.hpp>
// #include <septentrio_gnss_driver/msg/ins_nav_cart.hpp>
// #include <septentrio_gnss_driver/msg/ins_nav_geod.hpp>
// #include <septentrio_gnss_driver/msg/vel_sensor_setup.hpp>
// // Rosaic includes
// #include <septentrio_gnss_driver/communication/settings.hpp>
// #include <septentrio_gnss_driver/parsers/sbf_utilities.hpp>
// #include <septentrio_gnss_driver/parsers/string_utilities.


class FusionEngineNode : public rclcpp::Node
{
public:
  FusionEngineNode();
  ~FusionEngineNode();

  void handleFusionMessage(
    const MessageHeader & header,
    const void * payload);

private:

    // --- Connection ---
  std::string connection_type_;  // "tcp", "tty", or "udp"
  std::string ip_;           // Device IP
  std::string device_;         // Serial device path
  int port_;                 // TCP or UDP port
  bool debug_;
  FusionEngineInterface fe_interface_;

  
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_fix_publisher_;
  rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr nmea_publisher_;
  rclcpp::Subscription<mavros_msgs::msg::RTCM>::SharedPtr subscription_;

  uint16_t satellite_nb_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::string frame_id_;


  std::thread listener_thread_;

  double previous_gps_time_sec_ = 0;


  static constexpr double TIME_BETWEEN_NMEA_UPDATES_SEC_{30.0};

  void dataListenerService();


  void rosServiceLoop();
};

#endif  // FUSION_ENGINE_DRIVER__CORE__FUSION_ENGINE_NODE_HPP_