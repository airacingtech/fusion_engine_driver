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

#ifndef FUSION_ENGINE_DRIVER__CORE__FUSION_DISPATCH_HPP_
#define FUSION_ENGINE_DRIVER__CORE__FUSION_DISPATCH_HPP_

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "gps_msgs/msg/gps_status.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/rtcm.hpp"
#include "nmea_msgs/msg/sentence.hpp"

#include <point_one/fusion_engine/messages/core.h>
#include <point_one/fusion_engine/messages/ros.h>
#include "navigation_msgs.hpp"
#include "calibrated_msgs.hpp"
#include "raw_msgs.hpp"
#include "ros_msgs.hpp"
#include "sbf_msgs.hpp"

#include "helper.hpp"

using namespace point_one::fusion_engine::messages;
using namespace point_one::fusion_engine::messages::ros;

/******************************************************************************/
template<typename MsgT>
inline void publish(rclcpp::Node* node,
                         const std::string& topic,
                         const void* msg_ptr)
{
  static auto pub =
    node->create_publisher<MsgT>(topic, rclcpp::SensorDataQoS());
  pub->publish(*reinterpret_cast<const MsgT*>(msg_ptr));
}
/******************************************************************************/
inline const auto& kFactory()
{
  using Factory = std::function<void(rclcpp::Node*, const void*)>;
  static const std::unordered_map<MessageType, Factory> kTable = {
    // Navigation
    {MessageType::POSE,
     [](auto* n, auto* m) { publish<fusion_engine_msgs::msg::Pose>(n, "pose_filtered", m); }},
    {MessageType::POSE_AUX,
     [](auto* n, auto* m) { publish<fusion_engine_msgs::msg::PoseAux>(n, "pose_aux", m); }},
    {MessageType::CALIBRATION_STATUS,
     [](auto* n, auto* m) { publish<fusion_engine_msgs::msg::CalibrationStatus>(n, "calibration_status", m); }},
    {MessageType::GNSS_INFO,
     [](auto* n, auto* m) { publish<fusion_engine_msgs::msg::GnssInfo>(n, "gnss_info", m); }},
    {MessageType::GNSS_SATELLITE,
     [](auto* n, auto* m) { publish<fusion_engine_msgs::msg::GnssSatellite>(n, "gnss_satellite", m); }},
    {MessageType::RELATIVE_ENU_POSITION,
     [](auto* n, auto* m) { publish<fusion_engine_msgs::msg::RelativeEnuPosition>(n, "relative_enu_position", m); }},

    // Calibrated
    {MessageType::IMU_OUTPUT,
     [](auto* n, auto* m) { publish<sensor_msgs::msg::Imu>(n, "imu_calibrated", m); }},
    {MessageType::GNSS_ATTITUDE_OUTPUT,
     [](auto* n, auto* m) { publish<fusion_engine_msgs::msg::GnssAttitudeOutput>(n, "gnss_attitude", m); }},
    {MessageType::WHEEL_SPEED_OUTPUT,
     [](auto* n, auto* m) { publish<fusion_engine_msgs::msg::WheelSpeedOutput>(n, "wheel_speed", m); }},
    {MessageType::VEHICLE_SPEED_OUTPUT,
     [](auto* n, auto* m) { publish<fusion_engine_msgs::msg::VehicleSpeedOutput>(n, "vehicle_speed", m); }},

    // Raw
    {MessageType::RAW_IMU_OUTPUT,
     [](auto* n, auto* m) { publish<sensor_msgs::msg::Imu>(n, "imu_raw", m); }},
    {MessageType::RAW_GNSS_ATTITUDE_OUTPUT,
     [](auto* n, auto* m) { publish<fusion_engine_msgs::msg::RawGnssAttitudeOutput>(n, "gnss_attitude_raw", m); }},
    {MessageType::RAW_WHEEL_TICK_OUTPUT,
     [](auto* n, auto* m) { publish<fusion_engine_msgs::msg::RawWheelTickOutput>(n, "wheel_tick_raw", m); }},
    {MessageType::RAW_VEHICLE_TICK_OUTPUT,
     [](auto* n, auto* m) { publish<fusion_engine_msgs::msg::RawVehicleTickOutput>(n, "vehicle_tick_raw", m); }},
    {MessageType::RAW_WHEEL_SPEED_OUTPUT,
     [](auto* n, auto* m) { publish<fusion_engine_msgs::msg::RawWheelSpeedOutput>(n, "wheel_speed_raw", m); }},
    {MessageType::RAW_VEHICLE_SPEED_OUTPUT,
     [](auto* n, auto* m) { publish<fusion_engine_msgs::msg::RawVehicleSpeedOutput>(n, "vehicle_speed_raw", m); }},

    // ROS-standard outputs
    {MessageType::ROS_POSE,
     [](auto* n, auto* m) { publish<geometry_msgs::msg::PoseStamped>(n, "pose_ros", m); }},
    {MessageType::ROS_GPS_FIX,
     [](auto* n, auto* m) { publish<gps_msgs::msg::GPSFix>(n, "gpsfix_ros", m); }},
    {MessageType::ROS_IMU,
     [](auto* n, auto* m) { publish<sensor_msgs::msg::Imu>(n, "imu_ros", m); }},

     // SBF
  //    {MessageType::InputDataWrapper,
  //     [](auto* n, auto* m) { publish<fusion_engine_msgs::msg::InputDataWrapper>(n, "sbf/input_data_wrapper", m); }},
  };
  return kTable;
}

/******************************************************************************/

#endif // FUSION_ENGINE_DRIVER__CORE__FUSION_DISPATCH_HPP_