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

#include "std_msgs/msg/string.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"

#include "gps_msgs/msg/gps_fix.hpp"
#include "gps_msgs/msg/gps_status.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/rtcm.hpp"
#include "nmea_msgs/msg/sentence.hpp"

#include "fusion_engine_interface.hpp"
#include "fusion_engine_msgs/msg/pose.hpp"
#include "fusion_engine_msgs/msg/pose_aux.hpp"
#include "fusion_engine_msgs/msg/calibration_status.hpp"
#include "fusion_engine_msgs/msg/relative_enu_position.hpp"
#include "fusion_engine_msgs/msg/gnss_info.hpp"
#include "fusion_engine_msgs/msg/gnss_satellite.hpp"
#include "fusion_engine_msgs/msg/gnss_satellite_info.hpp"
#include "fusion_engine_msgs/msg/gnss_attitude_output.hpp"
#include "fusion_engine_msgs/msg/wheel_speed_output.hpp"
#include "fusion_engine_msgs/msg/vehicle_speed_output.hpp"
#include "fusion_engine_msgs/msg/raw_gnss_attitude_output.hpp"
#include "fusion_engine_msgs/msg/raw_vehicle_speed_output.hpp"
#include "fusion_engine_msgs/msg/raw_vehicle_tick_output.hpp"
#include "fusion_engine_msgs/msg/raw_wheel_speed_output.hpp"
#include "fusion_engine_msgs/msg/raw_wheel_tick_output.hpp"
#include "fusion_engine_msgs/msg/rpy.hpp"
#include "isaac_ros_nitros/nitros_publisher.hpp"
#include "isaac_ros_nitros_imu_type/nitros_imu.hpp"
#include "isaac_ros_nitros/nitros_node.hpp"

using nvidia::isaac_ros::nitros::NitrosImu;
using nvidia::isaac_ros::nitros::NitrosPublisher;
using nvidia::isaac_ros::nitros::NitrosTypeManager;

/**
 * @brief Point One Nav Atlas Node publishes realtime GPSFix/IMU/Pose messages.
 */
class FusionEngineNode : public rclcpp::Node
{
public:
  FusionEngineNode();
  ~FusionEngineNode();

  /**
   * @brief Receive Fusion Message, build and post them in ros system.
   *
   * @param header Message header with type of message.
   * @param payload Message content
   */
  void receivedFusionEngineMessage(
    const MessageHeader & header,
    const void * payload);

private:
  /**
   * @brief Interface made to manage the reception of the fusion engines's
   * messages.
   *
   */
  FusionEngineInterface fe_interface_;

  /**
   * @brief
   * Publishers for Navigation Messages
   * Pose (10000)
   * GNSSInfo (10001)
   * GNSSSatellite (10002)
   * PoseAux (10003)
   * CalibrationStatus (10004)
   * RelativeENUPosition (10005)
   *
   * Publishers for Calibrated Messages
   * IMUOutput (11000)
   * GNSSAttitudeOutput (11001)
   * WheelSpeedOutput (11135)
   * VehicleSpeedOutput (11136)
   *
   * Publishers for Raw Messages
   * RawIMUOutput (11002)
   * RawGNSSAttitudeOutput (11006)
   * RawWheelTickOutput (11123)
   * RawVehicleTickOutput (11124)
   * RawWheelSpeedOutput (11125)
   * RawVehicleSpeedOutput (11126)
   * InputDataWrapper (13120)
   *
   * Publishers for ROS Messages
   * ROSPose (12000)
   * ROSGPSFix (12010)
   * ROSIMUMessage (12011)
   */

  /**
   * @brief Smart pointer to a ROS 2 publisher for NavSatFix data.
   *
   * This smart pointer provides access to a ROS 2 publisher, which is used to
   * publish NavSatFix data on topic /fix. The publisher is represented by
   * the `rclcpp::Publisher<sensor_msgs::msg::NavSatFix>` type.
   */
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_fix_publisher_;

  /**
   * @brief Smart pointer to a ROS 2 publisher for NMEA sentence data.
   * This publisher is active only in serial mode.
   *
   * This smart pointer provides access to a ROS 2 publisher, which is used to
   * publish NMEA sentence data on /ntrip_client/nmea. The publisher is
   * represented by the `rclcpp::Publisher<nmea_msgs::msg::Sentence>` type.
   */
  rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr nmea_publisher_;

  /**
   * @brief Smart pointer to a ROS 2 subscription for RTCM data.
   * This subscriber is active only in serial mode.
   *
   * This smart pointer provides access to a ROS 2 subscription, which is used
   * to receive RTCM data on topic /ntrip_client/rtcm. The subscription is
   * represented by the `rclcpp::Subscription<mavros_msgs::msg::RTCM>` type.
   */
  rclcpp::Subscription<mavros_msgs::msg::RTCM>::SharedPtr subscription_;

  /**
   * @brief Number of satellite used for create nmea message.
   */
  uint16_t satellite_nb_;
  
  /**
   * @brief A smart pointer to a timer object, which allows scheduling periodic
   * callbacks.
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Id of the frame send in ros.
   */
  std::string frame_id_;

  /**
   * @brief Thread used for get data from the receiver.
   */
  std::thread listener_thread_;

  /**
   * @brief Timestamp of the precedent nmea message sent to the ntrip server.
   */
  double previous_gps_time_sec_ = 0;

  /**
   * @brief The delay accepted between NMEA message processing.
   *
   * This constant value is set to 30 seconds, and represents the maximum amount
   * of time that the system will wait before processing the next NMEA message.
   */
  static constexpr double TIME_BETWEEN_NMEA_UPDATES_SEC_{30.0};

  /**
   * @brief Call the listener service to manage data recption.
   *
   */
  void dataListenerService();

  /**
   * Initiate gps unit to read data.
   */
  void rosServiceLoop();
};

#endif  // FUSION_ENGINE_DRIVER__CORE__FUSION_ENGINE_NODE_HPP_