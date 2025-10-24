#ifndef FUSION_ENGINE_DRIVER__CORE__FUSION_DISPATCH_HPP_
#define FUSION_ENGINE_DRIVER__CORE__FUSION_DISPATCH_HPP_

#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <functional>

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
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

using Handler = std::function<void(rclcpp::Node*, const void*, const std::string&, const rclcpp::Time&)>;

/******************************************************************************/
/** Unified make + handle helper (construct + publish in one step) **/
template <typename SrcT, typename MsgT, typename DstT>
inline void handle(rclcpp::Node* node,
                   const std::string& topic,
                   const SrcT* payload,
                   const std::string& frame_id,
                   const rclcpp::Time& stamp)
{
  MsgT msg{*payload};
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;

  static auto pub = node->create_publisher<DstT>(topic, rclcpp::SensorDataQoS());
  pub->publish(static_cast<const DstT&>(msg));
}

/******************************************************************************/
/** Handler table using handle() **/
inline const auto& kHandlers()
{
  static const std::unordered_map<MessageType, Handler> kHandles = {
    // Navigation
    {MessageType::POSE, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){ handle<point_one::fusion_engine::messages::PoseMessage, navigation_msgs::Pose, fusion_engine_msgs::msg::Pose >(n, "pose_filtered", reinterpret_cast<const point_one::fusion_engine::messages::PoseMessage*>(p), id, t);
    }},
    // {MessageType::POSE_AUX, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
    //     handle<navigation_msgs::PoseAux, PoseAuxMessage>(n, "pose_aux",
    //                                                      reinterpret_cast<const PoseAuxMessage*>(p), id, t);
    // }},
    // {MessageType::CALIBRATION_STATUS, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
    //     handle<navigation_msgs::CalibrationStatus, CalibrationStatusMessage>(
    //       n, "calibration_status",
    //       reinterpret_cast<const CalibrationStatusMessage*>(p), id, t);
    // }},
    // {MessageType::GNSS_INFO, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
    //     handle<navigation_msgs::GnssInfo, GNSSInfoMessage>(n, "gnss_info",
    //                                                        reinterpret_cast<const GNSSInfoMessage*>(p), id, t);
    // }},
    // {MessageType::GNSS_SATELLITE, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
    //     handle<navigation_msgs::GnssSatellite, GNSSSatelliteMessage>(
    //       n, "gnss_satellite",
    //       reinterpret_cast<const GNSSSatelliteMessage*>(p), id, t);
    // }},
    // {MessageType::RELATIVE_ENU_POSITION, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
    //     handle<navigation_msgs::RelativeEnuPosition, RelativeENUPositionMessage>(
    //       n, "relative_enu_position",
    //       reinterpret_cast<const RelativeENUPositionMessage*>(p), id, t);
    // }},

    // // Calibrated
    // {MessageType::IMU_OUTPUT, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
    //     handle<calibrated_msgs::Imu, IMUOutput>(n, "imu_calibrated",
    //                                             reinterpret_cast<const IMUOutput*>(p), id, t);
    // }},
    // {MessageType::GNSS_ATTITUDE_OUTPUT, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
    //     handle<calibrated_msgs::GnssAttitudeOutput, GNSSAttitudeOutput>(
    //       n, "gnss_attitude",
    //       reinterpret_cast<const GNSSAttitudeOutput*>(p), id, t);
    // }},
    // {MessageType::WHEEL_SPEED_OUTPUT, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
    //     handle<calibrated_msgs::WheelSpeedOutput, WheelSpeedOutput>(
    //       n, "wheel_speed",
    //       reinterpret_cast<const WheelSpeedOutput*>(p), id, t);
    // }},
    // {MessageType::VEHICLE_SPEED_OUTPUT, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
    //     handle<calibrated_msgs::VehicleSpeedOutput, VehicleSpeedOutput>(
    //       n, "vehicle_speed",
    //       reinterpret_cast<const VehicleSpeedOutput*>(p), id, t);
    // }},

    // // Raw
    // {MessageType::RAW_IMU_OUTPUT, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
    //     handle<raw_msgs::RawImu, RawIMUOutput>(n, "imu_raw",
    //                                            reinterpret_cast<const RawIMUOutput*>(p), id, t);
    // }},
    // {MessageType::RAW_WHEEL_TICK_OUTPUT, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
    //     handle<raw_msgs::RawWheelTickOutput, RawWheelTickOutput>(
    //       n, "wheel_tick_raw",
    //       reinterpret_cast<const RawWheelTickOutput*>(p), id, t);
    // }},

    // // ROS-standard outputs
    // {MessageType::ROS_POSE, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
    //     handle<ros_msgs::Pose, PoseMessage>(n, "pose_ros",
    //                                         reinterpret_cast<const PoseMessage*>(p), id, t);
    // }},
    // {MessageType::ROS_GPS_FIX, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
    //     handle<ros_msgs::GPSFix, GPSFixMessage>(n, "gpsfix_ros",
    //                                             reinterpret_cast<const GPSFixMessage*>(p), id, t);
    // }},
    // {MessageType::ROS_IMU, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
    //     handle<ros_msgs::Imu, IMUMessage>(n, "imu_ros",
    //                                       reinterpret_cast<const IMUMessage*>(p), id, t);
    // }},
  };
  return kHandles;
}

/******************************************************************************/
/** Fast lookup **/
inline const Handler& findHandler(MessageType type)
{
  const auto& table = kHandlers();
  std::cout << "Inside" << "\n";
  auto it = table.find(type);
  if (it == table.end())
    RCLCPP_WARN(rclcpp::get_logger("FusionEngineNode"), "No handler for message type ", to_string(type));
  return it->second;
}

#endif  // FUSION_ENGINE_DRIVER__CORE__FUSION_DISPATCH_HPP_
