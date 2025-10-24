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
    {MessageType::POSE, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){ handle<point_one::fusion_engine::messages::PoseMessage, navigation_msgs::Pose, fusion_engine_msgs::msg::Pose>(n, "pose_filtered", reinterpret_cast<const point_one::fusion_engine::messages::PoseMessage*>(p), id, t); }},
    {MessageType::POSE_AUX, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){ handle<point_one::fusion_engine::messages::PoseAuxMessage, navigation_msgs::PoseAux, fusion_engine_msgs::msg::PoseAux>(n, "pose_aux", reinterpret_cast<const point_one::fusion_engine::messages::PoseAuxMessage*>(p), id, t); }},
    {MessageType::CALIBRATION_STATUS, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){ handle<point_one::fusion_engine::messages::CalibrationStatusMessage, navigation_msgs::CalibrationStatus, fusion_engine_msgs::msg::CalibrationStatus>(n, "calibration_status", reinterpret_cast<const point_one::fusion_engine::messages::CalibrationStatusMessage*>(p), id, t); }},
    {MessageType::GNSS_INFO, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){ handle<point_one::fusion_engine::messages::GNSSInfoMessage, navigation_msgs::GnssInfo, fusion_engine_msgs::msg::GnssInfo>(n, "gnss_info", reinterpret_cast<const point_one::fusion_engine::messages::GNSSInfoMessage*>(p), id, t); }},
    {MessageType::GNSS_SATELLITE, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){ handle<point_one::fusion_engine::messages::GNSSSatelliteMessage, navigation_msgs::GnssSatellite, fusion_engine_msgs::msg::GnssSatellite>(n, "gnss_satellite", reinterpret_cast<const point_one::fusion_engine::messages::GNSSSatelliteMessage*>(p), id, t); }},
    {MessageType::RELATIVE_ENU_POSITION, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){ handle<point_one::fusion_engine::messages::RelativeENUPositionMessage, navigation_msgs::RelativeEnuPosition, fusion_engine_msgs::msg::RelativeEnuPosition>(n, "relative_enu_position", reinterpret_cast<const point_one::fusion_engine::messages::RelativeENUPositionMessage*>(p), id, t); }},

    // Calibrated
    {MessageType::IMU_OUTPUT, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){ handle<point_one::fusion_engine::messages::IMUOutput, calibrated_msgs::Imu, sensor_msgs::msg::Imu>(n, "imu_calibrated", reinterpret_cast<const point_one::fusion_engine::messages::IMUOutput*>(p), id, t); }},
    {MessageType::GNSS_ATTITUDE_OUTPUT, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){ handle<point_one::fusion_engine::messages::GNSSAttitudeOutput, calibrated_msgs::GnssAttitudeOutput, fusion_engine_msgs::msg::GnssAttitudeOutput>(n, "gnss_attitude", reinterpret_cast<const point_one::fusion_engine::messages::GNSSAttitudeOutput*>(p), id, t); }},
    {MessageType::WHEEL_SPEED_OUTPUT, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){ handle<point_one::fusion_engine::messages::WheelSpeedOutput, calibrated_msgs::WheelSpeedOutput, fusion_engine_msgs::msg::WheelSpeedOutput>(n, "wheel_speed", reinterpret_cast<const point_one::fusion_engine::messages::WheelSpeedOutput*>(p), id, t); }},
    {MessageType::VEHICLE_SPEED_OUTPUT, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){ handle<point_one::fusion_engine::messages::VehicleSpeedOutput, calibrated_msgs::VehicleSpeedOutput, fusion_engine_msgs::msg::VehicleSpeedOutput>(n, "vehicle_speed", reinterpret_cast<const point_one::fusion_engine::messages::VehicleSpeedOutput*>(p), id, t); }},

    // Raw
    {MessageType::RAW_IMU_OUTPUT, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){ handle<point_one::fusion_engine::messages::RawIMUOutput, raw_msgs::RawImu, sensor_msgs::msg::Imu>(n, "imu_raw", reinterpret_cast<const point_one::fusion_engine::messages::RawIMUOutput*>(p), id, t); }},
    {MessageType::RAW_WHEEL_TICK_OUTPUT, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){ handle<point_one::fusion_engine::messages::RawWheelTickOutput, raw_msgs::RawWheelTickOutput, fusion_engine_msgs::msg::RawWheelTickOutput>(n, "wheel_tick_raw", reinterpret_cast<const point_one::fusion_engine::messages::RawWheelTickOutput*>(p), id, t); }},

    // ROS-standard
    {MessageType::ROS_POSE, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){ handle<point_one::fusion_engine::messages::ros::PoseMessage, ros_msgs::Pose, geometry_msgs::msg::PoseStamped>(n, "pose_ros", reinterpret_cast<const point_one::fusion_engine::messages::ros::PoseMessage*>(p), id, t); }},
    {MessageType::ROS_GPS_FIX, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){ handle<point_one::fusion_engine::messages::ros::GPSFixMessage, ros_msgs::GPSFix, gps_msgs::msg::GPSFix>(n, "gpsfix_ros", reinterpret_cast<const point_one::fusion_engine::messages::ros::GPSFixMessage*>(p), id, t); }},
    {MessageType::ROS_IMU, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){ handle<point_one::fusion_engine::messages::ros::IMUMessage, ros_msgs::Imu, sensor_msgs::msg::Imu>(n, "imu_ros", reinterpret_cast<const point_one::fusion_engine::messages::ros::IMUMessage*>(p), id, t); }},
  };
  return kHandles;
}

using SBFHandler = std::function<void(rclcpp::Node*, const uint8_t*, const std::string&, const rclcpp::Time&)>;
inline const auto& kSBF()
{
  static const std::unordered_map<SBFBlockID, SBFHandler> kSBFHandles = {
    {SBFBlockID::PVTGeodetic, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){ handle<PVTGeodetic, sbf_msgs::PVTGeodetic, fusion_engine_msgs::msg::PVTGeodetic>(n, "pvt_geodetic", reinterpret_cast<const PVTGeodetic*>(p), id, t); }},
  };
  return kSBFHandles;
}

/******************************************************************************/
inline const Handler& findHandler(const MessageHeader& header)
{
  const auto& table = kHandlers();
  static const Handler kNoOp = [](auto*, auto*, const std::string&, const rclcpp::Time&) {
  };
  static const Handler kSBFOp = [header](auto* n, auto* p, const std::string& f, const rclcpp::Time& t) {
    auto & contents = *reinterpret_cast <
            const point_one::fusion_engine::messages::InputDataWrapperMessage * > (p);
    if(contents.data_type == static_cast<uint16_t>(InputDataType::SBF_DATA)) {
      const uint8_t* inner_payload = reinterpret_cast<const uint8_t*>(&contents) + sizeof(InputDataWrapperMessage);
      size_t inner_size = header.payload_size_bytes - sizeof(InputDataWrapperMessage);

      if (isSBF(inner_payload, inner_size)) {
        uint16_t block_id = inner_payload[4] | (inner_payload[5] << 8);
        uint16_t block_num = block_id & 0x1FFF;
        RCLCPP_INFO(n->get_logger(), "Detected SBF block 0x%04X (%u)", block_num, block_num);
        kSBF().find(static_cast<SBFBlockID>(block_num))->second(n, inner_payload, f, t);
      }
    }
  };
  auto it = table.find(header.message_type);
  if (it != table.end())
    return it->second;
  if (header.message_type == MessageType::INPUT_DATA_WRAPPER)
    return kSBFOp;

  return kNoOp;
}

#endif  // FUSION_ENGINE_DRIVER__CORE__FUSION_DISPATCH_HPP_
