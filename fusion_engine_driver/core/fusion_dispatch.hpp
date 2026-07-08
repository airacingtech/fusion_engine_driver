#ifndef FUSION_ENGINE_DRIVER__CORE__FUSION_DISPATCH_HPP_
#define FUSION_ENGINE_DRIVER__CORE__FUSION_DISPATCH_HPP_

#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <functional>
#include <memory>

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
using SBFHandler = std::function<void(rclcpp::Node*, const uint8_t*, const std::string&, const rclcpp::Time&)>;

/******************************************************************************/
template <typename SrcT, typename MsgT, typename DstT>
inline void handle(rclcpp::Node* node,
                   const std::string& topic,
                   const SrcT* payload,
                   const std::string& frame_id,
                   const rclcpp::Time& stamp)
{
  static auto pub = node->create_publisher<DstT>(topic, rclcpp::SensorDataQoS());

  MsgT msg{*payload};
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;

  pub->publish(static_cast<const DstT&>(msg));
}

/******************************************************************************/
inline void handleGnssSignals(rclcpp::Node* node,
                              const MessageHeader& header,
                              const void* payload,
                              const std::string& frame_id,
                              const rclcpp::Time& stamp)
{
  static auto pub = node->create_publisher<fusion_engine_msgs::msg::GnssSignals>(
    "gnss_signal", rclcpp::SensorDataQoS());

  const auto* base =
    reinterpret_cast<const point_one::fusion_engine::messages::GNSSSignalsMessage*>(payload);
  const size_t base_size = sizeof(point_one::fusion_engine::messages::GNSSSignalsMessage);
  const size_t sat_size = sizeof(point_one::fusion_engine::messages::GNSSSatelliteInfo);
  const size_t sig_size = sizeof(point_one::fusion_engine::messages::GNSSSignalInfo);
  const size_t expected_size =
    base_size +
    (static_cast<size_t>(base->num_satellites) * sat_size) +
    (static_cast<size_t>(base->num_signals) * sig_size);

  if (header.payload_size_bytes < base_size || header.payload_size_bytes < expected_size) {
    RCLCPP_WARN(node->get_logger(),
                "GNSS_SIGNALS payload too small (size=%u expected>=%zu).",
                header.payload_size_bytes, expected_size);
    return;
  }

  navigation_msgs::GnssSignals msg{*base};
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;

  const uint8_t* data = reinterpret_cast<const uint8_t*>(payload);
  const auto* sats = reinterpret_cast<const point_one::fusion_engine::messages::GNSSSatelliteInfo*>(
    data + base_size);
  msg.satellites.reserve(base->num_satellites);
  for (size_t i = 0; i < base->num_satellites; ++i) {
    navigation_msgs::GnssSatelliteInfo sat_msg{sats[i]};
    msg.satellites.push_back(
      static_cast<const fusion_engine_msgs::msg::GnssSatelliteInfo&>(sat_msg));
  }

  const auto* sigs = reinterpret_cast<const point_one::fusion_engine::messages::GNSSSignalInfo*>(
    data + base_size + (static_cast<size_t>(base->num_satellites) * sat_size));
  msg.signals.reserve(base->num_signals);
  for (size_t i = 0; i < base->num_signals; ++i) {
    navigation_msgs::GnssSignalInfo sig_msg{sigs[i]};
    msg.signals.push_back(
      static_cast<const fusion_engine_msgs::msg::GnssSignalInfo&>(sig_msg));
  }

  pub->publish(static_cast<const fusion_engine_msgs::msg::GnssSignals&>(msg));
}

/******************************************************************************/
inline const auto& kHandlers()
{
  static const std::unordered_map<MessageType, Handler> kHandles = {
    // Navigation
    {MessageType::POSE, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
      handle<point_one::fusion_engine::messages::PoseMessage,
             navigation_msgs::Pose,
             fusion_engine_msgs::msg::Pose>(
        n, "pose_filtered",
        reinterpret_cast<const point_one::fusion_engine::messages::PoseMessage*>(p), id, t);
    }},
    {MessageType::POSE_AUX, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
      handle<point_one::fusion_engine::messages::PoseAuxMessage,
             navigation_msgs::PoseAux,
             fusion_engine_msgs::msg::PoseAux>(
        n, "pose_aux",
        reinterpret_cast<const point_one::fusion_engine::messages::PoseAuxMessage*>(p), id, t);
    }},
    {MessageType::CALIBRATION_STATUS, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
      handle<point_one::fusion_engine::messages::CalibrationStatusMessage,
             navigation_msgs::CalibrationStatus,
             fusion_engine_msgs::msg::CalibrationStatus>(
        n, "calibration_status",
        reinterpret_cast<const point_one::fusion_engine::messages::CalibrationStatusMessage*>(p), id, t);
    }},
    {MessageType::GNSS_INFO, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
      handle<point_one::fusion_engine::messages::GNSSInfoMessage,
             navigation_msgs::GnssInfo,
             fusion_engine_msgs::msg::GnssInfo>(
        n, "gnss_info",
        reinterpret_cast<const point_one::fusion_engine::messages::GNSSInfoMessage*>(p), id, t);
    }},
    {MessageType::RELATIVE_ENU_POSITION, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
      handle<point_one::fusion_engine::messages::RelativeENUPositionMessage,
             navigation_msgs::RelativeEnuPosition,
             fusion_engine_msgs::msg::RelativeEnuPosition>(
        n, "relative_enu_position",
        reinterpret_cast<const point_one::fusion_engine::messages::RelativeENUPositionMessage*>(p), id, t);
    }},

    // Calibrated
    {MessageType::IMU_OUTPUT, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
      handle<point_one::fusion_engine::messages::IMUOutput,
             calibrated_msgs::Imu,
             sensor_msgs::msg::Imu>(
        n, "imu_calibrated",
        reinterpret_cast<const point_one::fusion_engine::messages::IMUOutput*>(p), id, t);
    }},
    {MessageType::GNSS_ATTITUDE_OUTPUT, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
      handle<point_one::fusion_engine::messages::GNSSAttitudeOutput,
             calibrated_msgs::GnssAttitudeOutput,
             fusion_engine_msgs::msg::GnssAttitudeOutput>(
        n, "gnss_attitude",
        reinterpret_cast<const point_one::fusion_engine::messages::GNSSAttitudeOutput*>(p), id, t);
    }},
    {MessageType::WHEEL_SPEED_OUTPUT, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
      handle<point_one::fusion_engine::messages::WheelSpeedOutput,
             calibrated_msgs::WheelSpeedOutput,
             fusion_engine_msgs::msg::WheelSpeedOutput>(
        n, "wheel_speed",
        reinterpret_cast<const point_one::fusion_engine::messages::WheelSpeedOutput*>(p), id, t);
    }},
    {MessageType::VEHICLE_SPEED_OUTPUT, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
      handle<point_one::fusion_engine::messages::VehicleSpeedOutput,
             calibrated_msgs::VehicleSpeedOutput,
             fusion_engine_msgs::msg::VehicleSpeedOutput>(
        n, "vehicle_speed",
        reinterpret_cast<const point_one::fusion_engine::messages::VehicleSpeedOutput*>(p), id, t);
    }},

    // Raw
    {MessageType::RAW_IMU_OUTPUT, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
      handle<point_one::fusion_engine::messages::RawIMUOutput,
             raw_msgs::RawImu,
             sensor_msgs::msg::Imu>(
        n, "imu_raw",
        reinterpret_cast<const point_one::fusion_engine::messages::RawIMUOutput*>(p), id, t);
    }},
    {MessageType::RAW_WHEEL_TICK_OUTPUT, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
      handle<point_one::fusion_engine::messages::RawWheelTickOutput,
             raw_msgs::RawWheelTickOutput,
             fusion_engine_msgs::msg::RawWheelTickOutput>(
        n, "wheel_tick_raw",
        reinterpret_cast<const point_one::fusion_engine::messages::RawWheelTickOutput*>(p), id, t);
    }},

    // ROS-standard
    {MessageType::ROS_POSE, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
      handle<point_one::fusion_engine::messages::ros::PoseMessage,
             ros_msgs::Pose,
             geometry_msgs::msg::PoseStamped>(
        n, "pose_ros",
        reinterpret_cast<const point_one::fusion_engine::messages::ros::PoseMessage*>(p), id, t);
    }},
    {MessageType::ROS_GPS_FIX, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
      handle<point_one::fusion_engine::messages::ros::GPSFixMessage,
             ros_msgs::GPSFix,
             gps_msgs::msg::GPSFix>(
        n, "gpsfix_ros",
        reinterpret_cast<const point_one::fusion_engine::messages::ros::GPSFixMessage*>(p), id, t);
    }},
    {MessageType::ROS_IMU, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
      handle<point_one::fusion_engine::messages::ros::IMUMessage,
             ros_msgs::Imu,
             sensor_msgs::msg::Imu>(
        n, "imu_ros",
        reinterpret_cast<const point_one::fusion_engine::messages::ros::IMUMessage*>(p), id, t);
    }},
  };
  return kHandles;
}

/******************************************************************************/
inline const auto& kSBF()
{
  static const std::unordered_map<SBFBlockID, SBFHandler> kSBFHandles = {
    {SBFBlockID::PVTGeodetic, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
      handle<::PVTGeodetic,
             sbf_msgs::PVTGeodetic,
             fusion_engine_msgs::msg::PVTGeodetic>(
        n, "pvt_geodetic", reinterpret_cast<const ::PVTGeodetic*>(p), id, t);
    }},
    {SBFBlockID::PVTCartesian, [](auto* n, auto* p, const std::string& id, const rclcpp::Time& t){
      handle<::PVTCartesian,
             sbf_msgs::PVTCartesian,
             fusion_engine_msgs::msg::PVTCartesian>(
        n, "pvt_cartesian", reinterpret_cast<const ::PVTCartesian*>(p), id, t);
    }},
  };
  return kSBFHandles;
}

/******************************************************************************/
// Pull the device measurement time (p1_time, seconds) out of a payload for the
// message types that carry `Timestamp p1_time` as their first member (empty
// MessagePayload base => it sits at payload offset 0). All feed one P1 clock,
// so they share the estimator cleanly. NOTE: RAW_IMU_OUTPUT / GNSS_ATTITUDE_OUTPUT
// lead with MeasurementDetails.measurement_time, which is a DIFFERENT time base
// (needs measurement_time_source conversion) -- feeding it raw poisons the shared
// offset, so those keep receipt time until converted properly.
inline bool extractP1TimeSeconds(const MessageHeader& header,
                                 const void* payload,
                                 double& out)
{
  switch (header.message_type) {
    case MessageType::POSE:
    case MessageType::POSE_AUX:
    case MessageType::CALIBRATION_STATUS:
    case MessageType::GNSS_INFO:
    case MessageType::GNSS_SIGNALS:
    case MessageType::RELATIVE_ENU_POSITION:
    case MessageType::IMU_OUTPUT:
    case MessageType::WHEEL_SPEED_OUTPUT:
    case MessageType::VEHICLE_SPEED_OUTPUT:
    case MessageType::ROS_POSE:
    case MessageType::ROS_GPS_FIX:
    case MessageType::ROS_IMU:
      break;
    default:
      return false;
  }
  const auto* ts = reinterpret_cast<const Timestamp*>(payload);
  if (ts->seconds == Timestamp::INVALID) {
    return false;
  }
  out = static_cast<double>(ts->seconds) +
        static_cast<double>(ts->fraction_ns) * 1e-9;
  return true;
}

/******************************************************************************/
inline const Handler& findHandler(const MessageHeader& header)
{
  static const Handler kNoOp = [](auto*, auto*, const std::string&, const rclcpp::Time&) {};

  if (header.message_type == MessageType::GNSS_SIGNALS) {
    static Handler kGnssSignalsOp;
    kGnssSignalsOp = [&header](auto* n, auto* p, const std::string& f, const rclcpp::Time& t) {
      handleGnssSignals(n, header, p, f, t);
    };
    return kGnssSignalsOp;
  }

  const auto& table = kHandlers();
  const auto it = table.find(header.message_type);
  if (it != table.end())
    return it->second;
  
  if (header.message_type == MessageType::INPUT_DATA_WRAPPER) {
    static Handler kSBFOp;
    kSBFOp = [&header](auto* n, auto* p, const std::string& f, const rclcpp::Time& t) {
      auto& contents = *reinterpret_cast<const point_one::fusion_engine::messages::InputDataWrapperMessage*>(p);
      if (contents.data_type != static_cast<uint16_t>(InputDataType::SBF_DATA))
        return;

      const uint8_t* inner_payload = reinterpret_cast<const uint8_t*>(p) + sizeof(InputDataWrapperMessage);
      const size_t inner_size = header.payload_size_bytes - sizeof(InputDataWrapperMessage);
      if (!isSBF(inner_payload, inner_size))
        return;

      const uint16_t block_id = inner_payload[4] | (inner_payload[5] << 8);
      const uint16_t block_num = block_id & 0x1FFF;
      const auto it = kSBF().find(static_cast<SBFBlockID>(block_num));
      if (it != kSBF().end()) {
        it->second(n, inner_payload + 8, f, t);
      } else {
        RCLCPP_DEBUG(n->get_logger(),
                     "No registered SBF handler for block 0x%04X (%s)",
                     block_num, to_string(block_num).c_str());
      }
    };
    return kSBFOp;
  }
  
  return kNoOp;
}

#endif  // FUSION_ENGINE_DRIVER__CORE__FUSION_DISPATCH_HPP_
