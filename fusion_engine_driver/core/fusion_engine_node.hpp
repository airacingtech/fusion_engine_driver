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
#include "clock_sync.hpp"
#include "race_msgs/msg/wheel_speed_report.hpp"

class FusionEngineNode : public rclcpp::Node
{
public:
  explicit FusionEngineNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
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
  rclcpp::Subscription<race_msgs::msg::WheelSpeedReport>::SharedPtr wheel_speed_sub_;

  bool enable_wheel_speed_input_;
  uint32_t wheel_input_seq_ = 0;

  uint16_t satellite_nb_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::string frame_id_;

  // p1 -> GPS clock offset, taken exactly from POSE messages that carry the
  // same instant on both clocks. ClockSync remains the GPS-denied fallback.
  int64_t p1_to_gps_ns_ = 0;
  bool have_p1_to_gps_ = false;
  int64_t last_p1_ns_ = 0;
  bool have_last_p1_ = false;

  std::unique_ptr<art::ClockSync> clock_sync_;


  std::thread listener_thread_;

  double previous_gps_time_sec_ = 0;


  static constexpr double TIME_BETWEEN_NMEA_UPDATES_SEC_{30.0};

  void dataListenerService();


  void rosServiceLoop();

  /// Encode a wheel-speed report as a FusionEngine WheelSpeedInput and write it
  /// to the device over the active connection.
  void onWheelSpeedReport(const race_msgs::msg::WheelSpeedReport::SharedPtr msg);
};

#endif  // FUSION_ENGINE_DRIVER__CORE__FUSION_ENGINE_NODE_HPP_