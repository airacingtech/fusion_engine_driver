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