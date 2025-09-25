#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <unordered_map>

#include "fusion_engine_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "gps_msgs/msg/gps_status.hpp"
#include "mavros_msgs/msg/rtcm.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

/**
 * @brief Point One Nav Atlas Node publishes realtime GPSFix/IMU/Pose messages.
 */
class FusionEngineNode : public rclcpp::Node {
 public:
  FusionEngineNode();
  ~FusionEngineNode();

  /**
   * @brief Receive Fusion Message, build and post them in ros system.
   *
   * @param header Message header with type of message.
   * @param payload Message content
   */
  void receivedFusionEngineMessage(const MessageHeader &header,
                                   const void *payload);

  /**
   * Translate GPSFix to NavFixMsg
   * @param gps_fix Atlas gps data point.
   */
  void publishNavFixMsg(const gps_msgs::msg::GPSFix &gps_fix);

 private:
  /**
   * @brief Interface made to manage the reception of the fusion engines's
   * messages.
   *
   */
  FusionEngineInterface fe_interface_;
  
  /**
   * @brief Publishers for Navigation Messages
   * Pose (10000)
   * GNSSInfo (10001)
   * GNSSSatellite (10002)
   * PoseAux (10003)
   * CalibrationStatus (10004)
   * RelativeENUPosition (10005)
   */
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr filtered_pose_pub_;
/*  rclcpp::Publisher<>::SharedPtr gnss_info_pub_;
  rclcpp::Publisher<>::SharedPtr gnss_satellite_pub_;
  rclcpp::Publisher<>::SharedPtr pose_aux_pub_;
  rclcpp::Publisher<>::SharedPtr calibration_pub_;
  rclcpp::Publisher<>::SharedPtr relative_enu_pub_;
*/
  /**
   * @brief Publishers for Calibrated Messages
   * IMUOutput (11000)
   * GNSSAttitudeOutput (11005)
   * WheelSpeedOutput (11135)
   * VehicleSpeedOutput (11136)
   */
  /*rclcpp::Publisher<>::SharedPtr calibrated_imu_pub_;
  rclcpp::Publisher<>::SharedPtr calibrated_gnss_pub_;
  rclcpp::Publisher<>::SharedPtr calibrated_ws_pub_;
  rclcpp::Publisher<>::SharedPtr calibrated_ws_pub_;
*/
  /**
   * @brief Publishers for Raw Messages
   * RawIMUOutput (11002)
   * RawGNSSAttitudeOutput (11006)
   * RawWheelTickOutput (11123)
   * RawVehicleTickOutput (11124)
   * RawWheelSpeedOutput (11125)
   * RawVehicleSpeedOutput (11126)
   * InputDataWrapper (13120)
   */
  /*rclcpp::Publisher<>::SharedPtr raw_imu_pub_;
  rclcpp::Publisher<>::SharedPtr raw_gnss_pub_;
  rclcpp::Publisher<>::SharedPtr raw_wt_pub_;
  rclcpp::Publisher<>::SharedPtr raw_vt_pub_;
  rclcpp::Publisher<>::SharedPtr raw_ws_pub_;
  rclcpp::Publisher<>::SharedPtr raw_vs_pub_;
  rclcpp::Publisher<>::SharedPtr binary_pub_;
*/
  /**
   * @brief Publishers for ROS Messages
   * ROSPose (12000)
   * ROSGPSFix (12010)
   * ROSIMUMessage (12011)
   */
  /*rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ros_pose_pub_;
  rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr ros_gpsfix_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr ros_imu_pub_;
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
   * @brief Smart pointer to a ROS 2 publisher for marker data.
   *
   * This smart pointer provides access to a ROS 2 publisher, which is used to
   * publish marker data on topic /visualization_marker. The publisher is
   * represented by the `rclcpp::Publisher<visualization_msgs::msg::Marker>`
   * type.
   */
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_publisher_;

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
   * @brief A smart pointer to a timer object, which allows scheduling periodic
   * callbacks.
   */
  rclcpp::TimerBase::SharedPtr timer_;
  
  /**
   * @brief Number of satellite used for create nmea message.
   */
  uint16_t satellite_nb_;

  /**
   * @brief Id of the frame send in ros.
   */
  std::string frame_id_;

  /**
   * @brief Number of each marker send in ros.
   */
  int id = 0;

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
