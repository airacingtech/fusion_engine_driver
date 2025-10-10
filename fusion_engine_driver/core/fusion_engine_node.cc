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

#include "fusion_engine_node.hpp"

#pragma pack(push, 1)
struct PVTGeodetic {
  uint32_t TOW;        // GPS Time of Week [ms]
  uint16_t WNc;        // Week number
  uint8_t  Mode;       // PVT mode bits (0–3 solution type, 6=static flag, 7=2D/3D flag)
  uint8_t  Error;      // Error code
  double   latitude;   // degrees
  double   longitude;  // degrees
  double   height;     // m (ellipsoidal)
  float    undulation; // m (geoid height)
  float    vn;         // north velocity (m/s)
  float    ve;         // east velocity (m/s)
  float    vu;         // up velocity (m/s)
  float    cog;        // course over ground (degrees)
  float    RxClkBias;  // receiver clock bias (m)
  float    RxClkDrift; // receiver clock drift (m/s)
  uint8_t  timeSystem; // 0: GPS, 1: GLONASS, 2: Galileo, etc.
  uint8_t  datum;      // datum ID
  uint8_t  nrSV;       // number of satellites used
  uint8_t  WACorrInfo; // differential/RTK correction flags
  uint16_t referenceID;// base station ID
  uint16_t meanCorrAge;// correction age [0.01s]
  uint16_t signalInfo; // bitmask of used signals
  uint8_t  alertFlag;  // RAIM/integrity bits
  uint8_t  NrBases;    // number of base stations used
  uint16_t PPPInfo;    // PPP seed age/type bits
  uint16_t latency;    // 0.0001s units
  uint16_t HAccuracy;  // 0.01 m units (2DRMS horizontal)
  uint16_t VAccuracy;  // 0.01 m units (2σ vertical)
  uint8_t  misc;       // flags: baseline ref, ARP compensation, etc.
  uint8_t  padding[3]; // to make total length = 96 bytes
};
#pragma pack(pop)


/******************************************************************************/
FusionEngineNode::FusionEngineNode()
: Node("fusion_engine_node"),
  fe_interface_(std::bind(&FusionEngineNode::receivedFusionEngineMessage,
    this, std::placeholders::_1,
    std::placeholders::_2))
{
  this->declare_parameter("udp_port", 12345);
  this->declare_parameter("connection_type", "tcp");
  this->declare_parameter("tcp_ip", "localhost");
  this->declare_parameter("tty_port", "/dev/ttyUSB0");
  this->declare_parameter("tcp_port", 12345);
  this->declare_parameter("debug", false);
  this->declare_parameter("frame_id", "cg");
  frame_id_ = this->get_parameter("frame_id").as_string();
  timer_ =
    create_wall_timer(
    std::chrono::milliseconds(1),
    std::bind(&FusionEngineNode::rosServiceLoop, this));

  if (this->has_parameter("connection_type")) {
    std::string argValue(this->get_parameter("connection_type").as_string());
    if (argValue == "tcp") {
      fe_interface_.initialize(
        this, this->get_parameter("tcp_ip").as_string(),
        this->get_parameter("tcp_port").as_int());
      dataListenerService();
    } else if (argValue == "tty") {
      nmea_publisher_ = this->create_publisher < nmea_msgs::msg::Sentence > (
        "ntrip_client/nmea", 10);
      subscription_ = this->create_subscription < mavros_msgs::msg::RTCM > (
        "ntrip_client/rtcm", 10,
        [this](const mavros_msgs::msg::RTCM::SharedPtr msg) {
        if (this->get_parameter("debug").as_bool()) {
          RCLCPP_INFO(this->get_logger(), "RTCM message received.");
        }
        fe_interface_.write(msg->data.data(), msg->data.size());
      });
      fe_interface_.initialize(
        this,
        this->get_parameter("tty_port").as_string());
      listener_thread_ =
        std::thread(std::bind(&FusionEngineNode::dataListenerService, this));
    } else {
      std::cout << "Invalid args" << std::endl;
      rclcpp::shutdown();
    }
  } else {
    std::cout << "Invalid args" << std::endl;
    rclcpp::shutdown();
  }
}

/******************************************************************************/
FusionEngineNode::~FusionEngineNode()
{
  if (listener_thread_.joinable()) {
    fe_interface_.stop();
    listener_thread_.join();
  }
}

/******************************************************************************/
using Factory = std::function < void(FusionEngineNode *, const void * msg) >;
static const std::unordered_map < MessageType, Factory > & kFactory() {
  static const std::unordered_map < MessageType, Factory > kTable = {
    // Navigation Outputs
    {MessageType::POSE, [] (FusionEngineNode * n, const void * msg) {
        static auto pub = n->create_publisher < fusion_engine_msgs::msg::Pose > (
          "pose_filtered", rclcpp::SensorDataQoS());
        pub->publish(*reinterpret_cast < const fusion_engine_msgs::msg::Pose * > (msg));
      }},
    {MessageType::POSE_AUX, [] (FusionEngineNode * n, const void * msg) {
        static auto pub = n->create_publisher < fusion_engine_msgs::msg::PoseAux > (
          "pose_aux", rclcpp::SensorDataQoS());
        pub->publish(*reinterpret_cast < const fusion_engine_msgs::msg::PoseAux * > (msg));
      }},
    {MessageType::CALIBRATION_STATUS, [] (FusionEngineNode * n, const void * msg) {
        static auto pub = n->create_publisher < fusion_engine_msgs::msg::CalibrationStatus > (
          "calibration_status", rclcpp::SensorDataQoS());
        pub->publish(
          *reinterpret_cast < const fusion_engine_msgs::msg::CalibrationStatus * >
          (msg));
      }},
    {MessageType::GNSS_INFO, [] (FusionEngineNode * n, const void * msg) {
        static auto pub = n->create_publisher < fusion_engine_msgs::msg::GnssInfo > (
          "gnss_info", rclcpp::SensorDataQoS());
        pub->publish(*reinterpret_cast < const fusion_engine_msgs::msg::GnssInfo * > (msg));
      }},
    {MessageType::GNSS_SATELLITE, [] (FusionEngineNode * n, const void * msg) {
        static auto pub = n->create_publisher < fusion_engine_msgs::msg::GnssSatellite > (
          "gnss_satellite", rclcpp::SensorDataQoS());
        pub->publish(*reinterpret_cast < const fusion_engine_msgs::msg::GnssSatellite * > (msg));
      }},
    {MessageType::RELATIVE_ENU_POSITION, [] (FusionEngineNode * n, const void * msg) {
        static auto pub = n->create_publisher < fusion_engine_msgs::msg::RelativeEnuPosition > (
          "relative_enu_position", rclcpp::SensorDataQoS());
        pub->publish(
          *reinterpret_cast < const fusion_engine_msgs::msg::RelativeEnuPosition * > (msg));
      }},

    // Calibrated Outputs
    {MessageType::IMU_OUTPUT, [] (FusionEngineNode * n, const void * msg) {
        static auto pub = n->create_publisher < sensor_msgs::msg::Imu > (
          "imu_calibrated", rclcpp::SensorDataQoS());
        pub->publish(*reinterpret_cast < const sensor_msgs::msg::Imu * > (msg));
      }},
    {MessageType::GNSS_ATTITUDE_OUTPUT, [] (FusionEngineNode * n, const void * msg) {
        static auto pub = n->create_publisher < fusion_engine_msgs::msg::GnssAttitudeOutput > (
          "gnss_attitude", rclcpp::SensorDataQoS());
        pub->publish(
          *reinterpret_cast < const fusion_engine_msgs::msg::GnssAttitudeOutput * >
          (msg));
      }},
    {MessageType::WHEEL_SPEED_OUTPUT, [] (FusionEngineNode * n, const void * msg) {
        static auto pub = n->create_publisher < fusion_engine_msgs::msg::WheelSpeedOutput > (
          "wheel_speed", rclcpp::SensorDataQoS());
        pub->publish(*reinterpret_cast < const fusion_engine_msgs::msg::WheelSpeedOutput * > (msg));
      }},
    {MessageType::VEHICLE_SPEED_OUTPUT, [] (FusionEngineNode * n, const void * msg) {
        static auto pub = n->create_publisher < fusion_engine_msgs::msg::VehicleSpeedOutput > (
          "vehicle_speed", rclcpp::SensorDataQoS());
        pub->publish(
          *reinterpret_cast < const fusion_engine_msgs::msg::VehicleSpeedOutput * >
          (msg));
      }},

    // Raw Outputs
    {MessageType::RAW_IMU_OUTPUT, [] (FusionEngineNode * n, const void * msg) {
        static auto pub = n->create_publisher < sensor_msgs::msg::Imu > (
          "imu_raw", rclcpp::SensorDataQoS());
        pub->publish(*reinterpret_cast < const sensor_msgs::msg::Imu * > (msg));
      }},
    {MessageType::RAW_GNSS_ATTITUDE_OUTPUT, [] (FusionEngineNode * n, const void * msg) {
        static auto pub = n->create_publisher < fusion_engine_msgs::msg::RawGnssAttitudeOutput > (
          "gnss_attitude_raw", rclcpp::SensorDataQoS());
        pub->publish(
          *reinterpret_cast < const fusion_engine_msgs::msg::RawGnssAttitudeOutput * >
          (msg));
      }},
    {MessageType::RAW_WHEEL_TICK_OUTPUT, [] (FusionEngineNode * n, const void * msg) {
        static auto pub = n->create_publisher < fusion_engine_msgs::msg::RawWheelTickOutput > (
          "wheel_tick_raw", rclcpp::SensorDataQoS());
        pub->publish(
          *reinterpret_cast < const fusion_engine_msgs::msg::RawWheelTickOutput * >
          (msg));
      }},
    {MessageType::RAW_VEHICLE_TICK_OUTPUT, [] (FusionEngineNode * n, const void * msg) {
        static auto pub = n->create_publisher < fusion_engine_msgs::msg::RawVehicleTickOutput > (
          "vehicle_tick_raw", rclcpp::SensorDataQoS());
        pub->publish(
          *reinterpret_cast < const fusion_engine_msgs::msg::RawVehicleTickOutput * >
          (msg));
      }},
    {MessageType::RAW_WHEEL_SPEED_OUTPUT, [] (FusionEngineNode * n, const void * msg) {
        static auto pub = n->create_publisher < fusion_engine_msgs::msg::RawWheelSpeedOutput > (
          "wheel_speed_raw", rclcpp::SensorDataQoS());
        pub->publish(
          *reinterpret_cast < const fusion_engine_msgs::msg::RawWheelSpeedOutput * > (msg));
      }},
    {MessageType::RAW_VEHICLE_SPEED_OUTPUT, [] (FusionEngineNode * n, const void * msg) {
        static auto pub = n->create_publisher < fusion_engine_msgs::msg::RawVehicleSpeedOutput > (
          "vehicle_speed_raw", rclcpp::SensorDataQoS());
        pub->publish(
          *reinterpret_cast < const fusion_engine_msgs::msg::RawVehicleSpeedOutput * >
          (msg));
      }},
    // Note: No publisher for RawWheelTickOutput (11123) and RawVehicleTickOutput (11124) in original code
    // ROS Outputs
    {MessageType::ROS_POSE, [] (FusionEngineNode * n, const void * msg) {
        static auto pub = n->create_publisher < geometry_msgs::msg::PoseStamped > (
          "pose_ros", rclcpp::SensorDataQoS());
        pub->publish(*reinterpret_cast < const geometry_msgs::msg::PoseStamped * > (msg));
      }},
    {MessageType::ROS_GPS_FIX, [] (FusionEngineNode * n, const void * msg) {
        static auto pub = n->create_publisher < gps_msgs::msg::GPSFix > (
          "gpsfix_ros", rclcpp::SensorDataQoS());
        pub->publish(*reinterpret_cast < const gps_msgs::msg::GPSFix * > (msg));
      }},
    {MessageType::ROS_IMU, [] (FusionEngineNode * n, const void * msg) {
        static auto pub = n->create_publisher < sensor_msgs::msg::Imu > (
          "imu_ros", rclcpp::SensorDataQoS());
        pub->publish(*reinterpret_cast < const sensor_msgs::msg::Imu * > (msg));
      }},
  };
  return kTable;
}

/******************************************************************************/
void FusionEngineNode::receivedFusionEngineMessage(
  const MessageHeader & header,
  const void * payload)
{
  auto time = now();
  auto type = header.message_type;
  switch (type) {
    // Navigation Solutions
    case MessageType::POSE:
      {
        if (this->get_parameter("connection_type").as_string() == "tty") {
          auto & contents = *reinterpret_cast <
            const point_one::fusion_engine::messages::PoseMessage * > (payload);
          double gps_time_sec = contents.gps_time.seconds + contents.gps_time.fraction_ns * 1e-9;
          if (gps_time_sec - previous_gps_time_sec_ > TIME_BETWEEN_NMEA_UPDATES_SEC_) {
            nmea_msgs::msg::Sentence nmea = Helper::toNMEA(contents, satellite_nb_);
            nmea.header.stamp = this->now();
            nmea.header.frame_id = "gps";
            previous_gps_time_sec_ = gps_time_sec;
            nmea_publisher_->publish(nmea);
          }
        } else {
          auto & contents = *reinterpret_cast <
            const point_one::fusion_engine::messages::PoseMessage * > (payload);
          fusion_engine_msgs::msg::Pose msg = Helper::populate(contents);
          msg.header.frame_id = frame_id_;
          msg.header.stamp = time;
          kFactory().at(type)(this, &msg);
        }
        break;
      }
    case MessageType::GNSS_INFO:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::GNSSInfoMessage * > (payload);
        fusion_engine_msgs::msg::GnssInfo msg = Helper::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::GNSS_SATELLITE:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::GNSSSatelliteMessage * > (payload);
        fusion_engine_msgs::msg::GnssSatellite msg = Helper::populate(contents);
        satellite_nb_ = contents.num_satellites;
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::POSE_AUX:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::PoseAuxMessage * > (payload);
        fusion_engine_msgs::msg::PoseAux msg = Helper::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::CALIBRATION_STATUS:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::CalibrationStatusMessage * > (payload);
        fusion_engine_msgs::msg::CalibrationStatus msg = Helper::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::RELATIVE_ENU_POSITION:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::RelativeENUPositionMessage * > (payload);
        fusion_engine_msgs::msg::RelativeEnuPosition msg = Helper::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    // Calibrated Sensors
    case MessageType::IMU_OUTPUT:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::IMUOutput * > (payload);
        sensor_msgs::msg::Imu msg = Helper::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::GNSS_ATTITUDE_OUTPUT:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::GNSSAttitudeOutput * > (payload);
        fusion_engine_msgs::msg::GnssAttitudeOutput msg = Helper::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::WHEEL_SPEED_OUTPUT:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::WheelSpeedOutput * > (payload);
        fusion_engine_msgs::msg::WheelSpeedOutput msg = Helper::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::VEHICLE_SPEED_OUTPUT:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::VehicleSpeedOutput * > (payload);
        fusion_engine_msgs::msg::VehicleSpeedOutput msg = Helper::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    // Raw Sensors
    case MessageType::RAW_IMU_OUTPUT:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::RawIMUOutput * > (payload);
        sensor_msgs::msg::Imu msg = Helper::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::RAW_GNSS_ATTITUDE_OUTPUT:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::RawGNSSAttitudeOutput * > (payload);
        fusion_engine_msgs::msg::RawGnssAttitudeOutput msg = Helper::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::RAW_WHEEL_TICK_OUTPUT:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::RawWheelTickOutput * > (payload);
        fusion_engine_msgs::msg::RawWheelTickOutput msg = Helper::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::RAW_VEHICLE_TICK_OUTPUT:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::RawVehicleTickOutput * > (payload);
        fusion_engine_msgs::msg::RawVehicleTickOutput msg = Helper::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::RAW_WHEEL_SPEED_OUTPUT:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::RawWheelSpeedOutput * > (payload);
        fusion_engine_msgs::msg::RawWheelSpeedOutput msg = Helper::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::RAW_VEHICLE_SPEED_OUTPUT:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::RawVehicleSpeedOutput * > (payload);
        fusion_engine_msgs::msg::RawVehicleSpeedOutput msg = Helper::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    // ROS Messages
    case MessageType::ROS_POSE:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::ros::PoseMessage * > (payload);
        geometry_msgs::msg::PoseStamped msg = Helper::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::ROS_GPS_FIX:
      {
        auto & contents = *reinterpret_cast < const GPSFixMessage * > (payload);
        gps_msgs::msg::GPSFix msg = Helper::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::ROS_IMU:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::ros::IMUMessage * > (payload);
        sensor_msgs::msg::Imu msg = Helper::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::INPUT_DATA_WRAPPER: {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::InputDataWrapperMessage * > (payload);
          // RCLCPP_INFO(this->get_logger(),
          //     "InputDataWrapper: data_type=0x%04X (%u)",
          //     contents.data_type, contents.data_type);

  if (contents.data_type == 0x00A1) {
    const uint8_t* inner_payload =
    reinterpret_cast<const uint8_t*>(&contents) + sizeof(InputDataWrapperMessage);
    size_t inner_size = header.payload_size_bytes - sizeof(InputDataWrapperMessage);

    if (inner_payload[0] == 0x24 && inner_payload[1] == 0x40) {
      uint16_t crc      = inner_payload[2] | (inner_payload[3] << 8);
      uint16_t block_id = inner_payload[4] | (inner_payload[5] << 8);
      uint16_t length   = inner_payload[6] | (inner_payload[7] << 8);

      uint16_t block_num = block_id & 0x1FFF;     // bits 0–12
      uint8_t  revision  = (block_id >> 13) & 0x7;// bits 13–15
      if(block_num == 4007){
        const auto* pvt = reinterpret_cast<const PVTGeodetic*>(inner_payload + 8); // skip 8-byte SBF header
RCLCPP_INFO(get_logger(), "Lat: %.8f, Lon: %.8f, Height: %.3f, Vn: %.3f, Ve: %.3f",
            pvt->latitude, pvt->longitude, pvt->height, pvt->vn, pvt->ve);
      }
    //   RCLCPP_INFO(this->get_logger(),
    // "SBF block detected: ID=0x%04X (%s, rev=%u), length=%u, CRC=0x%04X",
    // block_id, Helper::to_string(block_num).c_str(), revision, length, crc);
    }
    //Helper::dumpHex(this->get_logger(), header, inner_size, "SBF Payload");
  }
  break;
      }
    default:
      break;
  }
}

/******************************************************************************/
void FusionEngineNode::rosServiceLoop()
{
  RCLCPP_INFO(this->get_logger(), "Service");
  timer_->cancel();
}

/******************************************************************************/
void FusionEngineNode::dataListenerService()
{
  fe_interface_.dataListenerService();
}

