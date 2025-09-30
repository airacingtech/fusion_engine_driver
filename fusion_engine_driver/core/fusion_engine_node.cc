#include "fusion_engine_node.hpp"

/******************************************************************************/
FusionEngineNode::FusionEngineNode()
    : Node("fusion_engine_node"),
      fe_interface_(std::bind(&FusionEngineNode::receivedFusionEngineMessage,
                              this, std::placeholders::_1,
                              std::placeholders::_2)) {
  this->declare_parameter("udp_port", 12345);
  this->declare_parameter("connection_type", "tcp");
  this->declare_parameter("tcp_ip", "localhost");
  this->declare_parameter("tty_port", "/dev/ttyUSB0");
  this->declare_parameter("tcp_port", 12345);
  this->declare_parameter("debug", false);
  this->declare_parameter("frame_id", "cg");
  frame_id_ = this->get_parameter("frame_id").as_string();
  timer_ =
      create_wall_timer(std::chrono::milliseconds(1),
                        std::bind(&FusionEngineNode::rosServiceLoop, this));

  if (this->has_parameter("connection_type")) {
    std::string argValue(this->get_parameter("connection_type").as_string());
    if (argValue == "tcp") {
      fe_interface_.initialize(this, this->get_parameter("tcp_ip").as_string(),
                               this->get_parameter("tcp_port").as_int());
      dataListenerService();
    } else if (argValue == "tty") {
      nmea_publisher_ = this->create_publisher<nmea_msgs::msg::Sentence>(
          "ntrip_client/nmea", 10);
      subscription_ = this->create_subscription<mavros_msgs::msg::RTCM>(
          "ntrip_client/rtcm", 10,
          [this](const mavros_msgs::msg::RTCM::SharedPtr msg) {
            if (this->get_parameter("debug").as_bool())
              RCLCPP_INFO(this->get_logger(), "RTCM message received.");
            fe_interface_.write(msg->data.data(), msg->data.size());
          });
      fe_interface_.initialize(this,
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
FusionEngineNode::~FusionEngineNode() {
  if (listener_thread_.joinable()) {
    fe_interface_.stop();
    listener_thread_.join();
  }
}

/******************************************************************************/
using Factory = std::function<void(FusionEngineNode*, const void* msg)>;
static const std::unordered_map<MessageType, Factory>& kFactory() {
  static const std::unordered_map<MessageType, Factory> kTable = {
      // Navigation Outputs
    {MessageType::POSE, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<fusion_engine_msgs::msg::Pose>(
        "pose_filtered", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const fusion_engine_msgs::msg::Pose*>(msg));
    }},
    {MessageType::POSE_AUX, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<fusion_engine_msgs::msg::PoseAux>(
        "pose_aux", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const fusion_engine_msgs::msg::PoseAux*>(msg));
    }},
    {MessageType::CALIBRATION_STATUS, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<fusion_engine_msgs::msg::CalibrationStatus>(
        "calibration_status", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const fusion_engine_msgs::msg::CalibrationStatus*>(msg));
    }},
    {MessageType::GNSS_INFO, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<fusion_engine_msgs::msg::GnssInfo>(
        "gnss_info", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const fusion_engine_msgs::msg::GnssInfo*>(msg));
    }},
    {MessageType::GNSS_SATELLITE, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<fusion_engine_msgs::msg::GnssSatellite>(
        "gnss_satellite", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const fusion_engine_msgs::msg::GnssSatellite*>(msg));
    }},
    {MessageType::RELATIVE_ENU_POSITION, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<fusion_engine_msgs::msg::RelativeEnuPosition>(
        "relative_enu_position", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const fusion_engine_msgs::msg::RelativeEnuPosition*>(msg));
    }},

    // Calibrated Outputs
    {MessageType::IMU_OUTPUT, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<sensor_msgs::msg::Imu>(
        "imu_calibrated", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const sensor_msgs::msg::Imu*>(msg));
    }},
    {MessageType::GNSS_ATTITUDE_OUTPUT, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<fusion_engine_msgs::msg::GnssAttitudeOutput>(
        "gnss_attitude", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const fusion_engine_msgs::msg::GnssAttitudeOutput*>(msg));
    }},
    {MessageType::WHEEL_SPEED_OUTPUT, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<fusion_engine_msgs::msg::WheelSpeedOutput>(
        "wheel_speed", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const fusion_engine_msgs::msg::WheelSpeedOutput*>(msg));
    }},
    {MessageType::VEHICLE_SPEED_OUTPUT, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<fusion_engine_msgs::msg::VehicleSpeedOutput>(
        "vehicle_speed", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const fusion_engine_msgs::msg::VehicleSpeedOutput*>(msg));
    }},

    // Raw Outputs
    {MessageType::RAW_IMU_OUTPUT, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<sensor_msgs::msg::Imu>(
          "imu_raw", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const sensor_msgs::msg::Imu*>(msg));
    }},
    {MessageType::RAW_GNSS_ATTITUDE_OUTPUT, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<fusion_engine_msgs::msg::RawGnssAttitudeOutput>(
          "gnss_attitude_raw", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const fusion_engine_msgs::msg::RawGnssAttitudeOutput*>(msg));
    }},
    {MessageType::RAW_WHEEL_TICK_OUTPUT, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<fusion_engine_msgs::msg::RawWheelTickOutput>(
          "wheel_tick_raw", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const fusion_engine_msgs::msg::RawWheelTickOutput*>(msg));
    }},
    {MessageType::RAW_VEHICLE_TICK_OUTPUT, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<fusion_engine_msgs::msg::RawVehicleTickOutput>(
          "vehicle_tick_raw", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const fusion_engine_msgs::msg::RawVehicleTickOutput*>(msg));
    }},
    {MessageType::RAW_WHEEL_SPEED_OUTPUT, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<fusion_engine_msgs::msg::RawWheelSpeedOutput>(
          "wheel_speed_raw", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const fusion_engine_msgs::msg::RawWheelSpeedOutput*>(msg));
    }},
    {MessageType::RAW_VEHICLE_SPEED_OUTPUT, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<fusion_engine_msgs::msg::RawVehicleSpeedOutput>(
          "vehicle_speed_raw", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const fusion_engine_msgs::msg::RawVehicleSpeedOutput*>(msg));
    }},
    // Note: No publisher for RawWheelTickOutput (11123) and RawVehicleTickOutput (11124) in original code
    // ROS Outputs
    {MessageType::ROS_POSE, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<geometry_msgs::msg::PoseStamped>(
      "pose_ros", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const geometry_msgs::msg::PoseStamped*>(msg));
    }},
    {MessageType::ROS_GPS_FIX, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<gps_msgs::msg::GPSFix>(
      "gpsfix_ros", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const gps_msgs::msg::GPSFix*>(msg));
    }},
    {MessageType::ROS_IMU, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<sensor_msgs::msg::Imu>(
      "imu_ros", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const sensor_msgs::msg::Imu*>(msg));
    }},
  };
  return kTable;
}

/******************************************************************************/
void FusionEngineNode::receivedFusionEngineMessage(const MessageHeader &header,
                                                   const void *payload) {
  auto time = now();
  auto type = header.message_type;
  switch(type) {
    // Navigation Solutions
    case MessageType::POSE:
      {
        if (this->get_parameter("connection_type").as_string() == "tty") {
          auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::PoseMessage *>(payload);
          double gps_time_sec = contents.gps_time.seconds + contents.gps_time.fraction_ns * 1e-9;
          if (gps_time_sec - previous_gps_time_sec_ > TIME_BETWEEN_NMEA_UPDATES_SEC_) {
            nmea_msgs::msg::Sentence nmea = ConversionUtils::toNMEA(contents, satellite_nb_);
            nmea.header.stamp = this->now();
            nmea.header.frame_id = "gps";
            previous_gps_time_sec_ = gps_time_sec;
            nmea_publisher_->publish(nmea);
          }
        }
        else {
          auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::PoseMessage *>(payload);
          fusion_engine_msgs::msg::Pose msg = ConversionUtils::populate(contents);
          msg.header.frame_id = frame_id_;
          msg.header.stamp = time;
          kFactory().at(type)(this, &msg);
        }
        break;
      }
    case MessageType::GNSS_INFO:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::GNSSInfoMessage *>(payload);
        fusion_engine_msgs::msg::GnssInfo msg = ConversionUtils::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::GNSS_SATELLITE:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::GNSSSatelliteMessage *>(payload);
        fusion_engine_msgs::msg::GnssSatellite msg = ConversionUtils::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::POSE_AUX:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::PoseAuxMessage *>(payload);
        fusion_engine_msgs::msg::PoseAux msg = ConversionUtils::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::CALIBRATION_STATUS:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::CalibrationStatusMessage *>(payload);
        fusion_engine_msgs::msg::CalibrationStatus msg = ConversionUtils::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::RELATIVE_ENU_POSITION:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::RelativeENUPositionMessage *>(payload);
        fusion_engine_msgs::msg::RelativeEnuPosition msg = ConversionUtils::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    // Calibrated Sensors
    case MessageType::IMU_OUTPUT:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::IMUOutput *>(payload);
        sensor_msgs::msg::Imu msg = ConversionUtils::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::GNSS_ATTITUDE_OUTPUT:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::GNSSAttitudeOutput *>(payload);
        fusion_engine_msgs::msg::GnssAttitudeOutput msg = ConversionUtils::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::WHEEL_SPEED_OUTPUT:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::WheelSpeedOutput *>(payload);
        fusion_engine_msgs::msg::WheelSpeedOutput msg = ConversionUtils::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::VEHICLE_SPEED_OUTPUT:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::VehicleSpeedOutput *>(payload);
        fusion_engine_msgs::msg::VehicleSpeedOutput msg = ConversionUtils::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    // Raw Sensors
    case MessageType::RAW_IMU_OUTPUT:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::RawIMUOutput *>(payload);
        sensor_msgs::msg::Imu msg = ConversionUtils::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::RAW_GNSS_ATTITUDE_OUTPUT:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::RawGNSSAttitudeOutput *>(payload);
        fusion_engine_msgs::msg::RawGnssAttitudeOutput msg = ConversionUtils::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::RAW_WHEEL_TICK_OUTPUT:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::RawWheelTickOutput *>(payload);
        fusion_engine_msgs::msg::RawWheelTickOutput msg = ConversionUtils::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::RAW_VEHICLE_TICK_OUTPUT:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::RawVehicleTickOutput *>(payload);
        fusion_engine_msgs::msg::RawVehicleTickOutput msg = ConversionUtils::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::RAW_WHEEL_SPEED_OUTPUT:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::RawWheelSpeedOutput *>(payload);
        fusion_engine_msgs::msg::RawWheelSpeedOutput msg = ConversionUtils::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::RAW_VEHICLE_SPEED_OUTPUT:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::RawVehicleSpeedOutput *>(payload);
        fusion_engine_msgs::msg::RawVehicleSpeedOutput msg = ConversionUtils::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    // ROS Messages
    case MessageType::ROS_POSE:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::ros::PoseMessage *>(payload);
        geometry_msgs::msg::PoseStamped msg = ConversionUtils::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
      break;
      }
    case MessageType::ROS_GPS_FIX:
      {
        auto &contents = *reinterpret_cast<const GPSFixMessage *>(payload);
        gps_msgs::msg::GPSFix msg = ConversionUtils::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
      break;
      }
    case MessageType::ROS_IMU:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::ros::IMUMessage *>(payload);
        sensor_msgs::msg::Imu msg = ConversionUtils::populate(contents);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
      break;
      }
    default:
      break;
  }
}

/******************************************************************************/
void FusionEngineNode::rosServiceLoop() {
  RCLCPP_INFO(this->get_logger(), "Service");
  timer_->cancel();
}

/******************************************************************************/
void FusionEngineNode::dataListenerService() {
  fe_interface_.dataListenerService();
}
