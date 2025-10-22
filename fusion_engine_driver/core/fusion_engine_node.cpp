#include "fusion_engine_node.hpp"

/******************************************************************************/
FusionEngineNode::FusionEngineNode()
: Node("fusion_engine_node"),
  fe_interface_(std::bind(&FusionEngineNode::handleFusionMessage,
    this, std::placeholders::_1,
    std::placeholders::_2))
{
  connection_type_ = declare_parameter("connection_type", "tcp");
  ip_ = declare_parameter("ip", "localhost");
  port_ = declare_parameter("port", 30200);
  device_ = declare_parameter("device", "/dev/ttyUSB0");
  debug_ = declare_parameter("debug", false);
  frame_id_ = declare_parameter("frame_id", "cg");
  timer_ = create_wall_timer(
    std::chrono::milliseconds(1),
    std::bind(&FusionEngineNode::rosServiceLoop, this));

  try {
    RCLCPP_INFO(get_logger(), "Initializing FusionEngineNode...");
    RCLCPP_INFO(get_logger(), "Connection type: %s", connection_type_.c_str());

    if (connection_type_ == "tty") {
      RCLCPP_INFO(get_logger(), "Device: %s", device_.c_str());

      nmea_publisher_ = this->create_publisher < nmea_msgs::msg::Sentence > (
        "ntrip_client/nmea", 10);
      subscription_ = this->create_subscription < mavros_msgs::msg::RTCM > (
        "ntrip_client/rtcm", 10,
        [this](const mavros_msgs::msg::RTCM::SharedPtr msg) {
        if (this->debug_) {
          RCLCPP_INFO(this->get_logger(), "RTCM message received.");
        }
        fe_interface_.write(msg->data.data(), msg->data.size());
      });

      fe_interface_.initialize(this, device_);
      listener_thread_ = std::thread(
        std::bind(&FusionEngineNode::dataListenerService, this));

    } else if (connection_type_ == "tcp" || connection_type_ == "udp") {
      RCLCPP_INFO(get_logger(), "IP: %s", ip_.c_str());
      RCLCPP_INFO(get_logger(), "Port: %d", port_);

      fe_interface_.initialize(this, ip_, port_);
      dataListenerService();
    } else {
      RCLCPP_ERROR(get_logger(), "Invalid connection type: %s", connection_type_.c_str());
      rclcpp::shutdown();
      return;
    }

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error initializing FusionEngineNode: %s", e.what());
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
void FusionEngineNode::handleFusionMessage(
  const MessageHeader & header,
  const void * payload)
{
  StatusResult status{Errors::noError};
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
          navigation_msgs::Pose msg{contents};
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
        navigation_msgs::GnssInfo msg{contents};
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::GNSS_SATELLITE:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::GNSSSatelliteMessage * > (payload);
        navigation_msgs::GnssSatellite msg{contents};
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
        navigation_msgs::PoseAux msg{contents};
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::CALIBRATION_STATUS:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::CalibrationStatusMessage * > (payload);
        navigation_msgs::CalibrationStatus msg{contents};
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::RELATIVE_ENU_POSITION:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::RelativeENUPositionMessage * > (payload);
        navigation_msgs::RelativeEnuPosition msg{contents};
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
        calibrated_msgs::Imu msg{contents};
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::GNSS_ATTITUDE_OUTPUT:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::GNSSAttitudeOutput * > (payload);
        calibrated_msgs::GnssAttitudeOutput msg{contents};
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::WHEEL_SPEED_OUTPUT:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::WheelSpeedOutput * > (payload);

        calibrated_msgs::WheelSpeedOutput msg{contents};
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::VEHICLE_SPEED_OUTPUT:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::VehicleSpeedOutput * > (payload);
        calibrated_msgs::VehicleSpeedOutput msg{contents};
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
        raw_msgs::RawImu msg{contents};
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::RAW_GNSS_ATTITUDE_OUTPUT:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::RawGNSSAttitudeOutput * > (payload);
        raw_msgs::RawGnssAttitudeOutput msg{contents};
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::RAW_WHEEL_TICK_OUTPUT:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::RawWheelTickOutput * > (payload);
        raw_msgs::RawWheelTickOutput msg{contents};
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::RAW_VEHICLE_TICK_OUTPUT:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::RawVehicleTickOutput * > (payload);
        raw_msgs::RawVehicleTickOutput msg{contents};
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::RAW_WHEEL_SPEED_OUTPUT:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::RawWheelSpeedOutput * > (payload);
        raw_msgs::RawWheelSpeedOutput msg{contents};
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::RAW_VEHICLE_SPEED_OUTPUT:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::RawVehicleSpeedOutput * > (payload);
        raw_msgs::RawVehicleSpeedOutput msg{contents};
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
        ros_msgs::Pose msg{contents};
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::ROS_GPS_FIX:
      {
        auto & contents = *reinterpret_cast < const GPSFixMessage * > (payload);
        ros_msgs::GPSFix msg{contents};
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::ROS_IMU:
      {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::ros::IMUMessage * > (payload);
        ros_msgs::Imu msg{contents};
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        kFactory().at(type)(this, &msg);
        break;
      }
    case MessageType::INPUT_DATA_WRAPPER: {
        auto & contents = *reinterpret_cast <
          const point_one::fusion_engine::messages::InputDataWrapperMessage * > (payload);

  if (contents.data_type == static_cast<uint16_t>(InputDataType::SBF_DATA)) {
    const uint8_t* inner_payload =
    reinterpret_cast<const uint8_t*>(&contents) + sizeof(InputDataWrapperMessage);
    size_t inner_size = header.payload_size_bytes - sizeof(InputDataWrapperMessage);

    if (isSBF(inner_payload, inner_size)) {
      uint16_t block_id = inner_payload[4] | (inner_payload[5] << 8);
      uint16_t block_num = block_id & 0x1FFF;

      if (block_num == static_cast<uint16_t>(SBFBlockID::PVTGeodetic)) {
        const auto* pvt = reinterpret_cast<const PVTGeodetic*>(inner_payload + 8); // skip 8-byte SBF header
        static auto pub = this->create_publisher<fusion_engine_msgs::msg::PVTGeodetic>(
          "pvt_geodetic", rclcpp::SensorDataQoS());
        sbf_msgs::PVTGeodetic msg{*pvt};
        msg.header.frame_id = frame_id_;
        msg.header.stamp = time;
        pub->publish(reinterpret_cast<const fusion_engine_msgs::msg::PVTGeodetic&>(msg));
      }
      // if(block_num == static_cast<uint16_t>(SBFBlockID::PVTCartesian) || 
      //    block_num == static_cast<uint16_t>(SBFBlockID::PVTCartesian_v2)) {
      //   const auto* pvt = reinterpret_cast<const PVTCartesian*>(inner_payload + 8); // skip 8-byte SBF header
      //    RCLCPP_INFO(get_logger(), "X: %.8f, Y: %.8f, Z: %.3f, Vx: %.3f, Vy: %.3f",
      //        pvt->x, pvt->y, pvt->z, pvt->vx, pvt->vy);
      // }

      if(Helper::to_string(block_num) == "UnknownSBFBlock"){
        RCLCPP_WARN(this->get_logger(),
          "Unknown SBF block detected: ID=0x%04X, length=%u, CRC=0x%04X",
          block_id);
      }
    //    RCLCPP_INFO(this->get_logger(),
    //  "SBF block detected: ID=0x%04X (%s, rev=%u), length=%u, CRC=0x%04X",
    //  block_id, Helper::to_string(block_num).c_str(), revision, length, crc);
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

