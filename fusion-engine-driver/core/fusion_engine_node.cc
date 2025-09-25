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
      static auto pub = n->create_publisher<geometry_msgs::msg::PoseStamped>(
      "pose_filtered", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const geometry_msgs::msg::PoseStamped*>(msg));
    }},
    {MessageType::POSE_AUX, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<geometry_msgs::msg::PoseStamped>(
      "pose_aux", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const geometry_msgs::msg::PoseStamped*>(msg));
    }},
    {MessageType::CALIBRATION_STATUS, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<std_msgs::msg::String>(
      "calibration_status", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const std_msgs::msg::String*>(msg));
    }},
    {MessageType::GNSS_INFO, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<gps_msgs::msg::GPSStatus>(
      "gnss_info", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const gps_msgs::msg::GPSStatus*>(msg));
    }},
    {MessageType::GNSS_SATELLITE, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<gps_msgs::msg::GPSStatus>(
      "gnss_satellite", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const gps_msgs::msg::GPSStatus*>(msg));
    }},
    {MessageType::RELATIVE_ENU_POSITION, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<geometry_msgs::msg::PoseStamped>(
      "relative_enu_position", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const geometry_msgs::msg::PoseStamped*>(msg));
    }},

    // Calibrated Outputs
    {MessageType::IMU_OUTPUT, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<sensor_msgs::msg::Imu>(
      "imu_calibrated", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const sensor_msgs::msg::Imu*>(msg));
    }},
    {MessageType::GNSS_ATTITUDE_OUTPUT, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<geometry_msgs::msg::PoseStamped>(
      "gnss_attitude", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const geometry_msgs::msg::PoseStamped*>(msg));
    }},
    {MessageType::WHEEL_SPEED_OUTPUT, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<std_msgs::msg::String>(
      "wheel_speed", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const std_msgs::msg::String*>(msg));
    }},
    {MessageType::VEHICLE_SPEED_OUTPUT, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<std_msgs::msg::String>(
      "vehicle_speed", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const std_msgs::msg::String*>(msg));
    }},

    // Raw Outputs
    {MessageType::RAW_IMU_OUTPUT, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<sensor_msgs::msg::Imu>(
      "raw_imu", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const sensor_msgs::msg::Imu*>(msg));
    }},
    {MessageType::RAW_GNSS_ATTITUDE_OUTPUT, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<geometry_msgs::msg::PoseStamped>(
      "raw_gnss_attitude", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const geometry_msgs::msg::PoseStamped*>(msg));
    }},
    {MessageType::RAW_WHEEL_TICK_OUTPUT, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<std_msgs::msg::String>(
      "raw_wheel_tick", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const std_msgs::msg::String*>(msg));
    }},
    {MessageType::RAW_VEHICLE_TICK_OUTPUT, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<std_msgs::msg::String>(
      "raw_vehicle_tick", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const std_msgs::msg::String*>(msg));
    }},
    {MessageType::RAW_WHEEL_SPEED_OUTPUT, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<std_msgs::msg::String>(
      "raw_wheel_speed", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const std_msgs::msg::String*>(msg));
    }},
    {MessageType::RAW_VEHICLE_SPEED_OUTPUT, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<std_msgs::msg::String>(
      "raw_vehicle_speed", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const std_msgs::msg::String*>(msg));
    }},

    // ROS Outputs
    {MessageType::ROS_POSE, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<geometry_msgs::msg::PoseStamped>(
      "ros_pose", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const geometry_msgs::msg::PoseStamped*>(msg));
    }},
    {MessageType::ROS_GPS_FIX, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<gps_msgs::msg::GPSFix>(
      "ros_gpsfix", rclcpp::SensorDataQoS());
      pub->publish(*reinterpret_cast<const gps_msgs::msg::GPSFix*>(msg));
    }},
    {MessageType::ROS_IMU, [](FusionEngineNode* n, const void* msg) {
      static auto pub = n->create_publisher<sensor_msgs::msg::Imu>(
      "ros_imu", rclcpp::SensorDataQoS());
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
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::PoseMessage *>(payload);
      break;
      }
    case MessageType::GNSS_INFO:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::GNSSInfoMessage *>(payload);
        break;
      }
    case MessageType::GNSS_SATELLITE:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::GNSSSatelliteMessage *>(payload);
        break;
      }
    case MessageType::POSE_AUX:
      {
        break;
      }
    case MessageType::CALIBRATION_STATUS:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::CalibrationStatusMessage *>(payload);
        break;
      }
    case MessageType::RELATIVE_ENU_POSITION:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::RelativeENUPositionMessage *>(payload);
        break;
      }
    // Calibrated Sensors
    case MessageType::IMU_OUTPUT:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::IMUOutput *>(payload);
        break;
      }
    case MessageType::GNSS_ATTITUDE_OUTPUT:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::GNSSAttitudeOutput *>(payload);
        break;
      }
    case MessageType::WHEEL_SPEED_OUTPUT:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::WheelSpeedOutput *>(payload);
        break;
      }
    case MessageType::VEHICLE_SPEED_OUTPUT:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::VehicleSpeedOutput *>(payload);
        break;
      }
    // Raw Sensors
    case MessageType::RAW_IMU_OUTPUT:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::RawIMUOutput *>(payload);
        break;
      }
    case MessageType::RAW_GNSS_ATTITUDE_OUTPUT:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::RawGNSSAttitudeOutput *>(payload);
        break;
      }
    case MessageType::RAW_WHEEL_TICK_OUTPUT:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::RawWheelTickOutput *>(payload);
        break;
      }
    case MessageType::RAW_VEHICLE_TICK_OUTPUT:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::RawVehicleTickOutput *>(payload);
        break;
      }
    case MessageType::RAW_WHEEL_SPEED_OUTPUT:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::RawWheelSpeedOutput *>(payload);
        break;
      }
    case MessageType::RAW_VEHICLE_SPEED_OUTPUT:
      {
        auto &contents = *reinterpret_cast<const point_one::fusion_engine::messages::RawVehicleSpeedOutput *>(payload);
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
      /*
  if (header.message_type == MessageType::ROS_GPS_FIX) {
    auto &contents = *reinterpret_cast<const GPSFixMessage *>(payload);
    gps_msgs::msg::GPSFix gps_fix = ConversionUtils::toGPSFix(contents);
    gps_fix.header.frame_id = frame_id_;
    gps_fix.header.stamp = time;
    gps_fix.status.satellites_visible = satellite_nb_;
    gps_fix_publisher_->publish(gps_fix);
    publishNavFixMsg(gps_fix);
  } else if (header.message_type == MessageType::IMU_OUTPUT) {
    auto &contents = *reinterpret_cast<
	const point_one::fusion_engine::messages::IMUOutput*>(payload);
    sensor_msgs::msg::Imu imu = ConversionUtils::toImu(contents);
    imu.header.frame_id = frame_id_;
    imu.header.stamp = time;

  /*  std::copy(std::begin(ros_imu_.orientation_covariance),
              std::end(ros_imu_.orientation_covariance),
              std::begin(imu.orientation_covariance));

    std::copy(std::begin(ros_imu_.angular_velocity_covariance),
              std::end(ros_imu_.angular_velocity_covariance),
              std::begin(imu.angular_velocity_covariance));

    std::copy(std::begin(ros_imu_.acceleration_covariance),
              std::end(ros_imu_.acceleration_covariance),
              std::begin(imu.linear_acceleration_covariance)); 
	      
    imu_publisher_->publish(imu);
  } else if (header.message_type == MessageType::ROS_IMU)  {
//    ros_imu_ = *reinterpret_cast<
//	    const point_one::fusion_engine::messages::ros::IMUMessage *>(payload);
  } else if (header.message_type == MessageType::ROS_POSE) {	  
    auto &contents = *reinterpret_cast<
        const point_one::fusion_engine::messages::ros::PoseMessage *>(payload);
    geometry_msgs::msg::PoseStamped pos = ConversionUtils::toPose(contents);
    visualization_msgs::msg::Marker points;
    pos.header.frame_id = frame_id_;
    pos.header.stamp = time;
    pose_publisher_->publish(pos);
    points.header.frame_id = frame_id_;
    points.header.stamp = this->now();
    points.ns = "basic_shapes";
    points.action = visualization_msgs::msg::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = id;
    points.type = visualization_msgs::msg::Marker::POINTS;
    points.scale.x = 0.1;
    points.scale.y = 0.1;
    points.color.g = 1.0f;
    points.color.a = 1.0;
    geometry_msgs::msg::Point p;
    p.x = pos.pose.position.x;
    p.y = pos.pose.position.y;
    p.z = pos.pose.position.z;

    if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z)) {
      if (this->get_parameter("debug").as_bool())
        RCLCPP_INFO(this->get_logger(), "Point published = [LLA=%f, %f, %f]",
                    p.x, p.y, p.z);
      points.points.push_back(p);
      publisher_->publish(points);
      id++;

    } else {
      if (this->get_parameter("debug").as_bool())
        RCLCPP_INFO(this->get_logger(), "Point dropped = [LLA=%f, %f, %f]", p.x,
                    p.y, p.z);
    }
  } else if (header.message_type == MessageType::POSE &&
             this->get_parameter("connection_type").as_string() == "tty") {
    auto &contents = *reinterpret_cast<
        const point_one::fusion_engine::messages::PoseMessage *>(payload);
    double gps_time_sec =
        contents.gps_time.seconds + contents.gps_time.fraction_ns * 1e-9;
    if (gps_time_sec - previous_gps_time_sec_ >
        TIME_BETWEEN_NMEA_UPDATES_SEC_) {
      nmea_msgs::msg::Sentence nmea =
          ConversionUtils::toNMEA(contents, satellite_nb_);
      nmea.header.stamp = this->now();
      nmea.header.frame_id = "gps";
      previous_gps_time_sec_ = gps_time_sec;
      nmea_publisher_->publish(nmea);
    }
  }
  }
  */
}

/******************************************************************************/
/*
	switch(type){
		// Navigation
		case MessageType::POSE:
			filtered_pose_pub_ = filtered_pose_pub_ ?
					    filtered_pose_pub_ :
					    this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_filtered", rclcpp::SensorDataQoS());
			break;
		case MessageType::GNSS_INFO:
			gnss_info_pub_ = gnss_info_pub_ ?
					 gnss_info_pub_ :
					 this->create_publisher<>("gnss_info", rclcpp:SensorDataQoS());
			break;
		case MessageType::GNSS_SATELLITE:
			gnss_satellite_pub_ = gnss_satellite_pub_ ?
					      gnss_satellite_pub_ :
					      this->create_publisher<>("gnss_satellite", rclcpp:SensorDataQoS());
			break;
		case MessageType::POSE_AUX:
			pose_aux_pub_ = pose_aux_pub_ ?
					pose_aux_pub_ :
					this->create_publisher<>("pose_aux", rclcppSensorDataQoS());
			break;
		case MessageType::CALIBRATION_STATUS:
			calibration_pub_ = calibration_pub_ ?
					   calibration_pub_ :
					   this->create_publisher<>("calibration_status", rclcppSensorDataQoS());
			break;
		case MessageType::RELATIVE_ENU_POSITION:
			relative_enu_pub_ = relative_enu_pub_ ?
					    relative_enu_pub_ :
					    this->create_publisher<>("relative_enu_position", rclcppSensorDataQoS());
			break;
		// Calibrated
		case MessageType::IMU_OUTPUT:
		case MessageType::GNSS_ATTITUDE_OUTPUT:
		case MessageType::WHEEL_SPEED_OUTPUT:
		case MessageType::VEHICLE_SPEED_OUTPUT:
		
		// Raw
		case MessageType::RAW_IMU_OUTPUT:
		case MessageType::RAW_GNSS_ATTITUDE_OUTPUT:
		case MessageType::RAW_WHEEL_TICK_OUTPUT:
		case MessageType::RAW_VEHICLE_TICK_OUTPUT:
		case MessageType::RAW_WHEEL_SPEED_OUTPUT:
		case MessageType::RAW_VEHICLE_SPEED_OUTPUT:

		// ROS
		case MessageType::ROS_POSE:
		case MessageType::ROS_GPS_FIX:
		case MessageType::ROS_IMU:
		default:
			break;
	}
}

*/
/*****************************************************************************/
void FusionEngineNode::publishNavFixMsg(const gps_msgs::msg::GPSFix &gps_fix) {
  /*
  sensor_msgs::msg::NavSatFix fix;
  fix.header = gps_fix.header;
  fix.status.status = gps_fix.status.status;
  fix.status.service = 0;
  if (gps_fix.status.position_source & gps_msgs::msg::GPSStatus::SOURCE_GPS) {
    fix.status.service =
        fix.status.service | sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  }
  if (gps_fix.status.orientation_source &
      gps_msgs::msg::GPSStatus::SOURCE_MAGNETIC) {
    fix.status.service =
        fix.status.service | sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
  }
  fix.latitude = gps_fix.latitude;
  fix.longitude = gps_fix.longitude;
  fix.altitude = gps_fix.altitude;
  fix.position_covariance = gps_fix.position_covariance;
  fix.position_covariance_type = gps_fix.position_covariance_type;
  nav_fix_publisher_->publish(fix);
  */
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
