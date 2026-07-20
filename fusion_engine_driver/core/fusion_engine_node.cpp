#include "fusion_engine_node.hpp"
#include <limits>
#include <cmath>
#include <new>

#include <point_one/fusion_engine/messages/measurements.h>
#include <point_one/fusion_engine/messages/crc.h>

namespace
{
// WheelSpeedInput encodes speed as m/s * 2^-10 (i.e. m/s * 1024). INT32_MAX is
// the "not available" sentinel.
int32_t toWheelSpeedCounts(float speed_mps)
{
  if (!std::isfinite(speed_mps)) {
    return INT32_MAX;
  }
  return static_cast<int32_t>(std::lround(speed_mps * 1024.0f));
}
}  // namespace

/******************************************************************************/
FusionEngineNode::FusionEngineNode(const rclcpp::NodeOptions & options)
: Node("fusion_engine_node", options),
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
  
  // PCAP parameters
  bool enable_pcap = declare_parameter("enable_pcap", false);
  std::string pcap_file = declare_parameter("pcap_file", "");

  // Clock sync: map device (p1) time -> host clock so bursty transport delivery
  // stops contaminating measurement timestamps. On by default; disable on
  // systems already disciplined in the background (PTP / GPS-PPS / chrony).
  art::ClockSync::Config clock_cfg;
  clock_cfg.enabled = declare_parameter("enable_clock_sync", true);
  clock_cfg.window_sec = declare_parameter("clock_sync.window_sec", 2.0);
  clock_cfg.min_samples = static_cast<std::size_t>(
    declare_parameter("clock_sync.min_samples", 50));
  clock_sync_ = std::make_unique<art::ClockSync>(clock_cfg);
  RCLCPP_INFO(get_logger(), "Clock sync %s",
    clock_cfg.enabled ? "ENABLED (device-time stamping)" : "disabled (receipt time)");

  // Wheel-speed injection: forward vehicle wheel speeds to the device as
  // FusionEngine WheelSpeedInput messages over the active connection. Off by
  // default since it feeds the device's nav solution; enable per vehicle.
  enable_wheel_speed_input_ = declare_parameter("enable_wheel_speed_input", false);
  const std::string wheel_speed_topic = declare_parameter(
    "wheel_speed_input_topic", std::string("/vehicle/wheel_speed_report"));
  if (enable_wheel_speed_input_) {
    wheel_speed_sub_ = create_subscription<race_msgs::msg::WheelSpeedReport>(
      wheel_speed_topic, rclcpp::SensorDataQoS(),
      std::bind(&FusionEngineNode::onWheelSpeedReport, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(),
      "Wheel speed input ENABLED: %s -> FusionEngine WheelSpeedInput",
      wheel_speed_topic.c_str());
  }

  timer_ = create_wall_timer(
    std::chrono::milliseconds(1),
    std::bind(&FusionEngineNode::rosServiceLoop, this));

  try {
    RCLCPP_INFO(get_logger(), "Initializing FusionEngineNode...");

    // Check if PCAP mode is enabled
    if (enable_pcap) {
      if (pcap_file.empty()) {
        RCLCPP_ERROR(get_logger(), "enable_pcap is true but pcap_file is not specified");
        rclcpp::shutdown();
        return;
      }
      
      RCLCPP_INFO(get_logger(), "PCAP mode enabled: %s", pcap_file.c_str());
      
      fe_interface_.initialize(this, pcap_file, ip_);
      dataListenerService();
      
    } else {
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

        fe_interface_.initialize(this, ip_, port_, connection_type_);
        // Run the blocking listener on its own thread so the node keeps
        // spinning (timers, and the wheel-speed input subscription callback).
        listener_thread_ = std::thread(
          std::bind(&FusionEngineNode::dataListenerService, this));
      } else {
        RCLCPP_ERROR(get_logger(), "Invalid connection type: %s", connection_type_.c_str());
        rclcpp::shutdown();
        return;
      }
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
  const rclcpp::Time arrival = this->now();
  rclcpp::Time stamp = arrival;

  // POSE carries the same instant on both device clocks; their difference is an
  // exact p1 -> GPS offset with no transport latency in it. The device holds it
  // through short GNSS outages (gps_time goes INVALID, last offset stays valid;
  // p1 drift vs GPS is negligible over an outage).
  if (header.message_type == MessageType::POSE) {
    // Qualified: `using namespace ...messages::ros` also exposes a PoseMessage.
    const auto& pose =
      *reinterpret_cast<const point_one::fusion_engine::messages::PoseMessage*>(payload);
    if (pose.p1_time.seconds != Timestamp::INVALID &&
        pose.gps_time.seconds != Timestamp::INVALID) {
      p1_to_gps_ns_ = timestampNs(pose.gps_time) - timestampNs(pose.p1_time);
      have_p1_to_gps_ = true;
    }
  }

  int64_t p1_ns;
  if (extractP1TimeNs(header, payload, p1_ns)) {
    // Backward p1 jump => device reset: the p1 epoch restarted, so the learned
    // offset is garbage until the next valid POSE refreshes it.
    if (have_last_p1_ && p1_ns < last_p1_ns_ - art::kNsPerSec) {
      have_p1_to_gps_ = false;
    }
    last_p1_ns_ = p1_ns;
    have_last_p1_ = true;

    if (have_p1_to_gps_) {
      stamp = rclcpp::Time(art::gps_ns_to_unix_ns(p1_ns + p1_to_gps_ns_), RCL_ROS_TIME);
    } else {
      // GPS-denied fallback: envelope-estimated p1 -> host mapping. corr is 0
      // while disabled/warming up (host_s == arrival); the small subtraction
      // preserves arrival's nanosecond precision.
      const double host_s = clock_sync_->update(p1_ns * 1e-9, arrival.seconds());
      const double corr_s = arrival.seconds() - host_s;
      stamp = arrival - rclcpp::Duration::from_seconds(corr_s);
    }
  }

  findHandler(header)(this, payload, frame_id_, stamp);
}

/******************************************************************************/
void FusionEngineNode::onWheelSpeedReport(
  const race_msgs::msg::WheelSpeedReport::SharedPtr msg)
{
  using point_one::fusion_engine::messages::CalculateCRC;
  using point_one::fusion_engine::messages::GearType;
  using point_one::fusion_engine::messages::MessageType;
  using point_one::fusion_engine::messages::SensorDataSource;
  using point_one::fusion_engine::messages::WheelSpeedInput;

  alignas(4) uint8_t buffer[sizeof(MessageHeader) + sizeof(WheelSpeedInput)];
  auto * header = new (buffer) MessageHeader();
  auto * payload = new (buffer + sizeof(MessageHeader)) WheelSpeedInput();

  header->message_type = MessageType::WHEEL_SPEED_INPUT;
  header->message_version = WheelSpeedInput::MESSAGE_VERSION;
  header->payload_size_bytes = sizeof(WheelSpeedInput);
  header->sequence_number = wheel_input_seq_++;

  // Wheel odometry originates from the vehicle CAN bus. measurement_time is left
  // invalid so the device timestamps the sample on arrival.
  payload->details.data_source = SensorDataSource::CAN;
  payload->front_left_speed = toWheelSpeedCounts(msg->front_left);
  payload->front_right_speed = toWheelSpeedCounts(msg->front_right);
  payload->rear_left_speed = toWheelSpeedCounts(msg->rear_left);
  payload->rear_right_speed = toWheelSpeedCounts(msg->rear_right);
  payload->gear = GearType::UNKNOWN;

  header->crc = CalculateCRC(buffer);
  fe_interface_.write(buffer, sizeof(buffer));
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

