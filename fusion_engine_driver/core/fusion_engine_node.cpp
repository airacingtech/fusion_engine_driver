#include "fusion_engine_node.hpp"
#include <limits>

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
        dataListenerService();
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

  double device_s;
  if (extractP1TimeSeconds(header, payload, device_s)) {
    const double host_s = clock_sync_->update(device_s, arrival.seconds());
    // corr is 0 while disabled/warming up (host_s == arrival); otherwise the
    // small subtraction preserves arrival's nanosecond precision.
    const double corr_s = arrival.seconds() - host_s;
    stamp = arrival - rclcpp::Duration::from_seconds(corr_s);
  }

  findHandler(header)(this, payload, frame_id_, stamp);
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

