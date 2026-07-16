#include "udp_listener.hpp"

/******************************************************************************/
UdpListener::UdpListener(
  rclcpp::Node * node, const std::string & ip,
  const int & port)
: node_(node), _ip(ip), _port(port) {}

/******************************************************************************/
void UdpListener::setCallback(
  const std::function < void(uint8_t *, size_t) > & func)
{
  callback_function_ = func;
}

/******************************************************************************/
void UdpListener::listen()
{
  uint8_t buffer[2048];

  if (open() != 0) {
    return;
  }
  running_ = true;
  try {
    while (running_) {
      ssize_t bytes_read = recv(sock_, buffer, sizeof(buffer), 0);
      if (bytes_read < 0) {
        RCLCPP_INFO(
          node_->get_logger(), "Error reading from UDP socket: %s (%d)",
          std::strerror(errno), errno);
        break;
      } else if (bytes_read == 0) {
        continue;
      }
      callback_function_(buffer, static_cast < size_t > (bytes_read));
    }
  } catch (std::exception const & ex) {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      "Decoder exception: " << ex.what());
  }
}

/******************************************************************************/
int UdpListener::open()
{
  sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock_ < 0) {
    RCLCPP_INFO(node_->get_logger(), "Error creating UDP socket");
    return 2;
  }

  sockaddr_in addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(_port);
  addr.sin_addr.s_addr = INADDR_ANY;  // device pushes to us; accept on any local iface

  if (bind(sock_, (sockaddr *)&addr, sizeof(addr)) < 0) {
    close(sock_);
    RCLCPP_INFO(
      node_->get_logger(),
      "Error binding UDP port %d: %s (%d)\n",
      _port, std::strerror(errno), errno);
    return 3;
  }

  RCLCPP_INFO(
    node_->get_logger(), "Listening for UDP on port %d (source '%s')",
    _port, _ip.c_str());
  return 0;
}

/******************************************************************************/
void UdpListener::write(uint8_t * data, size_t size)
{
  static_cast < void > (data);
  static_cast < void > (size);
}
