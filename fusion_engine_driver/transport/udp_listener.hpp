#pragma once

#include <arpa/inet.h>   // For inet_addr()
#include <netinet/in.h>  // For sockaddr_in, htons()
#include <sys/socket.h>  // For socket support.
#include <unistd.h>      // For close()

#include <cstring>  // For strerror()
#include <string>

#include "data_listener.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief Listens for FusionEngine data over UDP.
 *
 * The device is configured to push datagrams to this host:port; we bind and
 * receive. Each recv() returns exactly one datagram, so unlike the TCP stream
 * there is no cross-message coalescing in the socket.
 */
class UdpListener : public DataListener
{
public:
  /**
   * @param node ROS2 node for logging.
   * @param ip   Expected source IP (device); empty binds any source.
   * @param port Local UDP port to bind (the port the device sends to).
   */
  UdpListener(rclcpp::Node * node, const std::string & ip, const int & port);
  ~UdpListener() = default;

  void setCallback(const std::function<void(uint8_t *, size_t)> & func);
  void listen();
  void write(uint8_t * data, size_t size);

private:
  int open();

  rclcpp::Node * node_;
  std::string _ip;
  int _port;
  int sock_ = 0;
  std::function<void(uint8_t *, size_t)> callback_function_;
};
