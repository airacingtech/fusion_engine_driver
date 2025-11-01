#pragma once

#include <pcap/pcap.h>

#include <csignal>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>

#include "data_listener.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief A class for reading data from PCAP files.
 * 
 * This listener reads UDP packets from a PCAP file and processes them
 * as if they were received from the network in real-time.
 */
class PcapListener : public DataListener
{
public:
  /**
   * @brief Constructs a PcapListener object.
   *
   * @param node A pointer to the ROS2 node to use for logging.
   * @param pcap_file Path to the PCAP file to read.
   * @param filter_ip IP address to filter packets (empty string for no filtering).
   */
  PcapListener(
    rclcpp::Node * node,
    const std::string & pcap_file,
    const std::string & filter_ip);

  /**
   * @brief Destroys the PcapListener object.
   */
  ~PcapListener();

  /**
   * @brief Sets a callback function to be executed when data is received.
   *
   * @param func The callback function to set.
   */
  void setCallback(const std::function<void(uint8_t *, size_t)> & func);

  /**
   * @brief Reads and processes packets from the PCAP file.
   */
  void listen();

  /**
   * @brief Writes data (not supported for PCAP playback).
   *
   * @param data A pointer to the data to write.
   * @param size The size of the data to write.
   */
  void write(uint8_t * data, size_t size);

private:
  /**
   * @brief Opens the PCAP file and sets up the filter.
   * 
   * @return 0 on success, non-zero on error.
   */
  int open();

  /**
   * @brief Extracts UDP payload from a packet.
   * 
   * @param packet Pointer to the packet data.
   * @param header PCAP packet header.
   * @param payload_out Output pointer to the UDP payload.
   * @param payload_len_out Output length of the UDP payload.
   * @return true if UDP payload was successfully extracted.
   */
  bool extractUdpPayload(
    const u_char * packet,
    const struct pcap_pkthdr * header,
    const u_char ** payload_out,
    size_t * payload_len_out);

  /**
   * @brief Extracts TCP payload from a packet.
   * 
   * @param packet Pointer to the packet data.
   * @param header PCAP packet header.
   * @param payload_out Output pointer to the TCP payload.
   * @param payload_len_out Output length of the TCP payload.
   * @return true if TCP payload was successfully extracted.
   */
  bool extractTcpPayload(
    const u_char * packet,
    const struct pcap_pkthdr * header,
    const u_char ** payload_out,
    size_t * payload_len_out);

  /**
   * @brief A pointer to the ROS2 node for logging.
   */
  rclcpp::Node * node_;

  /**
   * @brief Path to the PCAP file.
   */
  std::string pcap_file_;

  /**
   * @brief IP address to filter packets (empty for no filtering).
   */
  std::string filter_ip_;

  /**
   * @brief PCAP file handle.
   */
  pcap_t * pcap_handle_;

  /**
   * @brief The callback function to execute on incoming data.
   */
  std::function<void(uint8_t *, size_t)> callback_function_;

  /**
   * @brief Timestamp from the last PCAP packet.
   */
  double last_pcap_timestamp_;
};
