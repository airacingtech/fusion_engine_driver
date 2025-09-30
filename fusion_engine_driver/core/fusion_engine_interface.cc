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

#include "fusion_engine_interface.hpp"

/******************************************************************************/
FusionEngineInterface::FusionEngineInterface(
  std::function < void(const MessageHeader & header, const void * payload_in) >
  funcPublisher)
: framer(2048), publisher(funcPublisher)
{
  framer.SetMessageCallback(
    std::bind(
      &FusionEngineInterface::messageReceived,
      this, std::placeholders::_1,
      std::placeholders::_2));
}

/******************************************************************************/
void FusionEngineInterface::initialize(
  rclcpp::Node * node,
  const std::string & tcp_ip,
  int tcp_port)
{
  this->node_ = node;
  data_listener_ = std::make_shared < TcpListener > (node_, tcp_ip, tcp_port);
  data_listener_->setCallback(
    std::bind(
      &FusionEngineInterface::decodeFusionEngineMessage, this,
      std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(
    node_->get_logger(), "Initialize connection_type tcp in port %d",
    tcp_port);
}

/******************************************************************************/
void FusionEngineInterface::initialize(
  rclcpp::Node * node,
  const std::string & tty_port)
{
  this->node_ = node;
  data_listener_ = std::make_shared < TtyListener > (node_, tty_port);
  data_listener_->setCallback(
    std::bind(
      &FusionEngineInterface::decodeFusionEngineMessage, this,
      std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(
    node_->get_logger(), "Initialize connection_type tty on port %s",
    tty_port.c_str());
}

/******************************************************************************/
void FusionEngineInterface::messageReceived(
  const MessageHeader & header,
  const void * payload_in)
{
  auto payload = static_cast < const uint8_t * > (payload_in);
  dumpHex(header, payload, MessageType::GNSS_INFO);
  publisher(header, payload);
}

/******************************************************************************/
void FusionEngineInterface::dumpHex(
  const MessageHeader & header, const uint8_t * payload,
  MessageType type)
{
  if (header.message_type == type) {
    std::ostringstream oss;
    oss << to_string(header.message_type)
        << " Payload (" << header.payload_size_bytes << " bytes):\n";

    for (uint32_t i = 0; i < header.payload_size_bytes; i++) {
      oss << std::uppercase << std::setfill('0') << std::setw(2)
          << std::hex << static_cast<int>(payload[i]) << " ";
      if ((i + 1) % 8 == 0) {
        oss << "\n";  // 8 bytes per line
      }
    }
    oss << "\n";

    RCLCPP_DEBUG(node_->get_logger(), "%s", oss.str().c_str());
  }
}
/******************************************************************************/
void FusionEngineInterface::decodeFusionEngineMessage(
  uint8_t * frame,
  size_t bytes_read)
{
  framer.OnData(frame, bytes_read);
}

/******************************************************************************/
void FusionEngineInterface::dataListenerService()
{
  RCLCPP_INFO(node_->get_logger(), "Start listening using connection_type");
  data_listener_->listen();
}

/******************************************************************************/
void FusionEngineInterface::write(uint8_t * data, size_t size)
{
  data_listener_->write(data, size);
}

/******************************************************************************/
void FusionEngineInterface::stop() {data_listener_->stop();}
