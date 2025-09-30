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

#pragma once

#include <string>
#include "data_listener.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial_port.hpp"

/**
 * @brief A listener class for reading data from a serial TTY port.
 */
class TtyListener : public DataListener
{
public:
  /**
   * @brief Construct a new Tty Listener object.
   *
   * @param node A pointer to the ROS2 node.
   * @param port The name of the serial port to listen on.
   */
  TtyListener(rclcpp::Node * node, const std::string & port);

  /**
   * @brief Destroy the Tty Listener object.
   */
  ~TtyListener() = default;

  /**
   * @brief Set the callback function that will be called when data is received.
   *
   * @param func The callback function to set.
   */
  void setCallback(const std::function<void(uint8_t *, size_t)> & func);

  /**
   * @brief Start listening for data on the serial port.
   */
  void listen();

  /**
   * @brief Write data to the serial port.
   *
   * @param data A pointer to the data to write.
   * @param size The size of the data to write.
   */
  void write(uint8_t * data, size_t size);

private:
  /**
   * @brief A pointer to the ROS2 node.
   */
  rclcpp::Node * node_;

  /**
   * @brief The name of the serial port to listen on.
   */
  std::string port_;

  /**
   * @brief The callback function that will be called when data is received.
   */
  std::function<void(uint8_t *, size_t)> callback_function_;

  /**
   * @brief The serial port used for reading and writing data.
   */
  SerialPort serial_port_;
};
