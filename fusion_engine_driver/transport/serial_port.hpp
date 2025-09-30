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

#include <errno.h>    // Error integer and strerror() function
#include <fcntl.h>    // Contains file controls like O_RDWR
#include <termios.h>  // Contains POSIX terminal control definitions
#include <unistd.h>   // write(), read(), close()

#include <cstring>
#include <iostream>
#include <string>

/**
 * @brief A class representing a serial port for reading and writing data over
 * serial.
 */
class SerialPort
{
public:
  /**
   * @brief Default constructor for SerialPort.
   */
  SerialPort() = default;

  /**
   * @brief Opens the serial port with the specified device path and baud rate.
   *
   * @param device_path The path to the serial device.
   * @param baud_rate The baud rate to use.
   * @return True if the serial port was successfully opened, false otherwise.
   */
  bool Open(const char * device_path, uint32_t baud_rate);

  /**
   * @brief Reads data from the serial port.
   *
   * @param read_buffer A pointer to the buffer to store the read data.
   * @param read_size The number of bytes to read.
   * @return The number of bytes read.
   */
  int Read(void * read_buffer, size_t read_size);

  /**
   * @brief Writes data to the serial port.
   *
   * @param write_buffer A pointer to the buffer containing the data to write.
   * @param write_size The number of bytes to write.
   * @return The number of bytes written.
   */
  int Write(const void * write_buffer, size_t write_size);

private:
  using SerialFile = int;

  /**
   * @brief The file descriptor for the serial port.
   */
  SerialFile serial_port_;
};
