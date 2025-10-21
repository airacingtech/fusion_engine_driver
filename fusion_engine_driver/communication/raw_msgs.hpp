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

#ifndef FUSION_ENGINE_DRIVER__COMMUNICATION__RAW_MSGS_HPP_
#define FUSION_ENGINE_DRIVER__COMMUNICATION__RAW_MSGS_HPP_

#include <cmath>
#include <cstdint>
#include <algorithm>

#include "sensor_msgs/msg/imu.hpp"
#include "fusion_engine_msgs/msg/raw_gnss_attitude_output.hpp"
#include "fusion_engine_msgs/msg/raw_wheel_speed_output.hpp"
#include "fusion_engine_msgs/msg/raw_vehicle_speed_output.hpp"
#include "fusion_engine_msgs/msg/raw_wheel_tick_output.hpp"
#include "fusion_engine_msgs/msg/raw_vehicle_tick_output.hpp"

/* ========================================================================== */
/*  RAW MESSAGE STRUCTS                                                      */
/*  Inline zero-cost wrappers converting PointOne → ROS 2                    */
/* ========================================================================== */

namespace raw_msgs {

struct RawImu : public sensor_msgs::msg::Imu {
  inline explicit RawImu(const point_one::fusion_engine::messages::RawIMUOutput& p) {
    linear_acceleration.x = (p.accel[0] == INT32_MAX) ? NAN :
      static_cast<double>(p.accel[0]) / 65536.0;
    linear_acceleration.y = (p.accel[1] == INT32_MAX) ? NAN :
      static_cast<double>(p.accel[1]) / 65536.0;
    linear_acceleration.z = (p.accel[2] == INT32_MAX) ? NAN :
      static_cast<double>(p.accel[2]) / 65536.0;

    angular_velocity.x = (p.gyro[0] == INT32_MAX) ? NAN :
      static_cast<double>(p.gyro[0]) / 65536.0;
    angular_velocity.y = (p.gyro[1] == INT32_MAX) ? NAN :
      static_cast<double>(p.gyro[1]) / 65536.0;
    angular_velocity.z = (p.gyro[2] == INT32_MAX) ? NAN :
      static_cast<double>(p.gyro[2]) / 65536.0;
  }
};

struct RawGnssAttitudeOutput : public fusion_engine_msgs::msg::RawGnssAttitudeOutput {
  inline explicit RawGnssAttitudeOutput(const point_one::fusion_engine::messages::RawGNSSAttitudeOutput& p) {
    solution_type = static_cast<int>(p.solution_type);

    baseline.x = p.relative_position_enu_m[0];
    baseline.y = p.relative_position_enu_m[1];
    baseline.z = p.relative_position_enu_m[2];

    baseline_covariance.x = p.position_std_enu_m[0];
    baseline_covariance.y = p.position_std_enu_m[1];
    baseline_covariance.z = p.position_std_enu_m[2];
  }
};

struct RawWheelSpeedOutput : public fusion_engine_msgs::msg::RawWheelSpeedOutput {
  inline explicit RawWheelSpeedOutput(const point_one::fusion_engine::messages::RawWheelSpeedOutput& p) {
    gear = static_cast<int>(p.gear);

    fl = (p.front_left_speed  == INT32_MAX) ? NAN : p.front_left_speed  / 1024.0;
    fr = (p.front_right_speed == INT32_MAX) ? NAN : p.front_right_speed / 1024.0;
    rl = (p.rear_left_speed   == INT32_MAX) ? NAN : p.rear_left_speed   / 1024.0;
    rr = (p.rear_right_speed  == INT32_MAX) ? NAN : p.rear_right_speed  / 1024.0;
  }
};

struct RawVehicleSpeedOutput : public fusion_engine_msgs::msg::RawVehicleSpeedOutput {
  inline explicit RawVehicleSpeedOutput(const point_one::fusion_engine::messages::RawVehicleSpeedOutput& p) {
    gear = static_cast<int>(p.gear);
    speed = p.vehicle_speed;
  }
};

struct RawWheelTickOutput : public fusion_engine_msgs::msg::RawWheelTickOutput {
  inline explicit RawWheelTickOutput(const point_one::fusion_engine::messages::RawWheelTickOutput& p) {
    gear = static_cast<int>(p.gear);
    fl = p.front_left_wheel_ticks;
    fr = p.front_right_wheel_ticks;
    rl = p.rear_left_wheel_ticks;
    rr = p.rear_right_wheel_ticks;
  }
};

struct RawVehicleTickOutput : public fusion_engine_msgs::msg::RawVehicleTickOutput {
  inline explicit RawVehicleTickOutput(const point_one::fusion_engine::messages::RawVehicleTickOutput& p) {
    gear = static_cast<int>(p.gear);
    tick = p.tick_count;
  }
};

}  // namespace raw_msgs

#endif  // FUSION_ENGINE_DRIVER__COMMUNICATION__RAW_MSGS_HPP_
