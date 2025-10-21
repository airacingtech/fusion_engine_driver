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

#ifndef FUSION_ENGINE_DRIVER__COMMUNICATION__CALIBRATED_MSGS_HPP_
#define FUSION_ENGINE_DRIVER__COMMUNICATION__CALIBRATED_MSGS_HPP_

#include <algorithm>
#include "sensor_msgs/msg/imu.hpp"
#include "fusion_engine_msgs/msg/gnss_attitude_output.hpp"
#include "fusion_engine_msgs/msg/wheel_speed_output.hpp"
#include "fusion_engine_msgs/msg/vehicle_speed_output.hpp"
#include "fusion_engine_msgs/msg/rpy.hpp"

namespace calibrated_msgs {
struct Imu : public sensor_msgs::msg::Imu {
  inline explicit Imu(const point_one::fusion_engine::messages::IMUOutput& p) {
    linear_acceleration.x = p.accel_mps2[0];
    linear_acceleration.y = p.accel_mps2[1];
    linear_acceleration.z = p.accel_mps2[2];

    linear_acceleration_covariance[0] = p.accel_std_mps2[0] * p.accel_std_mps2[0];
    linear_acceleration_covariance[4] = p.accel_std_mps2[1] * p.accel_std_mps2[1];
    linear_acceleration_covariance[8] = p.accel_std_mps2[2] * p.accel_std_mps2[2];

    angular_velocity.x = p.gyro_rps[0];
    angular_velocity.y = p.gyro_rps[1];
    angular_velocity.z = p.gyro_rps[2];

    angular_velocity_covariance[0] = p.gyro_std_rps[0] * p.gyro_std_rps[0];
    angular_velocity_covariance[4] = p.gyro_std_rps[1] * p.gyro_std_rps[1];
    angular_velocity_covariance[8] = p.gyro_std_rps[2] * p.gyro_std_rps[2];
  }
};

struct GnssAttitudeOutput : public fusion_engine_msgs::msg::GnssAttitudeOutput {
  inline explicit GnssAttitudeOutput(const point_one::fusion_engine::messages::GNSSAttitudeOutput& p) {
    solution_type = static_cast<int>(p.solution_type);

    rpy.roll  = p.ypr_deg[2];
    rpy.pitch = p.ypr_deg[1];
    rpy.yaw   = p.ypr_deg[0];

    rpy_covariance[0] = p.ypr_std_deg[2] * p.ypr_std_deg[2];
    rpy_covariance[4] = p.ypr_std_deg[1] * p.ypr_std_deg[1];
    rpy_covariance[8] = p.ypr_std_deg[0] * p.ypr_std_deg[0];

    baseline_distance = p.baseline_distance_m;
    baseline_distance_covariance = p.baseline_distance_std_m * p.baseline_distance_std_m;
  }
};

struct WheelSpeedOutput : public fusion_engine_msgs::msg::WheelSpeedOutput {
  inline explicit WheelSpeedOutput(const point_one::fusion_engine::messages::WheelSpeedOutput& p) {
    gear = static_cast<int>(p.gear);
    fl = p.front_left_speed_mps;
    fr = p.front_right_speed_mps;
    rl = p.rear_left_speed_mps;
    rr = p.rear_right_speed_mps;
  }
};

struct VehicleSpeedOutput : public fusion_engine_msgs::msg::VehicleSpeedOutput {
  inline explicit VehicleSpeedOutput(const point_one::fusion_engine::messages::VehicleSpeedOutput& p) {
    gear = static_cast<int>(p.gear);
    speed = p.vehicle_speed_mps;
  }
};

}  // namespace calibrated_msgs

#endif  // FUSION_ENGINE_DRIVER__COMMUNICATION__CALIBRATED_MSGS_HPP_
