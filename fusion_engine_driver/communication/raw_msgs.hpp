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

namespace raw_msgs {
struct RawImu : public sensor_msgs::msg::Imu {
  ~RawImu() noexcept = default;
  inline explicit RawImu(const point_one::fusion_engine::messages::RawIMUOutput& p) noexcept {
    linear_acceleration.x = (p.accel[0] == INT32_MAX) ? NAN :
      static_cast<double>(p.accel[0]) / 65536.0;
    linear_acceleration.y = (p.accel[1] == INT32_MAX) ? NAN :
      static_cast<double>(p.accel[1]) / 65536.0;
    linear_acceleration.z = (p.accel[2] == INT32_MAX) ? NAN :
      static_cast<double>(p.accel[2]) / 65536.0;

    angular_velocity.x = (p.gyro[0] == INT32_MAX) ? NAN :
      static_cast<double>(p.gyro[0]) / 1048576.0;
    angular_velocity.y = (p.gyro[1] == INT32_MAX) ? NAN :
      static_cast<double>(p.gyro[1]) / 1048576.0;
    angular_velocity.z = (p.gyro[2] == INT32_MAX) ? NAN :
      static_cast<double>(p.gyro[2]) / 1048576.0;
  }
};

struct RawGnssAttitudeOutput : public fusion_engine_msgs::msg::RawGnssAttitudeOutput {
  ~RawGnssAttitudeOutput() noexcept = default;
  inline explicit RawGnssAttitudeOutput(const point_one::fusion_engine::messages::RawGNSSAttitudeOutput& p) noexcept {
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
  ~RawWheelSpeedOutput() noexcept = default;
  inline explicit RawWheelSpeedOutput(const point_one::fusion_engine::messages::RawWheelSpeedOutput& p) noexcept {
    gear = static_cast<int>(p.gear);

    fl = (p.front_left_speed  == INT32_MAX) ? NAN : p.front_left_speed  / 1024.0;
    fr = (p.front_right_speed == INT32_MAX) ? NAN : p.front_right_speed / 1024.0;
    rl = (p.rear_left_speed   == INT32_MAX) ? NAN : p.rear_left_speed   / 1024.0;
    rr = (p.rear_right_speed  == INT32_MAX) ? NAN : p.rear_right_speed  / 1024.0;
  }
};

struct RawVehicleSpeedOutput : public fusion_engine_msgs::msg::RawVehicleSpeedOutput {
  ~RawVehicleSpeedOutput() noexcept = default;
  inline explicit RawVehicleSpeedOutput(const point_one::fusion_engine::messages::RawVehicleSpeedOutput& p) noexcept {
    gear = static_cast<int>(p.gear);
    speed = p.vehicle_speed;
  }
};

struct RawWheelTickOutput : public fusion_engine_msgs::msg::RawWheelTickOutput {
  ~RawWheelTickOutput() noexcept = default;
  inline explicit RawWheelTickOutput(const point_one::fusion_engine::messages::RawWheelTickOutput& p) noexcept {
    gear = static_cast<int>(p.gear);
    fl = p.front_left_wheel_ticks;
    fr = p.front_right_wheel_ticks;
    rl = p.rear_left_wheel_ticks;
    rr = p.rear_right_wheel_ticks;
  }
};

struct RawVehicleTickOutput : public fusion_engine_msgs::msg::RawVehicleTickOutput {
  ~RawVehicleTickOutput() noexcept = default;
  inline explicit RawVehicleTickOutput(const point_one::fusion_engine::messages::RawVehicleTickOutput& p) noexcept {
    gear = static_cast<int>(p.gear);
    tick = p.tick_count;
  }
};

}  // namespace raw_msgs

#endif  // FUSION_ENGINE_DRIVER__COMMUNICATION__RAW_MSGS_HPP_
