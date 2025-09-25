#pragma once

#include <point_one/fusion_engine/messages/ros.h>

#include <chrono>
#include <cmath>
#include <ctime>
#include <numeric>
#include <sstream>
#include <map>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "mavros_msgs/msg/rtcm.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "fusion_engine_msgs/msg/pose.hpp"
#include "fusion_engine_msgs/msg/pose_aux.hpp"
#include "fusion_engine_msgs/msg/calibration_status.hpp"
#include "fusion_engine_msgs/msg/relative_enu_position.hpp"
#include "fusion_engine_msgs/msg/gnss_info.hpp"
#include "fusion_engine_msgs/msg/gnss_satellite.hpp"
#include "fusion_engine_msgs/msg/gnss_satellite_info.hpp"
#include "fusion_engine_msgs/msg/imu_output.hpp"
#include "fusion_engine_msgs/msg/gnss_attitude_output.hpp"
#include "fusion_engine_msgs/msg/wheel_speed_output.hpp"
#include "fusion_engine_msgs/msg/vehicle_speed_output.hpp"
#include "fusion_engine_msgs/msg/raw_gnss_attitude_output.hpp"
#include "fusion_engine_msgs/msg/raw_imu_output.hpp"
#include "fusion_engine_msgs/msg/raw_vehicle_speed_output.hpp"
#include "fusion_engine_msgs/msg/raw_vehicle_tick_output.hpp"
#include "fusion_engine_msgs/msg/raw_wheel_speed_output.hpp"
#include "fusion_engine_msgs/msg/raw_wheel_tick_output.hpp"


class ConversionUtils {
  public:
  
  /***********************************************************************************************************/
  /* Filtered / Navigation Utils */
static fusion_engine_msgs::msg::Pose populate(const point_one::fusion_engine::messages::PoseMessage& contents) {
  fusion_engine_msgs::msg::Pose msg;

  msg.p1_time.seconds     = contents.p1_time.seconds;
  msg.p1_time.fraction_ns = contents.p1_time.fraction_ns;
  msg.gps_time.seconds    = contents.gps_time.seconds;
  msg.gps_time.fraction_ns= contents.gps_time.fraction_ns;

  msg.solution_type = static_cast<uint8_t>(contents.solution_type);
  msg.flags         = contents.flags;
  msg.undulation_cm = contents.undulation_cm;

  msg.latitude_deg  = contents.lla_deg[0];
  msg.longitude_deg = contents.lla_deg[1];
  msg.altitude_m    = contents.lla_deg[2];

  msg.position_std_enu_m[0] = contents.position_std_enu_m[0];
  msg.position_std_enu_m[1] = contents.position_std_enu_m[1];
  msg.position_std_enu_m[2] = contents.position_std_enu_m[2];

  msg.ypr_deg[0] = contents.ypr_deg[0];
  msg.ypr_deg[1] = contents.ypr_deg[1];
  msg.ypr_deg[2] = contents.ypr_deg[2];

  msg.ypr_std_deg[0] = contents.ypr_std_deg[0];
  msg.ypr_std_deg[1] = contents.ypr_std_deg[1];
  msg.ypr_std_deg[2] = contents.ypr_std_deg[2];

  msg.velocity_flu_mps[0] = contents.velocity_body_mps[0];
  msg.velocity_flu_mps[1] = contents.velocity_body_mps[1];
  msg.velocity_flu_mps[2] = contents.velocity_body_mps[2];

  msg.velocity_std_flu_mps[0] = contents.velocity_std_body_mps[0];
  msg.velocity_std_flu_mps[1] = contents.velocity_std_body_mps[1];
  msg.velocity_std_flu_mps[2] = contents.velocity_std_body_mps[2];

  msg.aggregate_protection_level_m  = contents.aggregate_protection_level_m;
  msg.horizontal_protection_level_m = contents.horizontal_protection_level_m;
  msg.vertical_protection_level_m   = contents.vertical_protection_level_m;

  return msg;
}

static fusion_engine_msgs::msg::PoseAux populate(const point_one::fusion_engine::messages::PoseAuxMessage& contents) {
  fusion_engine_msgs::msg::PoseAux msg;

  msg.p1_time.seconds      = contents.p1_time.seconds;
  msg.p1_time.fraction_ns  = contents.p1_time.fraction_ns;

  msg.position_std_body_m[0] = contents.position_std_body_m[0];
  msg.position_std_body_m[1] = contents.position_std_body_m[1];
  msg.position_std_body_m[2] = contents.position_std_body_m[2];

  std::copy(std::begin(contents.position_std_body_m),
            std::end(contents.position_std_body_m),
            std::begin(msg.position_std_body_m));

  msg.attitude_quaternion[0] = contents.attitude_quaternion[0];
  msg.attitude_quaternion[1] = contents.attitude_quaternion[1];
  msg.attitude_quaternion[2] = contents.attitude_quaternion[2];
  msg.attitude_quaternion[3] = contents.attitude_quaternion[3];

  msg.velocity_enu_mps[0] = contents.velocity_enu_mps[0];
  msg.velocity_enu_mps[1] = contents.velocity_enu_mps[1];
  msg.velocity_enu_mps[2] = contents.velocity_enu_mps[2];

  msg.velocity_std_enu_mps[0] = contents.velocity_std_enu_mps[0];
  msg.velocity_std_enu_mps[1] = contents.velocity_std_enu_mps[1];
  msg.velocity_std_enu_mps[2] = contents.velocity_std_enu_mps[2];

  return msg;
}

// GNSS Info
static fusion_engine_msgs::msg::GnssInfo populate(const point_one::fusion_engine::messages::GNSSInfoMessage& contents) {
  fusion_engine_msgs::msg::GnssInfo msg;

  msg.p1_time.seconds      = contents.p1_time.seconds;
  msg.p1_time.fraction_ns  = contents.p1_time.fraction_ns;
  msg.gps_time.seconds     = contents.gps_time.seconds;
  msg.gps_time.fraction_ns = contents.gps_time.fraction_ns;

  msg.leap_second     = contents.leap_second;
  msg.num_satellites        = contents.num_svs;
  msg.corrections_age_decisec = contents.corrections_age * 0.1;
  msg.baseline_distance_m = contents.baseline_distance * 10;
  msg.reference_station_id = contents.reference_station_id;

  msg.gdop = contents.gdop;
  msg.pdop = contents.pdop;
  msg.hdop = contents.hdop;
  msg.vdop = contents.vdop;

  msg.gps_time_std_dev_s = contents.gps_time_std_sec;

  return msg;
}

// GNSS Satellite Info
static fusion_engine_msgs::msg::GnssSatelliteInfo populate(const point_one::fusion_engine::messages::SatelliteInfo& sat) {
  fusion_engine_msgs::msg::GnssSatelliteInfo msg;
  msg.prn           = sat.prn;
  msg.constellation = static_cast<uint8_t>(sat.system);
  msg.cn0_db_hz     = sat.cn0;
  msg.elevation_deg = sat.elevation_deg;
  msg.azimuth_deg   = sat.azimuth_deg;
  msg.usage         = sat.usage;
  return msg;
}

static fusion_engine_msgs::msg::GnssSatellite populate(const point_one::fusion_engine::messages::GNSSSatelliteMessage& contents) {
  fusion_engine_msgs::msg::GnssSatellite msg;

  msg.p1_time.seconds      = contents.p1_time.seconds;
  msg.p1_time.fraction_ns  = contents.p1_time.fraction_ns;
  msg.gps_time.seconds     = contents.gps_time.seconds;
  msg.gps_time.fraction_ns = contents.gps_time.fraction_ns;

  msg.num_satellites = contents.num_satellites;
  return msg;
}

// Calibration Status
static fusion_engine_msgs::msg::CalibrationStatus populate(const point_one::fusion_engine::messages::CalibrationStatusMessage& contents) {
  fusion_engine_msgs::msg::CalibrationStatus msg;

  msg.p1_time.seconds      = contents.p1_time.seconds;
  msg.p1_time.fraction_ns  = contents.p1_time.fraction_ns;

  msg.calibration_stage = static_cast<uint8_t>(contents.calibration_stage);

  msg.ypr_mounting_angle[0] = contents.ypr_deg[0];
  msg.ypr_mounting_angle[1] = contents.ypr_deg[1];
  msg.ypr_mounting_angle[2] = contents.ypr_deg[2];

  msg.ypr_std_dev_deg[0] = contents.ypr_std_dev_deg[0];
  msg.ypr_std_dev_deg[1] = contents.ypr_std_dev_deg[1];
  msg.ypr_std_dev_deg[2] = contents.ypr_std_dev_deg[2];

  msg.travel_distance_m = contents.travel_distance_m;
  msg.state_verified    = contents.state_verified;

  msg.gyro_bias_percent        = contents.gyro_bias_percent_complete * 0.5;
  msg.accel_bias_percent       = contents.accel_bias_percent_complete * 0.5;
  msg.mounting_angle_bias_percent = contents.mounting_angle_percent_complete * 0.5;

  msg.min_travel_distance_m = contents.min_travel_distance_m;
  msg.max_ypr_std_dev_deg[0] = contents.mounting_angle_max_std_dev_deg[0];
  msg.max_ypr_std_dev_deg[1] = contents.mounting_angle_max_std_dev_deg[1];
  msg.max_ypr_std_dev_deg[2] = contents.mounting_angle_max_std_dev_deg[2];

  return msg;
}

// Relative ENU Position
static fusion_engine_msgs::msg::RelativeEnuPosition populate(const point_one::fusion_engine::messages::RelativeENUPositionMessage& contents) {
  fusion_engine_msgs::msg::RelativeEnuPosition msg;

  msg.p1_time.seconds      = contents.p1_time.seconds;
  msg.p1_time.fraction_ns  = contents.p1_time.fraction_ns;
  msg.gps_time.seconds     = contents.gps_time.seconds;
  msg.gps_time.fraction_ns = contents.gps_time.fraction_ns;

  msg.solution_type        = static_cast<uint8_t>(contents.solution_type);
  msg.reference_station_id = contents.reference_station_id;

  msg.east_m  = contents.relative_position_enu_m[0];
  msg.north_m = contents.relative_position_enu_m[1];
  msg.up_m    = contents.relative_position_enu_m[2];

  msg.east_std_m  = contents.position_std_enu_m[0];
  msg.north_std_m = contents.position_std_enu_m[1];
  msg.up_std_m    = contents.position_std_enu_m[2];

  return msg;
}

  /***********************************************************************************************************/
  /* Calibrated Utils */

  static sensor_msgs::msg::Imu populate(const point_one::fusion_engine::messages::IMUOutput& contents) {
    sensor_msgs::msg::Imu msg;

    msg.linear_acceleration.x = contents.accel_mps2[0];
    msg.linear_acceleration.y = contents.accel_mps2[1];
    msg.linear_acceleration.z = contents.accel_mps2[2];

    msg.linear_acceleration_covariance[0] = contents.accel_std_mps2[0] * contents.accel_std_mps2[0];
    msg.linear_acceleration_covariance[4] = contents.accel_std_mps2[1] * contents.accel_std_mps2[1];
    msg.linear_acceleration_covariance[8] = contents.accel_std_mps2[2] * contents.accel_std_mps2[2];

    msg.angular_velocity.x = contents.gyro_rps[0];
    msg.angular_velocity.y = contents.gyro_rps[1];
    msg.angular_velocity.z = contents.gyro_rps[2];

    msg.angular_velocity_covariance[0] = contents.gyro_std_rps[0] * contents.gyro_std_rps[0];
    msg.angular_velocity_covariance[4] = contents.gyro_std_rps[1] * contents.gyro_std_rps[1];
    msg.angular_velocity_covariance[8] = contents.gyro_std_rps[2] * contents.gyro_std_rps[2];
    return msg;
  }

  static fusion_engine_msgs::msg::GnssAttitudeOutput populate(const point_one::fusion_engine::messages::GNSSAttitudeOutput& contents) {
    fusion_engine_msgs::msg::GnssAttitudeOutput msg;

    msg.solution_type = static_cast<int>(contents.solution_type);
    msg.flags         = contents.flags;


    msg.ypr_deg[0] = contents.ypr_deg[0]; msg.ypr_deg[1] = contents.ypr_deg[1]; msg.ypr_deg[2] = contents.ypr_deg[2];
    msg.ypr_std_deg[0] = contents.ypr_std_deg[0]; msg.ypr_std_deg[1] = contents.ypr_std_deg[1]; msg.ypr_std_deg[2] = contents.ypr_std_deg[2];

    msg.baseline_distance_m      = contents.baseline_distance_m;
    msg.baseline_distance_std_m  = contents.baseline_distance_std_m;

    return msg;
  }

  static fusion_engine_msgs::msg::WheelSpeedOutput populate(const point_one::fusion_engine::messages::WheelSpeedOutput& contents) {
    fusion_engine_msgs::msg::WheelSpeedOutput msg;

    msg.front_left_speed  = contents.front_left_speed_mps;
    msg.front_right_speed = contents.front_right_speed_mps;
    msg.rear_left_speed   = contents.rear_left_speed_mps;
    msg.rear_right_speed  = contents.rear_right_speed_mps;

    msg.gear  = static_cast<int>(contents.gear);
    msg.flags = contents.flags;

    return msg;
  }

  static fusion_engine_msgs::msg::VehicleSpeedOutput populate(const point_one::fusion_engine::messages::VehicleSpeedOutput& contents) {
    fusion_engine_msgs::msg::VehicleSpeedOutput msg;

    msg.speed = contents.vehicle_speed_mps;
    msg.gear  = static_cast<int>(contents.gear);
    msg.flags = contents.flags;

    return msg;
  }

  /***********************************************************************************************************/
  /* Raw Utils */
  static fusion_engine_msgs::msg::RawImuOutput populate(const point_one::fusion_engine::messages::RawIMUOutput& contents) {
    fusion_engine_msgs::msg::RawImuOutput msg;

    // Convert accel (m/s² * 2^-16)
    msg.accel[0] = (contents.accel[0] == INT32_MAX) ? NAN
                 : static_cast<double>(contents.accel[0]) / 65536.0;
    msg.accel[1] = (contents.accel[1] == INT32_MAX) ? NAN
                 : static_cast<double>(contents.accel[1]) / 65536.0;
    msg.accel[2] = (contents.accel[2] == INT32_MAX) ? NAN
                 : static_cast<double>(contents.accel[2]) / 65536.0;
    
    // Convert gyro (rad/s * 2^-16)
    msg.gyro[0] = (contents.gyro[0] == INT32_MAX) ? NAN
                : static_cast<double>(contents.gyro[0]) / 65536.0;
    msg.gyro[1] = (contents.gyro[1] == INT32_MAX) ? NAN
                : static_cast<double>(contents.gyro[1]) / 65536.0;
    msg.gyro[2] = (contents.gyro[2] == INT32_MAX) ? NAN
                : static_cast<double>(contents.gyro[2]) / 65536.0;

    // Convert temperature (°C * 2^-7)
    msg.temperature = (contents.temperature == INT16_MAX) ? NAN
                      : static_cast<double>(contents.temperature) / 128.0;
    return msg;
    }

  
  static fusion_engine_msgs::msg::RawGnssAttitudeOutput populate(const point_one::fusion_engine::messages::RawGNSSAttitudeOutput& contents) {
    fusion_engine_msgs::msg::RawGnssAttitudeOutput msg;
    msg.solution_type = static_cast<int>(contents.solution_type);
    msg.flags         = contents.flags;

    msg.relative_position_enu_m.x = contents.relative_position_enu_m[0]; msg.relative_position_enu_m.y = contents.relative_position_enu_m[1]; msg.relative_position_enu_m.z = contents.relative_position_enu_m[2];
    msg.position_std_enu_m.x = contents.position_std_enu_m[0]; msg.position_std_enu_m.y = contents.position_std_enu_m[1]; msg.position_std_enu_m.z = contents.position_std_enu_m[2];
    return msg;
  }
  
  static fusion_engine_msgs::msg::RawWheelSpeedOutput populate(const point_one::fusion_engine::messages::RawWheelSpeedOutput& contents) {
    fusion_engine_msgs::msg::RawWheelSpeedOutput msg;
    msg.front_left_speed  = (contents.front_left_speed  == INT32_MAX) ? NAN : contents.front_left_speed  / 1024.0;
    msg.front_right_speed = (contents.front_right_speed == INT32_MAX) ? NAN : contents.front_right_speed / 1024.0;
    msg.rear_left_speed   = (contents.rear_left_speed   == INT32_MAX) ? NAN : contents.rear_left_speed   / 1024.0;
    msg.rear_right_speed  = (contents.rear_right_speed  == INT32_MAX) ? NAN : contents.rear_right_speed  / 1024.0;
    msg.gear = static_cast<int>(contents.gear);
    return msg;
  }

  static fusion_engine_msgs::msg::RawVehicleSpeedOutput populate(const point_one::fusion_engine::messages::RawVehicleSpeedOutput& contents) {
    fusion_engine_msgs::msg::RawVehicleSpeedOutput msg;
    msg.vehicle_speed = contents.vehicle_speed;
    msg.gear = static_cast<int>(contents.gear);
    return msg;
  }

  static fusion_engine_msgs::msg::RawWheelTickOutput populate(const point_one::fusion_engine::messages::RawWheelTickOutput& contents) {
    fusion_engine_msgs::msg::RawWheelTickOutput msg;
    msg.front_left_wheel_ticks  = contents.front_left_wheel_ticks;
    msg.front_right_wheel_ticks = contents.front_right_wheel_ticks;
    msg.rear_left_wheel_ticks   = contents.rear_left_wheel_ticks;
    msg.rear_right_wheel_ticks  = contents.rear_right_wheel_ticks;
    msg.gear = static_cast<int>(contents.gear);
    return msg;
  }
  
  static fusion_engine_msgs::msg::RawVehicleTickOutput populate(const point_one::fusion_engine::messages::RawVehicleTickOutput& contents) {
    fusion_engine_msgs::msg::RawVehicleTickOutput msg;
    msg.tick_count = contents.tick_count;
    msg.gear = static_cast<int>(contents.gear);
    return msg;
  }

  /***********************************************************************************************************/
  /* ROS Utils */
  static geometry_msgs::msg::PoseStamped populate(const point_one::fusion_engine::messages::ros::PoseMessage& contents) {
    geometry_msgs::msg::PoseStamped msg;
    msg.pose.position.x    = contents.position_rel_m[0]; msg.pose.position.y    = contents.position_rel_m[1]; msg.pose.position.z    = contents.position_rel_m[2];
    msg.pose.orientation.x = contents.orientation[0];    msg.pose.orientation.y = contents.orientation[1];    msg.pose.orientation.z = contents.orientation[2]; msg.pose.orientation.w = contents.orientation[3];
    return msg;
  }

  static gps_msgs::msg::GPSFix populate(const point_one::fusion_engine::messages::ros::GPSFixMessage& contents) {
    gps_msgs::msg::GPSFix msg;
    msg.latitude  = contents.latitude_deg;    msg.longitude = contents.longitude_deg;
    msg.altitude  = contents.altitude_m;      msg.track     = contents.track_deg;
    msg.speed     = contents.speed_mps;       msg.climb     = contents.climb_mps;
    msg.pitch     = contents.pitch_deg;       msg.roll      = contents.roll_deg;       msg.dip = contents.dip_deg;

    msg.time      = contents.p1_time.seconds;
    
    msg.gdop      = contents.gdop;           msg.pdop      = contents.pdop;           msg.hdop = contents.hdop;
    msg.vdop      = contents.vdop;           msg.tdop      = contents.tdop;

    msg.err       = contents.err_3d_m;       msg.err_horz  = contents.err_horiz_m;    msg.err_vert  = contents.err_vert_m;
    msg.err_track = contents.err_track_deg;  msg.err_speed = contents.err_speed_mps;  msg.err_climb = contents.err_climb_mps;
    msg.err_pitch = contents.err_pitch_deg;  msg.err_roll  = contents.err_roll_deg;   msg.err_dip   = contents.err_dip_deg;
    msg.err_time  = contents.err_time_sec;
    std::copy(std::begin(contents.position_covariance_m2), std::end(contents.position_covariance_m2),
              std::begin(msg.position_covariance));
    msg.position_covariance_type = contents.position_covariance_type;
    return msg;
  }

  static sensor_msgs::msg::Imu populate(const point_one::fusion_engine::messages::ros::IMUMessage& contents) {
    sensor_msgs::msg::Imu msg;
    msg.orientation.x = contents.orientation[0]; msg.orientation.y = contents.orientation[1];
    msg.orientation.z = contents.orientation[2]; msg.orientation.w = contents.orientation[3];

    msg.angular_velocity.x = contents.angular_velocity_rps[0];
    msg.angular_velocity.y = contents.angular_velocity_rps[1];
    msg.angular_velocity.z = contents.angular_velocity_rps[2];

    msg.linear_acceleration.x = contents.acceleration_mps2[0];
    msg.linear_acceleration.y = contents.acceleration_mps2[1];
    msg.linear_acceleration.z = contents.acceleration_mps2[2];

    std::copy(std::begin(contents.orientation_covariance), std::end(contents.orientation_covariance),
              std::begin(msg.orientation_covariance));

    std::copy(std::begin(contents.angular_velocity_covariance), std::end(contents.angular_velocity_covariance),
              std::begin(msg.angular_velocity_covariance));

    std::copy(std::begin(contents.acceleration_covariance), std::end(contents.acceleration_covariance),
              std::begin(msg.linear_acceleration_covariance));
    return msg;
  }  

  /***********************************************************************************************************/
  /**
   * @brief Calculate the XOR checksum of a string message.
   *
   * @param message The string message to calculate the checksum for.
   * @return The calculated checksum as a single char.
   */
  static char calculateChecksum(const std::string& message) {
    char checksum = 0;
    for (auto c : message) {
      checksum ^= c;
    }
    return checksum;
  }

  static int convertNtpToUnix(int current_timestamp) {
    return current_timestamp - 2208988800;
  }

  static int64_t convertUnixToNtp(int64_t current_timestamp) {
    return current_timestamp + 2208988800;
  }

  /**
   * @brief Convert GPS time to UTC time in string format.
   *
   * @param gps_time The GPS time to convert, in seconds.
   * @return The UTC time in string format (hhmmss.ssssss).
   */
  static std::string convertGpsToUtc(double gps_time) {
    const int kSecondsInDay = 86400;
    auto currentTime = std::chrono::system_clock::now();
    std::time_t currentTimeT = std::chrono::system_clock::to_time_t(currentTime);
    int64_t currentTimestamp = static_cast<int64_t>(currentTimeT);
    const int kLeapSecondsOffset = getLeapSeconds(currentTimestamp);

    // Extract GPS time components
    int gps_hours = static_cast<int>((gps_time / 3600.0));
    int gps_minutes = static_cast<int>((gps_time - gps_hours * 3600.0) / 60.0);
    double gps_seconds = gps_time - gps_hours * 3600.0 - gps_minutes * 60.0;

    // Convert GPS time to seconds
    int gps_seconds_total =
        gps_hours * 3600 + gps_minutes * 60 + static_cast<int>(gps_seconds);

    // Subtract leap seconds offset
    int utc_seconds_total = gps_seconds_total - kLeapSecondsOffset;

    // Handle cases where the UTC time is negative or greater than 24 hours
    if (utc_seconds_total < 0) {
      utc_seconds_total += kSecondsInDay;
    } else if (utc_seconds_total >= kSecondsInDay) {
      utc_seconds_total -= kSecondsInDay;
    }

    // Convert UTC time to hours, minutes, and seconds
    int utc_hours = int(utc_seconds_total / 3600) % 24;
    int utc_minutes = (utc_seconds_total % 3600) / 60;
    double utc_seconds = static_cast<double>(utc_seconds_total % 60) +
                         gps_seconds - static_cast<int>(gps_seconds);

    // Create output string with the desired format
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(2) << utc_hours << std::setw(2)
       << utc_minutes << std::fixed << std::setprecision(3) << std::setw(6)
       << utc_seconds;
    return ss.str();
  }

  /**
   * @brief Converts a degree-based latitude or longitude to a NMEA DDMM format
   * @param angle_deg The angle to be converted in degrees
   * @param is_longitude A flag indicating whether the angle represents a
   * longitude
   * @return A string in NMEA DDMM format
   */
  static std::string nmeaDegToDdmm(double angle_deg,
                                      bool is_longitude = false) {
    std::string direction;
    if (is_longitude) {
      direction = angle_deg >= 0.0 ? "E" : "W";
    } else {
      direction = angle_deg >= 0.0 ? "N" : "S";
    }

    double abs_angle_deg = std::fabs(angle_deg);
    int degree = static_cast<int>(abs_angle_deg);
    double minute = (abs_angle_deg - degree) * 60.0;

    std::ostringstream oss;
    oss << degree << std::fixed << std::setprecision(2) << minute << ","
        << direction;
    return oss.str();
  }

  /**
   * @brief Converts the given PoseMessage to a NMEA GPGGA sentence.
   *
   * @param contents The PoseMessage to convert.
   * @param nb_satellite The number of GPS satellites used to generate the pose.
   * @return A NMEA GPGGA sentence as a nmea_msgs::msg::Sentence.
   */
  static nmea_msgs::msg::Sentence toNMEA(
      const point_one::fusion_engine::messages::PoseMessage& contents,
      uint16_t& nb_satellite) {
    double gps_time_sec =
        contents.gps_time.seconds + contents.gps_time.fraction_ns * 1e-9;

    std::stringstream nmea_stream;
    nmea_msgs::msg::Sentence nmea;
    nmea_stream << "$GPGGA,";
    nmea_stream << convertGpsToUtc(gps_time_sec) << ",";
    nmea_stream << nmeaDegToDdmm(contents.lla_deg[0], false) << ",";
    nmea_stream << nmeaDegToDdmm(contents.lla_deg[1], true) << ",";
    // nmea_stream << static_cast<uint8_t>(contents.solution_type) << ",";
    nmea_stream << std::setprecision(1) << 1 << ",";    // hdop
    nmea_stream << nb_satellite << ",";                 // nb sattelite
    nmea_stream << std::setprecision(1) << 1.6 << ",";  // antenna alt
    nmea_stream << std::setprecision(1) << 1.6 << ",";
    nmea_stream << "M,";  // unit alt M or F
    nmea_stream << std::setprecision(4) << contents.lla_deg[2] << ",";
    nmea_stream << "M,-20.7,M,,";  // unit alt M or F
    unsigned char checksum = calculateChecksum(nmea_stream.str().substr(1));

    nmea_stream << "*" << std::hex << static_cast<int>(checksum) << std::endl;
    nmea.sentence = nmea_stream.str();
    return nmea;
  }

  static int getLeapSeconds(int64_t current_timestamp) {
    static const std::map<int64_t, int> leap_seconds =
    {
      {2272060800, 10},     // 1 Jan 1972
      {2287785600, 11},     // 1 Jul 1972
      {2303683200, 12},     // 1 Jan 1973
      {2335219200, 13},     // 1 Jan 1974
      {2366755200, 14},     // 1 Jan 1975
      {2398291200, 15},     // 1 Jan 1976
      {2429913600, 16},     // 1 Jan 1977
      {2461449600, 17},     // 1 Jan 1978
      {2492985600, 18},     // 1 Jan 1979
      {2524521600, 19},     // 1 Jan 1980
      {2571782400, 20},     // 1 Jul 1981
      {2603318400, 21},     // 1 Jul 1982
      {2634854400, 22},     // 1 Jul 1983
      {2698012800, 23},     // 1 Jul 1985
      {2776982400, 24},     // 1 Jan 1988
      {2840140800, 25},     // 1 Jan 1990
      {2871676800, 26},     // 1 Jan 1991
      {2918937600, 27},     // 1 Jul 1992
      {2950473600, 28},     // 1 Jul 1993
      {2982009600, 29},     // 1 Jul 1994
      {3029443200, 30},     // 1 Jan 1996
      {3076704000, 31},     // 1 Jul 1997
      {3124137600, 32},     // 1 Jan 1999
      {3345062400, 33},     // 1 Jan 2006
      {3439756800, 34},     // 1 Jan 2009
      {3550089600, 35},     // 1 Jul 2012
      {3644697600, 36},     // 1 Jul 2015
      {3692217600, 37}      // 1 Jan 2017
    };
    int64_t timestamp_converted = convertUnixToNtp(current_timestamp);
    auto result = leap_seconds.lower_bound(timestamp_converted);
    // Here we handle the case where our input time is outside
    // of our map bounds.
    if (result == leap_seconds.end()) {
      return std::prev(result)->second;
    }
    // Here we handle the case where our input time is within range of our
    // map bounds and is not equal in time to a specific key.
    if (result->first > timestamp_converted) {
      return std::prev(result)->second;
    }
    // Here we handle the case where our input time is within range of our
    // map bounds and is equal in time to a specific key.
    return result->second;
  }
};
