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

class ConversionUtils {
  public:

  /***********************************************************************************************************/
  /* ROS Wrappers */
  static geometry_msgs::msg::PoseStamped populate(const point_one::fusion_engine::messages::ros::PoseMessage& contents) {
    geometry_msgs::msg::PoseStamped msg;
    msg.pose.position.x    = contents.position_rel_m[0]; msg.pose.position.y    = contents.position_rel_m[1]; msg.pose.position.z    = contents.position_rel_m[2];
    msg.pose.orientation.x = contents.orientation[0];    msg.pose.orientation.y = contents.orientation[1];    msg.pose.orientation.z = contents.orientation[2]; msg.pose.orientation.w = contents.orientation[3];
    return msg;
  }

  // static gps_msgs::msg::GPSFix populate(const point_one::fusion_engine::messages::ros::GPSFixMessage& contents) {
  //   gps_msgs::msg::GPSFix msg;
  //   msg.latitude  = contents.latitude_deg;    msg.longitude = contents.longitude_deg;
  //   msg.altitude  = contents.altitude_m;      msg.track     = contents.track_deg;
  //   msg.speed     = contents.speed_mps;       msg.climb     = contents.climb_mps;
  //   msg.pitch     = contents.pitch_deg;       msg.roll      = contents.roll_deg;       msg.dip = contents.dip_deg;

  //   msg.time      = contents.p1_time.seconds;
    
  //   msg.gdop      = contents.gdop;           msg.pdop      = contents.pdop;           msg.hdop = contents.hdop;
  //   msg.vdop      = contents.vdop;           msg.tdop      = contents.tdop;

  //   msg.err       = contents.err_3d_m;       msg.err_horz  = contents.err_horiz_m;    msg.err_vert  = contents.err_vert_m;
  //   msg.err_track = contents.err_track_deg;  msg.err_speed = contents.err_speed_mps;  msg.err_climb = contents.err_climb_mps;
  //   msg.err_pitch = contents.err_pitch_deg;  msg.err_roll  = contents.err_roll_deg;   msg.err_dip   = contents.err_dip_deg;
  //   msg.err_time  = contents.err_time_sec;
  //   std::copy(std::begin(contents.position_covariance_m2),
  //             std::end(contents.position_covariance_m2),
  //             std::begin(msg.position_covariance));
  //   msg.position_covariance_type = contents.position_covariance_type;
  //   return msg;
  // }

  // static sensor_msgs::msg::Imu populate(const point_one::fusion_engine::messages::ros::IMUMessage& contents) {
  //   sensor_msgs::msg::Imu msg;
  //   msg.orientation.x = contents.orientation[0]; msg.orientation.y = contents.orientation[1];
  //   msg.orientation.z = contents.orientation[2]; msg.orientation.w = contents.orientation[3];

  //   std::copy(std::begin(contents.orientation_covariance),
  //             std::end(contents.orientation_covariance),
  //             std::begin(msg.orientation_covariance));

  //   msg.angular_velocity.x = contents.angular_velocity_rps[0];
  //   msg.angular_velocity.y = contents.angular_velocity_rps[1];
  //   msg.angular_velocity.z = contents.angular_velocity_rps[2];

  //   std::copy(std::begin(contents.angular_velocity_covariance),
  //             std::end(contents.angular_velocity_covariance),
  //             std::begin(msg.angular_velocity_covariance));

  //   msg.linear_acceleration.x = contents.acceleration_mps2[0];
  //   msg.linear_acceleration.y = contents.acceleration_mps2[1];
  //   msg.linear_acceleration.z = contents.acceleration_mps2[2];

  //   std::copy(std::begin(contents.acceleration_covariance),
  //             std::end(contents.acceleration_covariance),
  //             std::begin(msg.linear_acceleration_covariance));
  //   return msg;
  // }

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
