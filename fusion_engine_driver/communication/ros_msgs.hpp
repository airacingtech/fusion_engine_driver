#ifndef FUSION_ENGINE_DRIVER__COMMUNICATION__ROS_MSGS_HPP_
#define FUSION_ENGINE_DRIVER__COMMUNICATION__ROS_MSGS_HPP_

#include <algorithm>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
namespace ros_msgs {
struct Pose : public geometry_msgs::msg::PoseStamped {
  inline explicit Pose(const point_one::fusion_engine::messages::ros::PoseMessage& p) {
    pose.position.x = p.position_rel_m[0];
    pose.position.y = p.position_rel_m[1];
    pose.position.z = p.position_rel_m[2];

    pose.orientation.x = p.orientation[0];
    pose.orientation.y = p.orientation[1];
    pose.orientation.z = p.orientation[2];
    pose.orientation.w = p.orientation[3];
  }
};

struct GPSFix : public gps_msgs::msg::GPSFix {
  inline explicit GPSFix(const point_one::fusion_engine::messages::ros::GPSFixMessage& p) {
    latitude  = p.latitude_deg;
    longitude = p.longitude_deg;
    altitude  = p.altitude_m;
    track     = p.track_deg;
    speed     = p.speed_mps;
    climb     = p.climb_mps;
    pitch     = p.pitch_deg;
    roll      = p.roll_deg;
    dip       = p.dip_deg;

    time = p.p1_time.seconds;

    gdop = p.gdop;
    pdop = p.pdop;
    hdop = p.hdop;
    vdop = p.vdop;
    tdop = p.tdop;

    err        = p.err_3d_m;
    err_horz   = p.err_horiz_m;
    err_vert   = p.err_vert_m;
    err_track  = p.err_track_deg;
    err_speed  = p.err_speed_mps;
    err_climb  = p.err_climb_mps;
    err_pitch  = p.err_pitch_deg;
    err_roll   = p.err_roll_deg;
    err_dip    = p.err_dip_deg;
    err_time   = p.err_time_sec;

    std::copy(
      std::begin(p.position_covariance_m2),
      std::end(p.position_covariance_m2),
      std::begin(position_covariance));

    position_covariance_type = p.position_covariance_type;
  }
};

struct Imu : public sensor_msgs::msg::Imu {
  inline explicit Imu(const point_one::fusion_engine::messages::ros::IMUMessage& p) {
    orientation.x = p.orientation[0];
    orientation.y = p.orientation[1];
    orientation.z = p.orientation[2];
    orientation.w = p.orientation[3];

    angular_velocity.x = p.angular_velocity_rps[0];
    angular_velocity.y = p.angular_velocity_rps[1];
    angular_velocity.z = p.angular_velocity_rps[2];

    linear_acceleration.x = p.acceleration_mps2[0];
    linear_acceleration.y = p.acceleration_mps2[1];
    linear_acceleration.z = p.acceleration_mps2[2];

    std::copy(
      std::begin(p.orientation_covariance),
      std::end(p.orientation_covariance),
      std::begin(orientation_covariance));

    std::copy(
      std::begin(p.angular_velocity_covariance),
      std::end(p.angular_velocity_covariance),
      std::begin(angular_velocity_covariance));

    std::copy(
      std::begin(p.acceleration_covariance),
      std::end(p.acceleration_covariance),
      std::begin(linear_acceleration_covariance));
  }
};

}  // namespace ros_msgs

#endif  // FUSION_ENGINE_DRIVER__COMMUNICATION__ROS_MSGS_HPP_
