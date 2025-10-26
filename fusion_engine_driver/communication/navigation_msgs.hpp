#ifndef FUSION_ENGINE_DRIVER__COMMUNICATION__NAVIGATION_MSGS_HPP_
#define FUSION_ENGINE_DRIVER__COMMUNICATION__NAVIGATION_MSGS_HPP_

#include "fusion_engine_msgs/msg/pose.hpp"
#include "fusion_engine_msgs/msg/pose_aux.hpp"
#include "fusion_engine_msgs/msg/calibration_status.hpp"
#include "fusion_engine_msgs/msg/relative_enu_position.hpp"
#include "fusion_engine_msgs/msg/gnss_info.hpp"
#include "fusion_engine_msgs/msg/gnss_satellite.hpp"
#include "fusion_engine_msgs/msg/gnss_satellite_info.hpp"

namespace navigation_msgs {
struct Pose : public fusion_engine_msgs::msg::Pose {
  ~Pose() noexcept = default;
  inline explicit Pose(const point_one::fusion_engine::messages::PoseMessage& p) noexcept {
    p1_time.seconds = p.p1_time.seconds;
    p1_time.fraction_ns = p.p1_time.fraction_ns;
    gps_time.seconds = p.gps_time.seconds;
    gps_time.fraction_ns = p.gps_time.fraction_ns;

    solution_type = static_cast<uint8_t>(p.solution_type);
    undulation = p.undulation_cm;
    latitude  = p.lla_deg[0];
    longitude = p.lla_deg[1];
    altitude  = p.lla_deg[2];

    position_covariance[0] = p.position_std_enu_m[0] * p.position_std_enu_m[0];
    position_covariance[4] = p.position_std_enu_m[1] * p.position_std_enu_m[1];
    position_covariance[8] = p.position_std_enu_m[2] * p.position_std_enu_m[2];

    rpy.roll  = p.ypr_deg[2];
    rpy.pitch = p.ypr_deg[1];
    rpy.yaw   = p.ypr_deg[0];
    rpy_covariance[0] = p.ypr_std_deg[2] * p.ypr_std_deg[2];
    rpy_covariance[4] = p.ypr_std_deg[1] * p.ypr_std_deg[1];
    rpy_covariance[8] = p.ypr_std_deg[0] * p.ypr_std_deg[0];

    velflu.x = p.velocity_body_mps[0];
    velflu.y = p.velocity_body_mps[1];
    velflu.z = p.velocity_body_mps[2];

    velflu_covariance[0] = p.velocity_std_body_mps[0] * p.velocity_std_body_mps[0];
    velflu_covariance[4] = p.velocity_std_body_mps[1] * p.velocity_std_body_mps[1];
    velflu_covariance[8] = p.velocity_std_body_mps[2] * p.velocity_std_body_mps[2];

    aggregate_protection_level  = p.aggregate_protection_level_m;
    horizontal_protection_level = p.horizontal_protection_level_m;
    vertical_protection_level   = p.vertical_protection_level_m;
  }
};

struct PoseAux : public fusion_engine_msgs::msg::PoseAux {
  ~PoseAux() noexcept = default;
  inline explicit PoseAux(const point_one::fusion_engine::messages::PoseAuxMessage& p) {
    p1_time.seconds = p.p1_time.seconds;
    p1_time.fraction_ns = p.p1_time.fraction_ns;

    attitude.x = p.attitude_quaternion[0];
    attitude.y = p.attitude_quaternion[1];
    attitude.z = p.attitude_quaternion[2];
    attitude.w = p.attitude_quaternion[3];

    velenu.x = p.velocity_enu_mps[0];
    velenu.y = p.velocity_enu_mps[1];
    velenu.z = p.velocity_enu_mps[2];

    velenu_covariance[0] = p.velocity_std_enu_mps[0] * p.velocity_std_enu_mps[0];
    velenu_covariance[1] = p.velocity_std_enu_mps[1] * p.velocity_std_enu_mps[1];
    velenu_covariance[2] = p.velocity_std_enu_mps[2] * p.velocity_std_enu_mps[2];

    posbody_covariance[0] = p.position_std_body_m[0] * p.position_std_body_m[0];
    posbody_covariance[1] = p.position_std_body_m[1] * p.position_std_body_m[1];
    posbody_covariance[2] = p.position_std_body_m[2] * p.position_std_body_m[2];

    posenu_covariance[0] = p.position_cov_enu_m2[0];
    posenu_covariance[1] = p.position_cov_enu_m2[4];
    posenu_covariance[2] = p.position_cov_enu_m2[8];
  }
}; 

struct GnssInfo : public fusion_engine_msgs::msg::GnssInfo {
  ~GnssInfo() noexcept = default;
  inline explicit GnssInfo(const point_one::fusion_engine::messages::GNSSInfoMessage& p) {
    p1_time.seconds = p.p1_time.seconds;
    p1_time.fraction_ns = p.p1_time.fraction_ns;
    gps_time.seconds = p.gps_time.seconds;
    gps_time.fraction_ns = p.gps_time.fraction_ns;

    leap_second = p.leap_second;
    num_satellites = p.num_svs;
    corrections_age = p.corrections_age;
    baseline_distance = p.baseline_distance;
    reference_station_id = p.reference_station_id;

    gdop = p.gdop;
    pdop = p.pdop;
    hdop = p.hdop;
    vdop = p.vdop;

    gps_time_covariance = p.gps_time_std_sec * p.gps_time_std_sec;
  }
};

struct GnssSatelliteInfo : public fusion_engine_msgs::msg::GnssSatelliteInfo {
  ~GnssSatelliteInfo() noexcept = default;
  inline explicit GnssSatelliteInfo(const point_one::fusion_engine::messages::SatelliteInfo& sat) noexcept {
    prn = sat.prn;
    constellation = static_cast<uint8_t>(sat.system);
    cn0 = sat.cn0;
    elevation = sat.elevation_deg;
    azimuth = sat.azimuth_deg;
    usage = sat.usage;
  }
};

struct GnssSatellite : public fusion_engine_msgs::msg::GnssSatellite {
  ~GnssSatellite() noexcept = default;
  inline explicit GnssSatellite(const point_one::fusion_engine::messages::GNSSSatelliteMessage& p) noexcept {
    p1_time.seconds = p.p1_time.seconds;
    p1_time.fraction_ns = p.p1_time.fraction_ns;
    gps_time.seconds = p.gps_time.seconds;
    gps_time.fraction_ns = p.gps_time.fraction_ns;

    num_satellites = p.num_satellites;
  }
};

struct CalibrationStatus : public fusion_engine_msgs::msg::CalibrationStatus {
  ~CalibrationStatus() noexcept = default;
  inline explicit CalibrationStatus(const point_one::fusion_engine::messages::CalibrationStatusMessage& p) noexcept {
    p1_time.seconds = p.p1_time.seconds;
    p1_time.fraction_ns = p.p1_time.fraction_ns;
    calibration_stage = static_cast<uint8_t>(p.calibration_stage);
    state_verified = p.state_verified;

    rpy_mounting.roll  = p.ypr_deg[2];
    rpy_mounting.pitch = p.ypr_deg[1];
    rpy_mounting.yaw   = p.ypr_deg[0];

    rpy_mounting_covariance[2] = p.ypr_std_dev_deg[2] * p.ypr_std_dev_deg[2];
    rpy_mounting_covariance[1] = p.ypr_std_dev_deg[1] * p.ypr_std_dev_deg[1];
    rpy_mounting_covariance[0] = p.ypr_std_dev_deg[0] * p.ypr_std_dev_deg[0];

    rpy_max_covariance[2] = p.mounting_angle_max_std_dev_deg[2] * p.mounting_angle_max_std_dev_deg[2];
    rpy_max_covariance[1] = p.mounting_angle_max_std_dev_deg[1] * p.mounting_angle_max_std_dev_deg[1];
    rpy_max_covariance[0] = p.mounting_angle_max_std_dev_deg[0] * p.mounting_angle_max_std_dev_deg[0];

    travel_distance = p.travel_distance_m;
    min_travel_distance = p.min_travel_distance_m;

    gyro_bias_percent        = p.gyro_bias_percent_complete * 0.5;
    accel_bias_percent       = p.accel_bias_percent_complete * 0.5;
    mounting_angle_bias_percent = p.mounting_angle_percent_complete * 0.5;
  }
};

struct RelativeEnuPosition : public fusion_engine_msgs::msg::RelativeEnuPosition {
  ~RelativeEnuPosition() noexcept = default;
  inline explicit RelativeEnuPosition(const point_one::fusion_engine::messages::RelativeENUPositionMessage& p) noexcept {
    p1_time.seconds = p.p1_time.seconds;
    p1_time.fraction_ns = p.p1_time.fraction_ns;
    gps_time.seconds = p.gps_time.seconds;
    gps_time.fraction_ns = p.gps_time.fraction_ns;

    solution_type = static_cast<uint8_t>(p.solution_type);
    reference_station_id = p.reference_station_id;

    east  = p.relative_position_enu_m[0];
    north = p.relative_position_enu_m[1];
    up    = p.relative_position_enu_m[2];

    enu_covariance[0] = p.position_std_enu_m[0] * p.position_std_enu_m[0];
    enu_covariance[1] = p.position_std_enu_m[1] * p.position_std_enu_m[1];
    enu_covariance[2] = p.position_std_enu_m[2] * p.position_std_enu_m[2];

  }
};

}  // namespace navigation_msgs
#endif // FUSION_ENGINE_DRIVER__COMMUNICATION__NAVIGATION_MSGS_HPP_