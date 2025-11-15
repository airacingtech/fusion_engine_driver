#ifndef FUSION_ENGINE_DRIVER__COMMUNICATION__SBF_MSGS_HPP_
#define FUSION_ENGINE_DRIVER__COMMUNICATION__SBF_MSGS_HPP_

#include "sbf_typedefs.hpp"
#include "sbf_utils.hpp"
#include "fusion_engine_msgs/msg/pvt_cartesian.hpp"
#include "fusion_engine_msgs/msg/pvt_geodetic.hpp"

namespace sbf_msgs {
struct PVTCartesian : public fusion_engine_msgs::msg::PVTCartesian {
  inline explicit PVTCartesian(const ::PVTCartesian& p) noexcept {
    tow_ms = p.TOW;
    week_number = p.WNc;
    mode = p.Mode;
    error = p.Error;
    x = p.x;
    y = p.y;
    z = p.z;
    undulation = p.Undulation;
    vx = p.vx;
    vy = p.vy;
    vz = p.vz;
    rx_clock_bias = p.RxClkBias;
    rx_clock_drift = p.RxClkDrift;
    time_system = p.TimeSystem;
    datum = p.Datum;
    num_sats = p.NrSV;
    wa_corr_info = p.WACorrInfo;
  }
};

struct PVTGeodetic : public fusion_engine_msgs::msg::PVTGeodetic {
  ~PVTGeodetic() noexcept = default;
  inline explicit PVTGeodetic(const ::PVTGeodetic& p) noexcept {
    tow_ms = p.TOW;
    week_number = p.WNc;
    time_system = p.timeSystem;
    datum = p.datum;
    latitude = p.latitude * 180.0 / M_PI;
    longitude = p.longitude * 180.0 / M_PI;
    height = p.height;
    undulation = p.undulation;
    north_velocity = p.vn;
    east_velocity = p.ve;
    up_velocity = p.vu;
    course_over_ground = p.cog;
    rx_clock_bias = p.RxClkBias;
    rx_clock_drift = p.RxClkDrift;
    latency = static_cast<float>(p.latency) * 0.0001f;
    horizontal_accuracy = static_cast<float>(p.HAccuracy) * 0.01f;
    vertical_accuracy = static_cast<float>(p.VAccuracy) * 0.01f;
    mean_corr_age = static_cast<float>(p.meanCorrAge) * 0.01f;
    reference_id = p.referenceID;
    signal_info = p.signalInfo;
    ppp_info = p.PPPInfo;
    mode = p.Mode;
    error = p.Error;
    num_sats = p.nrSV;
    wa_corr_info = p.WACorrInfo;
    alert_flag = p.alertFlag;
    num_bases = p.NrBases;
    misc_flags = p.misc;
  }
};

}  // namespace sbf_msgs

#endif  // FUSION_ENGINE_DRIVER__COMMUNICATION__SBF_MSGS_HPP_