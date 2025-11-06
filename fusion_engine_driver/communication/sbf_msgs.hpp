#ifndef FUSION_ENGINE_DRIVER__COMMUNICATION__SBF_MSGS_HPP_
#define FUSION_ENGINE_DRIVER__COMMUNICATION__SBF_MSGS_HPP_

#include "sbf_typedefs.hpp"

#include "fusion_engine_msgs/msg/pvt_cartesian.hpp"
#include "fusion_engine_msgs/msg/pvt_geodetic.hpp"
#include "fusion_engine_msgs/msg/pos_cov_geodetic.hpp"
#include "fusion_engine_msgs/msg/vel_cov_geodetic.hpp"

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
    latitude = p.latitude;
    longitude = p.longitude;
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

struct PosCovGeodetic : public fusion_engine_msgs::msg::PosCovGeodetic {
  ~PosCovGeodetic() noexcept = default;
  inline explicit PosCovGeodetic(const ::PosCovGeodetic& p) noexcept {
    tow_ms = p.TOW;
    week_number = p.WNc;
    cov_nn = p.Cov_NN;
    cov_ne = p.Cov_NE;
    cov_nu = p.Cov_NU;
    cov_ee = p.Cov_EE;
    cov_eu = p.Cov_EU;
    cov_uu = p.Cov_UU;
  }
};

struct VelCovGeodetic : public fusion_engine_msgs::msg::VelCovGeodetic {
  ~VelCovGeodetic() noexcept = default;
  inline explicit VelCovGeodetic(const ::VelCovGeodetic& p) noexcept {
    tow_ms = p.TOW;
    week_number = p.WNc;
    cov_vn_vn = p.Cov_VnVn;
    cov_vn_ve = p.Cov_VnVe;
    cov_vn_vu = p.Cov_VnVu;
    cov_ve_ve = p.Cov_VeVe;
    cov_ve_vu = p.Cov_VeVu;
    cov_vu_vu = p.Cov_VuVu;
  }
};
}  // namespace sbf_msgs

#endif  // FUSION_ENGINE_DRIVER__COMMUNICATION__SBF_MSGS_HPP_