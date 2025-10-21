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

#ifndef FUSION_ENGINE_DRIVER__COMMUNICATION__SEPENTRIO_MSGS_HPP_
#define FUSION_ENGINE_DRIVER__COMMUNICATION__SEPENTRIO_MSGS_HPP_

#include <septentrio_gnss_driver/msg/pvt_geodetic.hpp>

namespace septentrio_msgs {

struct PVTGeodetic : public septentrio_gnss_driver::msg::PVTGeodetic {
  inline explicit PVTGeodetic(
    const point_one::fusion_engine::messages::septentrio::PVTGeodeticMessage& p)
  {
    // ----------------------------------------------------------------------
    // Position (geodetic)
    // ----------------------------------------------------------------------
    latitude_deg = p.latitude_deg;
    longitude_deg = p.longitude_deg;
    height_m = p.height_m;

    // ----------------------------------------------------------------------
    // Standard deviation of position
    // ----------------------------------------------------------------------
    undulation_m = p.undulation_m;
    diff_age_s = p.diff_age_s;
    solution_age_s = p.solution_age_s;

    datum = p.datum;
    nr_sv = p.nr_sv;
    nr_sv_used = p.nr_sv_used;

    mode = p.mode;
    error = p.error;

    // ----------------------------------------------------------------------
    // Covariance and velocity
    // ----------------------------------------------------------------------
    std_dev_lat_m = p.std_dev_lat_m;
    std_dev_lon_m = p.std_dev_lon_m;
    std_dev_height_m = p.std_dev_height_m;

    // Horizontal and vertical accuracy
    h_accuracy_m = p.h_accuracy_m;
    v_accuracy_m = p.v_accuracy_m;

    // ----------------------------------------------------------------------
    // Velocity (ENU)
    // ----------------------------------------------------------------------
    vel_north_mps = p.vel_north_mps;
    vel_east_mps = p.vel_east_mps;
    vel_up_mps = p.vel_up_mps;

    // Standard deviations of velocity
    std_dev_north_mps = p.std_dev_north_mps;
    std_dev_east_mps = p.std_dev_east_mps;
    std_dev_up_mps = p.std_dev_up_mps;

    // ----------------------------------------------------------------------
    // Time & Quality indicators
    // ----------------------------------------------------------------------
    cno_max = p.cno_max;
    hdop = p.hdop;
    vdop = p.vdop;
    tdop = p.tdop;
    pdop = p.pdop;
    gps_tow = p.gps_tow;
    week = p.week;

    // ----------------------------------------------------------------------
    // Receiver & quality flags
    // ----------------------------------------------------------------------
    nav_system = p.nav_system;
    error_indicator = p.error_indicator;
    mode_indicator = p.mode_indicator;
    base_station_id = p.base_station_id;
    reserved = p.reserved;
  }
};


}  // namespace septentrio_msgs

#endif  // FUSION_ENGINE_DRIVER__COMMUNICATION__SEPENTRIO_MSGS_HPP_