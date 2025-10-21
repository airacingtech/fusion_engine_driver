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
#ifndef FUSION_ENGINE_DRIVER__CORE__FUSION_DRIVER_MSGS_HPP_
#define FUSION_ENGINE_DRIVER__CORE__FUSION_DRIVER_MSGS_HPP_

#include "std_msgs/msg/string.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"

#include "gps_msgs/msg/gps_fix.hpp"
#include "gps_msgs/msg/gps_status.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/rtcm.hpp"
#include "nmea_msgs/msg/sentence.hpp"

#include "fusion_engine_msgs/msg/pose.hpp"
#include "fusion_engine_msgs/msg/pose_aux.hpp"
#include "fusion_engine_msgs/msg/calibration_status.hpp"
#include "fusion_engine_msgs/msg/relative_enu_position.hpp"
#include "fusion_engine_msgs/msg/gnss_info.hpp"
#include "fusion_engine_msgs/msg/gnss_satellite.hpp"
#include "fusion_engine_msgs/msg/gnss_satellite_info.hpp"
#include "fusion_engine_msgs/msg/gnss_attitude_output.hpp"
#include "fusion_engine_msgs/msg/wheel_speed_output.hpp"
#include "fusion_engine_msgs/msg/vehicle_speed_output.hpp"
#include "fusion_engine_msgs/msg/raw_gnss_attitude_output.hpp"
#include "fusion_engine_msgs/msg/raw_vehicle_speed_output.hpp"
#include "fusion_engine_msgs/msg/raw_vehicle_tick_output.hpp"
#include "fusion_engine_msgs/msg/raw_wheel_speed_output.hpp"
#include "fusion_engine_msgs/msg/raw_wheel_tick_output.hpp"
#include "fusion_engine_msgs/msg/rpy.hpp"



#endif  // FUSION_ENGINE_DRIVER__CORE__FUSION_DRIVER_MSGS_HPP_