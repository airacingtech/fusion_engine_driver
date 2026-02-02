cmake_minimum_required(VERSION 3.8)
project(fusion_engine_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Timestamp.msg"
  "msg/RPY.msg"
  "msg/navigation/Pose.msg"
  "msg/navigation/PoseAux.msg"
  "msg/navigation/GnssInfo.msg"
  "msg/navigation/GnssSatelliteInfo.msg"
  "msg/navigation/GnssSignalInfo.msg"
  "msg/navigation/GnssSignals.msg"
  "msg/navigation/CalibrationStatus.msg"
  "msg/navigation/RelativeEnuPosition.msg"
  "msg/calibrated/WheelSpeedOutput.msg"
  "msg/calibrated/VehicleSpeedOutput.msg"
  "msg/calibrated/GnssAttitudeOutput.msg"
  "msg/raw/RawWheelSpeedOutput.msg"
  "msg/raw/RawVehicleSpeedOutput.msg"
  "msg/raw/RawWheelTickOutput.msg"
  "msg/raw/RawVehicleTickOutput.msg"
  "msg/raw/RawGnssAttitudeOutput.msg"
  "msg/septentrio/PVTCartesian.msg"
  "msg/septentrio/PVTGeodetic.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_package()
