cmake_minimum_required(VERSION 3.8)
project(fusion_engine_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RawImuOutput.msg"
  "msg/RawWheelSpeedOutput.msg"
  "msg/RawVehicleSpeedOutput.msg"
  "msg/RawWheelTickOutput.msg"
  "msg/RawVehicleTickOutput.msg"
  "msg/RawGnssAttitudeOutput.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_package()
