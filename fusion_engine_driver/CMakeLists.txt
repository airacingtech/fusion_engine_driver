cmake_minimum_required(VERSION 3.8)
project(fusion_engine_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
set(P1_FE_BUILD_EXAMPLES OFF)
include(FetchContent)

FetchContent_Declare(
  fusion_engine_client
  GIT_REPOSITORY git@github.com:airacingtech/fusion-engine-client.git
  GIT_TAG origin/master
)

FetchContent_Populate(fusion_engine_client)

add_subdirectory(${fusion_engine_client_SOURCE_DIR} ${fusion_engine_client_BINARY_DIR})

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(fusion_engine_msgs REQUIRED)
find_package(race_msgs REQUIRED)

find_library(PCAP_LIBRARY pcap)
include_directories(./core ./utils ./transport ./communication)

set(SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/core/main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/core/fusion_engine_node.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/core/fusion_engine_interface.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/transport/serial_port.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/transport/tcp_listener.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/transport/udp_listener.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/transport/tty_listener.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/transport/pcap_listener.cpp
)

add_library(${PROJECT_NAME}_node SHARED ${SRC})
target_link_libraries(${PROJECT_NAME}_node fusion_engine_client ${PCAP_LIBRARY})

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_components
  std_msgs
  gps_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  mavros_msgs
  nmea_msgs
  fusion_engine_msgs
  race_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "FusionEngineNode"
  EXECUTABLE fusion_engine_ros_driver
)

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY params launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_clock_sync test/test_clock_sync.cpp)
  target_include_directories(test_clock_sync PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/utils)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
