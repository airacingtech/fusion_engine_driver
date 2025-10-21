cmake_minimum_required(VERSION 3.8)
project(fusion_engine_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

# Disable example building for fusion engine client
set(P1_FE_BUILD_EXAMPLES OFF)

# ------------------------------
# Fetch content
# ------------------------------
include(FetchContent)

FetchContent_Declare(
  fusion_engine_client
  GIT_REPOSITORY git@github.com:airacingtech/fusion-engine-client.git
  GIT_TAG origin/master
)

FetchContent_Declare(
  septentrio_gnss_upstream
  GIT_REPOSITORY https://github.com/septentrio-gnss/septentrio_gnss_driver.git
  GIT_TAG 209fccea85e76bbccc71e3599daad53decb21af4
)

FetchContent_Populate(fusion_engine_client)
FetchContent_Populate(septentrio_gnss_upstream)

add_subdirectory(${fusion_engine_client_SOURCE_DIR} ${fusion_engine_client_BINARY_DIR})

# ------------------------------
# Dependencies needed BEFORE rosidl_generate_interfaces()
# ------------------------------
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Other deps for your driver
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(fusion_engine_msgs REQUIRED)

include_directories(./core ./utils ./transport ./communication)

# ------------------------------
# Build your executable
# ------------------------------
set(SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/core/main.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/core/fusion_engine_node.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/core/fusion_engine_interface.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/transport/serial_port.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/transport/tcp_listener.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/transport/tty_listener.cc
)

add_executable(fusion_engine_ros_driver ${SRC})
target_link_libraries(fusion_engine_ros_driver fusion_engine_client)

ament_target_dependencies(fusion_engine_ros_driver
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
)

# ------------------------------
# Generate Septentrio messages ONLY (no driver)
# ------------------------------
set(SEPTENTRIO_MSG_PKG septentrio_gnss_msgs)  # name of the generated interface "package"
set(SEPT_MSGS_DIR ${septentrio_gnss_upstream_SOURCE_DIR}/msg)

# Collect .msg files relative to msg/ and prefix as <abs_base>:msg/<file>
file(GLOB SEPT_MSG_FILES RELATIVE "${SEPT_MSGS_DIR}" "${SEPT_MSGS_DIR}/*.msg")
list(TRANSFORM SEPT_MSG_FILES PREPEND "${septentrio_gnss_upstream_SOURCE_DIR}:msg/")


# Generate the interfaces
rosidl_generate_interfaces(${SEPTENTRIO_MSG_PKG}
  ${SEPT_MSG_FILES}
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

# Make generated headers visible to THIS target during compile
target_include_directories(fusion_engine_ros_driver PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# Ensure build order by linking the typesupport target (brings proper deps)
rosidl_get_typesupport_target(cpp_typesupport_target ${SEPTENTRIO_MSG_PKG} rosidl_typesupport_cpp)
if(cpp_typesupport_target)
  target_link_libraries(fusion_engine_ros_driver ${cpp_typesupport_target})
endif()

# ------------------------------
# Install
# ------------------------------
install(TARGETS fusion_engine_ros_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY params launch
  DESTINATION share/${PROJECT_NAME}/
)

# Optional: install generated headers so others *could* include them from install/
# (not strictly required for your use; uncomment if you want them installed)
# install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/${SEPTENTRIO_MSG_PKG}
#   DESTINATION include
# )

# ------------------------------
# Testing + exports
# ------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
