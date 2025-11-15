from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    directory = get_package_share_directory("fusion_engine_driver")

    param_file_path = os.path.join(directory, "params", "fusion_engine_driver.param.yaml")

    fusion_node = Node(
        package="fusion_engine_driver",
        executable="fusion_engine_ros_driver",
        name="fusion_engine_node",
        output="screen",
        arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[param_file_path],
        remappings=[
            # Navigation Solutions (FusionEngine)
            ("pose_filtered", "/atlas/pose_filtered"),
            ("pose_aux", "/atlas/pose_aux"),
            ("calibration_status", "/atlas/calibration_status"),
            ("gnss_info", "/atlas/gnss_info"),
            ("gnss_satellite", "/atlas/gnss_satellite"),
            ("relative_enu", "/atlas/relative_enu"),
            # Calibrated Sensor Outputs
            ("imu_calibrated", "/atlas/imu_calibrated"),
            ("gnss_attitude", "/atlas/gnss_attitude"),
            ("wheel_speed", "/atlas/wheel_speed"),
            ("vehicle_speed", "/atlas/vehicle_speed"),
            # Raw Sensor Outputs
            ("imu_raw", "/atlas/imu_raw"),
            ("gnss_attitude_raw", "/atlas/gnss_attitude_raw"),
            # ROS Wrappers
            ("pose_ros", "/atlas/pose_ros"),
            ("gpsfix_ros", "/atlas/gpsfix_ros"),
            ("imu_ros", "/atlas/imu_ros"),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(fusion_node)
    return ld
