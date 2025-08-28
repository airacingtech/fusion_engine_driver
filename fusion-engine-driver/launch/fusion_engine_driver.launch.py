from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    directory = get_package_share_directory('fusion-engine-driver')

    param_file_path = os.path.join(
        directory,
        'params',
        'fusion_engine_driver.param.yaml'
    )
    
    fusion_node = Node(
        package='fusion-engine-driver',
        executable='fusion_engine_ros_driver',
	name='fusion_engine_node',
        output='screen',
        parameters=[param_file_path],
        remappings=[
            ('nav_sat_fix', '/atlas/fix'),
            ('gps_fix', '/atlas/gps'),
            ('imu', '/atlas/imu'),
            ('pose', '/atlas/pose'),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(fusion_node)
    return ld
