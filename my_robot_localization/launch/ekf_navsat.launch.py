import os
from glob import glob
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        parameters=[os.path.join(get_package_share_directory("my_robot_localization"), 'config/config', 'ekf.yaml')],
        remappings=[('/odometry/filtered', '/odometry/filtered')],
    )
    ld.add_action(ekf_node)

    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        parameters=[os.path.join(get_package_share_directory("my_robot_localization"), 'config/config', 'navsat_transform_node.yaml')],
        remappings=[('gps/fix', '/gps/data_fixed'), ('/imu', '/imu/data')]
    )
    ld.add_action(navsat_node)

    return ld
