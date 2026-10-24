import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

#This is the kalman filter launch file >>>>>> Launch number in order 2

def generate_launch_description():
    pkg = get_package_share_directory('p23_bringup')

    ekf_config = os.path.join(pkg, 'config', 'ekf_odom.yaml')
    navsat_config = os.path.join(pkg, 'config', 'navsat_transform.yaml')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', '/odometry/local'),
        ],
    )

    gnss_covariance_fix_node = Node(
        package='p23_bringup',
        executable='gnss_covariance_fix.py',
        name='gnss_covariance_fix',
        output='screen',
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_config],
        remappings=[
            ('imu',               '/imu/data'),
            ('gps/fix',           '/gnss/front/fix_cov'),
            ('odometry/filtered', '/odometry/local'),
        ],
    )

    map_frame_publisher_node = Node(
        package='p23_bringup',
        executable='map_frame_publisher.py',
        name='map_frame_publisher',
        output='screen',
    )

    return LaunchDescription([ekf_node, gnss_covariance_fix_node, navsat_transform_node, map_frame_publisher_node])
