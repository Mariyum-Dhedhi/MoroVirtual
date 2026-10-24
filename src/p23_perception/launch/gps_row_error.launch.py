import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    field_config_path = os.path.join(
        get_package_share_directory("p23_bringup"), "config", "field_config.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock",
        ),
        Node(
            package="p23_perception",
            executable="gps_row_error_node",
            name="gps_row_error_node",
            output="screen",
            parameters=[field_config_path],
        ),
    ])
