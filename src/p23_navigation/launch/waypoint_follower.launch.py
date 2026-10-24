import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_waypoints_file = os.environ.get(
        "P23_WAYPOINTS_FILE",
        os.path.join(get_package_share_directory("p23_navigation"), "config", "waypoints.csv"),
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    waypoints_file = LaunchConfiguration("waypoints_file", default=default_waypoints_file)

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        ),
        DeclareLaunchArgument(
            "waypoints_file",
            default_value=default_waypoints_file,
            description="Path to the waypoints CSV (requires index, x, y, point_type)",
        ),
        Node(
            package="p23_navigation",
            executable="waypoint_follower.py",
            name="waypoint_follower",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "waypoints_file": waypoints_file,
            }],
        ),
    ])
