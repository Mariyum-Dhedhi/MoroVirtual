from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# "It is only for visualizing the robot in RVIZ2"

def generate_launch_description():
    """Bringup for the physical P23 robot."""
    pkg_description = FindPackageShare("p23_robot_description")

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_description, "launch", "display.launch.py"])
        ),
        launch_arguments={"use_gui": "false"}.items(),
    )

    # TODO: Add hardware interface / ros2_control launch here once implemented.

    return LaunchDescription([
        description,
    ])
