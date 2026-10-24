from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_worlds  = FindPackageShare("p23_gazebo_worlds")
    pkg_ros_gz  = FindPackageShare("ros_gz_sim")

    # When URDF is converted to SDF, package:// becomes model://.
    # IGN_GAZEBO_RESOURCE_PATH must include the *parent* of each package's share
    # directory so model://p23_robot_description/meshes/... and
    # model://p23_gazebo_worlds/models/... resolve correctly.
    resource_path = os.pathsep.join([
        os.path.dirname(get_package_share_directory("p23_robot_description")),
        os.path.dirname(get_package_share_directory("p23_gazebo_worlds")),
    ])

    world = LaunchConfiguration("world")

    declare_world = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([pkg_worlds, "worlds", "soil.sdf"]),
        description="Full path to a Gazebo Fortress SDF world file",
    )

    set_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=resource_path,
    )

    # -r  : run simulation immediately (don't pause at startup)
    # -v 3: informative verbosity without log flooding
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": [world, " -r -v 3"]}.items(),
    )

    return LaunchDescription([
        declare_world,
        set_resource_path,
        gazebo,
    ])
