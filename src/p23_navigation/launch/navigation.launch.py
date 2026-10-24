from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_nav2 = FindPackageShare("nav2_bringup")
    pkg_p23_nav = FindPackageShare("p23_navigation")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    slam = LaunchConfiguration("slam", default="true")
    map_yaml = LaunchConfiguration("map", default="")
    params_file = LaunchConfiguration(
        "params_file",
        default=PathJoinSubstitution([pkg_p23_nav, "config", "nav2_params.yaml"]),
    )

    declare_use_sim = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock",
    )
    declare_slam = DeclareLaunchArgument(
        "slam",
        default_value="true",
        description="Run SLAM (true) or localization only (false)",
    )
    declare_map = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Path to map YAML when slam:=false",
    )
    declare_params = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([pkg_p23_nav, "config", "nav2_params.yaml"]),
        description="Path to Nav2 parameter file",
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav2, "launch", "bringup_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam": slam,
            "map": map_yaml,
            "params_file": params_file,
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim,
        declare_slam,
        declare_map,
        declare_params,
        nav2_bringup,
    ])
