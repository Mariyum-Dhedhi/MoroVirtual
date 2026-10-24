"""Standalone RViz launch for sensor visualisation.

Run AFTER  ros2 launch p23_bringup sim.launch.py  is fully up.
"""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("p23_robot_description"), "rviz", "sensors.rviz"]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_cfg],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([rviz])
