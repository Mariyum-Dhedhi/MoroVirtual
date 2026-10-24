from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        # teleop_twist_keyboard requires a controlling terminal;
        # run this launch file in a dedicated terminal window.
    )

    return LaunchDescription([teleop])
