from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("p23_robot_description")

    default_xacro = PathJoinSubstitution([pkg, "urdf", "p23_robot.urdf.xacro"])
    default_rviz  = PathJoinSubstitution([pkg, "config", "display.rviz"])

    xacro_file = LaunchConfiguration("xacro_file")
    rviz_config = LaunchConfiguration("rviz_config")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "xacro_file",
            default_value=default_xacro,
            description="Absolute path to the robot Xacro file",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz,
            description="Absolute path to the RViz2 config file",
        ),

        # Publishes /robot_description and the TF tree (fixed joints only).
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": False,
            }],
        ),

        # GUI slider for the 4 continuous wheel joints.
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        ),

        # RViz2 with the pre-configured display layout.
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])
