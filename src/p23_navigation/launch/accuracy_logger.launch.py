from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    output_csv = LaunchConfiguration("output_csv", default="accuracy_log.csv")
    reached_events_csv = LaunchConfiguration(
        "reached_events_csv", default="reached_events.csv")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        ),
        DeclareLaunchArgument(
            "output_csv",
            default_value="accuracy_log.csv",
            description="Path to the 10 Hz accuracy trace CSV",
        ),
        DeclareLaunchArgument(
            "reached_events_csv",
            default_value="reached_events.csv",
            description="Path to the per-waypoint-reached-event CSV",
        ),
        Node(
            package="p23_navigation",
            executable="accuracy_logger.py",
            name="accuracy_logger",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "output_csv": output_csv,
                "reached_events_csv": reached_events_csv,
            }],
        ),
    ])
