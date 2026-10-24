from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Perception pipeline launch — add nodes here as they are implemented."""
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    declare_use_sim = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock",
    )

    # TODO: Add camera / lidar / sensor-fusion nodes once implemented.

    return LaunchDescription([
        declare_use_sim,
    ])
