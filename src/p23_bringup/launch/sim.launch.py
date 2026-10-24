from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# This is the main launch file

def generate_launch_description():
    pkg_desc    = FindPackageShare("p23_robot_description")
    pkg_worlds  = FindPackageShare("p23_gazebo_worlds")
    pkg_bringup = FindPackageShare("p23_bringup")

    world      = LaunchConfiguration("world")
    xacro_file = PathJoinSubstitution([pkg_desc, "urdf", "p23_robot.urdf.xacro"])
    bridge_cfg = PathJoinSubstitution([pkg_bringup, "config", "bridge.yaml"])

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str,
    )

    declare_world = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([pkg_worlds, "worlds", "soil.sdf"]),
        description="Gazebo Fortress SDF world file",
    )

    # ── Ignition Gazebo Fortress ────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_worlds, "launch", "gazebo.launch.py"])
        ),
        launch_arguments={"world": world}.items(),
    )

    # ── robot_state_publisher ──────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
    )

    # ── Spawn robot (3 s delay — wait for Gazebo) ──────────────────────
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_p23_ugv",
        arguments=[
            "-name",  "p23_ugv",
            "-topic", "/robot_description",
            "-x", "18.0", "-y", "0.0", "-z", "0.55",
            "-R", "0.0", "-P", "0.0", "-Y", "0.0",
        ],
        output="screen",
    )

    # ── ros_gz_bridge — clock + all sensor topics ──────────────────────
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[{
            "config_file": bridge_cfg,
            "use_sim_time": True,
        }],
    )

    # ── Controller spawners (6 s — after robot spawns) ─────────────────
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )
    spawn_steer = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["steer_controller",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )
    spawn_drive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["drive_controller",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # ── 4WS kinematic node (8 s — after controllers are active) ────────
    four_ws_drive = Node(
        package="p23_bringup",
        executable="four_ws_drive.py",
        name="four_ws_drive",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # ── Static TF: bridge Ignition scoped sensor frame → URDF link frame ─
    # Ignition publishes LiDAR PointCloud2 with frame_id = p23_ugv/lidar_3d_link/lidar_3d
    # (after preserveFixedJoint stops lumping). RSP publishes lidar_3d_link.
    # Identity transform: the sensor data is already expressed in lidar_3d_link coords.
    lidar_tf_fix = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_tf_fix",
        arguments=[
            "0", "0", "0", "0", "0", "0",
            "lidar_3d_link",
            "p23_ugv/lidar_3d_link/lidar_3d",
        ],
        parameters=[{"use_sim_time": True}],
    )

    # Bridge Ignition's scoped IMU sensor frame to the URDF imu_link frame
    # (after preserveFixedJoint stops lumping). RSP publishes imu_link.
    # Identity transform: the sensor data is already expressed in imu_link coords.
    imu_tf_fix = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_tf_fix",
        arguments=[
            "0", "0", "0", "0", "0", "0",
            "imu_link",
            "p23_ugv/imu_link/imu_sensor",
        ],
        parameters=[{"use_sim_time": True}],
    )

    gnss_tf_fix = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gnss_tf_fix",
        arguments=["0", "0", "0", "0", "0", "0", "gnss_front_link", "p23_ugv/imu_link/gnss_front"],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        declare_world,
        gazebo,
        robot_state_publisher,
        lidar_tf_fix,
        imu_tf_fix,
        gnss_tf_fix,
        bridge,
        TimerAction(period=3.0, actions=[spawn_robot]),
        TimerAction(period=6.0, actions=[spawn_jsb, spawn_steer, spawn_drive]),
        TimerAction(period=8.0, actions=[four_ws_drive]),
    ])
