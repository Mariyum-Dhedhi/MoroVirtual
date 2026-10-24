# Technical overview

## Field preparation

`src/Viz_shapefiles/` contains processing scripts and prepared field geometry. Orthophotos provide georeferenced images; plot boundaries identify the crop layout. The scripts compute local plot corners and row centerlines, then generate a back-and-forth waypoint route.

Start with `compute_local_plot_corners.py`, `compute_row_centerlines.py` and `generate_boustrophedon_waypoints.py`. The saved `outputs/waypoints.csv` contains route order, local coordinates, plot/row grouping and waypoint type. Datum, orientation and units must match the simulator's coordinates.

## Robot and world

`p23_robot_description/urdf/p23_robot.urdf.xacro` assembles the chassis, wheels, steering joints, sensors and control interfaces. The robot state publisher uses the expanded URDF and joint states to publish TF transforms.

`p23_gazebo_worlds/worlds/soil.sdf` defines the default field scene. `p23_bringup/launch/sim.launch.py` launches Gazebo, spawns the robot, starts the sensor bridge and brings up the controllers. Startup currently uses fixed delays of 3, 6 and 8 seconds for spawning and control. The selected scene does not reconstruct every plot in the dataset.

`p23_bringup/config/bridge.yaml` maps Gazebo topics to ROS messages for the clock, GNSS, IMU, cameras, LiDAR, scanners and ground-truth odometry.

## Localization

`ekf.launch.py` starts four components:

- `gnss_covariance_fix.py` supplies GNSS uncertainty values.
- `robot_localization/ekf_node` estimates 2D pose from GPS position and IMU yaw/yaw rate, publishing `/odometry/local`.
- `navsat_transform_node` converts geographic fixes into local metric odometry using the configured datum.
- `map_frame_publisher.py` combines GPS position with local orientation, publishes `/odometry/filtered` and computes `map → odom`.

The waypoint follower consumes `/odometry/filtered`. This topic comes from the custom map-frame publisher rather than directly from the EKF. Inspect GNSS, IMU, TF and datum alignment when debugging position errors. The dual-GNSS heading script is not enabled by this launch file.

## Waypoint control

`p23_navigation/p23_navigation/waypoint_follower.py` loads and sorts the waypoint CSV. Every 0.2 seconds it computes target distance and normalized heading error.

The robot first rotates until heading error is within 0.05 rad. It then requests 1 m/s forward velocity and applies angular correction with gain 1.5, clamped to ±1 rad/s. A waypoint is considered reached when estimated distance is below 0.3 m. The node requests zero velocity before initial odometry arrives and after the final waypoint.

The arrival threshold is not measured positioning accuracy. The follower has no obstacle avoidance or stale-odometry watchdog.

`p23_bringup/scripts/four_ws_drive.py` converts body velocity into four steering angles and wheel velocities. Position controllers steer the joints; velocity controllers drive the wheels. Joint states feed the robot model and TF.

## Logging and analysis

`accuracy_logger.py` samples ground-truth pose, estimated pose, target and waypoint index at 10 Hz. Reached events are logged separately in their callbacks. The target subscription uses transient-local durability to receive the latest target after a late start.

`analyze_accuracy.py` calculates:

| Metric | Definition |
|---|---|
| Terminal waypoint error | Ground-truth distance to the target when arrival was reported |
| Localization error | Distance between estimated and ground-truth positions |
| Cross-track error | Distance from ground truth to an assigned crop-row segment |

The analyzer reports mean, RMS, maximum error, counts and exclusions. `scripts/export_centerlines.py` converts the supplied JSON geometry to its expected CSV format. Check coordinate alignment and excluded samples before interpreting a run.

## UAV scanning

`field_coverage_flight_tight.py` sends PX4 offboard commands. `field_map_accumulator.py` combines LiDAR returns with vehicle pose. The height and coverage scripts process the resulting `/tmp/field_map.npy` cloud.

This workflow requires an external PX4 simulator, DDS transport, matching `px4_msgs` and sensor setup. It is separate from the ground-robot launch and is not installed as a colcon package. Some scripts retain fixed input/output paths.
