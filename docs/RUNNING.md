# Running MoroVirtual

## 1. Install simulation dependencies

Use Ubuntu 22.04 with ROS 2 Humble. The inspected code uses Fortress plugin names and `ignition.msgs` bridge types. Official references: [Fortress installation](https://gazebosim.org/docs/fortress/install_ubuntu/) and [Humble gz_ros2_control](https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html).

On the host terminal, with ROS apt repositories already configured:

```bash
sudo apt update
sudo apt install ros-humble-ros-gz ros-humble-gz-ros2-control \
  ros-humble-ros2-controllers ros-humble-xacro \
  ros-humble-robot-localization ros-humble-tf-transformations \
  ros-humble-joint-state-publisher ros-humble-rviz2 \
  python3-transforms3d python3-pandas python3-numpy python3-matplotlib \
  python3-pil python3-opencv python3-rosdep
```

Verify `ign gazebo --help`. If `ign` is still missing, follow the official Fortress installation above. Do not substitute Gazebo Classic or a different simulator generation without adapting the plugins, bridge types, and resources.

```bash
cd ~/MoroVirtual
source /opt/ros/humble/setup.bash
python3 scripts/check_environment.py
# Run sudo rosdep init only if rosdep has never been initialized on this host.
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install
source install/setup.bash
```

Optional Nav2 dependencies are declared in the navigation manifest even though the custom follower does not use Nav2. GIS regeneration additionally needs GeoPandas, Rasterio, Shapely, PyYAML, and the local source dataset; inspect each GIS script's CLI before regenerating outputs.

## 2. Start the ground robot

In every new terminal:

```bash
cd ~/MoroVirtual
source /opt/ros/humble/setup.bash
source install/setup.bash
```

**Terminal 1 — simulator and robot:**

```bash
ros2 launch p23_bringup sim.launch.py
```

This launches the default `soil.sdf`, robot state publisher, sensor bridge, delayed robot spawn (3 seconds), controller spawners (6 seconds), and steering node (8 seconds). Wait for controller activation messages. Startup uses fixed delays rather than readiness checks, so slow systems may need further tuning.

**Terminal 2 — localization:**

```bash
ros2 launch p23_bringup ekf.launch.py
```

**Terminal 3 — record accuracy before starting navigation:**

```bash
mkdir -p results
ros2 launch p23_navigation accuracy_logger.launch.py \
  output_csv:="$PWD/results/accuracy_log.csv" \
  reached_events_csv:="$PWD/results/reached_events.csv"
```

**Terminal 4 — follow saved waypoints:**

```bash
ros2 launch p23_navigation waypoint_follower.launch.py
```

The saved route is installed into the navigation package. To override it, pass `waypoints_file:=/absolute/path/to/waypoints.csv` or set `P23_WAYPOINTS_FILE`. Required columns are `index,x,y,point_type`; extra columns are allowed.

**Optional visualization:**

```bash
rviz2 -d src/p23_robot_description/rviz/sensors.rviz
```

Check `ros2 control list_controllers`, `ros2 topic hz /clock`, `ros2 topic hz /odometry/filtered`, and `ros2 topic echo /waypoint_reached`. Use Ctrl+C to end each process; end navigation first. To inspect stationary sensors, leave the follower stopped.

## 3. Analyze a completed run

Stop the logger after navigation finishes so its CSVs are closed. The analysis script requires `row_id,x1,y1,x2,y2` centerline CSV input; the supplied geometry is JSON, so first export it:

```bash
python3 scripts/export_centerlines.py \
  src/Viz_shapefiles/outputs/row_centerlines.json results/row_centerlines.csv
python3 src/p23_navigation/scripts/analyze_accuracy.py \
  --accuracy-log results/accuracy_log.csv \
  --reached-events results/reached_events.csv \
  --row-centerlines results/row_centerlines.csv \
  --output results/summary_stats.csv
```

Verify the centerlines and robot ground truth use the same coordinates and datum. Do not interpret a route preview as measured accuracy. Empty or incomplete runs may not produce meaningful statistics.

## 4. UAV workflow is separate

`p23_uav/scripts/field_coverage_flight_tight.py` imports `px4_msgs` and expects PX4 offboard topics. It requires a compatible PX4 simulator, vehicle, DDS transport, matching message definitions, and LiDAR data. These are not provisioned by the ground-robot launch. `field_map_accumulator.py` builds a field cloud; the coverage and height generators read `/tmp/field_map.npy`. Some plotting scripts retain original absolute paths. The shipped UAV plots can be viewed without launching that stack.

## Troubleshooting

| Symptom | Likely cause / next check |
|---|---|
| `Package 'ros_gz_sim' not found` | Install simulation dependencies and source Humble |
| `xacro` or control plugin missing | Run the environment checker; verify the Humble plugin package |
| Robot exists but does not move | Check controller activation, `/cmd_vel`, and `/odometry/filtered` |
| No map odometry | Inspect GNSS, IMU, `/odometry/local`, `/odometry/gps`, and TF |
| Missing waypoint file | Rebuild navigation after the install rule change or pass an absolute CSV path |
| World assets absent | Confirm installed model files and `IGN_GAZEBO_RESOURCE_PATH` from the world launch |
| Notebook launches unrelated demo | Use the ROS launch files; Binder assets were inherited from another lab |
