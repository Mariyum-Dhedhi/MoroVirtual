#!/bin/bash
# Usage: source this script before every new launch session
# Then:
#   Terminal 1: ros2 launch p23_bringup sim.launch.py
#   Terminal 2: ros2 launch p23_bringup ekf.launch.py
#   Terminal 3: ros2 launch p23_navigation waypoint_follower.launch.py
# Never launch ekf.launch.py more than once per session.

# 1. Kill all ROS2 launch processes
pkill -9 -f "ros2 launch" 2>/dev/null

pkill -9 -f "ign gazebo" 2>/dev/null

# 2. Kill specific node processes by name
pkill -9 -f "ekf_filter_node" 2>/dev/null
pkill -9 -f "navsat_transform" 2>/dev/null
pkill -9 -f "map_frame_publisher" 2>/dev/null
pkill -9 -f "gnss_covariance_fix" 2>/dev/null
pkill -9 -f "dual_gnss_heading" 2>/dev/null
pkill -9 -f "waypoint_follower" 2>/dev/null

# 3. Verify all processes are actually gone
echo "Waiting for all nodes to die..."
for name in "ign gazebo" ekf_filter_node navsat_transform \
            map_frame_publisher gnss_covariance_fix dual_gnss_heading \
            waypoint_follower "ros2 launch"; do
  while pgrep -f "$name" > /dev/null 2>&1; do
    echo "  still waiting for $name..."
    sleep 1
  done
done
echo "All processes confirmed dead."

# 4. Stop and restart the ROS2 daemon
ros2 daemon stop 2>/dev/null
sleep 2
ros2 daemon start

# 5. Verify nothing is left
echo "Remaining ROS2 nodes:"
ros2 node list 2>/dev/null || echo "No nodes running (clean)"
