#!/usr/bin/env python3
"""Read-only checks; run after sourcing /opt/ros/humble/setup.bash."""
import os
from pathlib import Path
import shutil
import subprocess
import sys

print('ROS distribution:', os.environ.get('ROS_DISTRO', '(not sourced)'))
missing = []
for command in ['ros2', 'colcon', 'ign']:
    found = shutil.which(command)
    print(f'{command}: {found or "MISSING"}')
    if not found:
        missing.append(command)
packages = ['xacro', 'ros_gz_sim', 'ros_gz_bridge', 'gz_ros2_control',
            'controller_manager', 'position_controllers', 'velocity_controllers',
            'joint_state_broadcaster', 'robot_localization', 'tf_transformations',
            'robot_state_publisher']
if shutil.which('ros2'):
    result = subprocess.run(['ros2', 'pkg', 'list'], capture_output=True, text=True)
    if result.returncode:
        print(result.stderr)
        sys.exit(result.returncode)
    available = set(result.stdout.split())
    for name in packages:
        found = name in available
        print(f'{name}: {"OK" if found else "MISSING"}')
        if not found:
            missing.append(name)
root = Path(__file__).resolve().parents[1]
print('Waypoint CSV:', root / 'src/Viz_shapefiles/outputs/waypoints.csv')
print('Ready for dependency checks' if not missing else 'Missing: ' + ', '.join(missing))
sys.exit(bool(missing))
