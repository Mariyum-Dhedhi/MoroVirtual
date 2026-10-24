# p23_robot_description

Robot description package for Project P23. Contains URDF/Xacro models, meshes, and a display launch file for visualizing the robot in RViz2.

## Directory Layout

```
p23_robot_description/
├── urdf/          # URDF and Xacro model files
├── meshes/        # Visual and collision mesh files (STL/DAE)
├── config/        # RViz configurations, joint limits, etc.
└── launch/
    └── display.launch.py   # Spawn robot in RViz2 with joint_state_publisher_gui
```

## Usage

```bash
ros2 launch p23_robot_description display.launch.py
```

## Notes

- Robot URDF is not yet implemented. Add `p23_robot.urdf.xacro` under `urdf/` when ready.
- `meshes/` should contain sub-directories `visual/` and `collision/`.
