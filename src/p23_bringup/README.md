# p23_bringup

Top-level bringup package for Project P23. Orchestrates launch of the robot description, simulation, navigation, and perception stacks.

## Directory Layout

```
p23_bringup/
├── config/               # Shared parameters used across launch files
└── launch/
    ├── sim.launch.py     # Full simulation bringup (Gazebo + robot description)
    └── robot.launch.py   # Physical robot bringup (hardware + robot description)
```

## Usage

```bash
# Simulation
ros2 launch p23_bringup sim.launch.py

# Physical robot
ros2 launch p23_bringup robot.launch.py
```

## Notes

- `sim.launch.py` and `robot.launch.py` are the two primary entry points for the project.
- Hardware interface nodes should be added to `robot.launch.py` once the driver is ready.
