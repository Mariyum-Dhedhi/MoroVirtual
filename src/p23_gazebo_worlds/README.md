# p23_gazebo_worlds

Gazebo simulation worlds for Project P23. Contains SDF world files and custom model assets.

## Directory Layout

```
p23_gazebo_worlds/
├── worlds/
│   └── empty.world       # Baseline empty world with ground plane and sun
├── models/               # Custom SDF model assets
└── launch/
    └── gazebo.launch.py  # Launch Gazebo with a selectable world
```

## Usage

```bash
# Launch default empty world
ros2 launch p23_gazebo_worlds gazebo.launch.py

# Launch with a custom world file
ros2 launch p23_gazebo_worlds gazebo.launch.py world:=/absolute/path/to/my.world
```

## Notes

- Add new `.world` files under `worlds/`.
- Custom SDF models belong under `models/<model_name>/` following the standard Gazebo model structure.
