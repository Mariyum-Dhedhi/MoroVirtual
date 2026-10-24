# p23_perception

Perception package for Project P23. Will host nodes for camera processing, lidar point-cloud processing, and sensor fusion.

## Directory Layout

```
p23_perception/
├── src/                      # C++ node source files (add when implementing nodes)
├── include/p23_perception/   # C++ headers
├── config/                   # Per-sensor and pipeline parameters
└── launch/
    └── perception.launch.py  # Launch the full perception pipeline
```

## Usage

```bash
ros2 launch p23_perception perception.launch.py

# Physical robot (real sensor clock)
ros2 launch p23_perception perception.launch.py use_sim_time:=false
```

## Planned Nodes

| Node | Topic(s) | Description |
|------|----------|-------------|
| `camera_processor` | `/camera/image_raw` → `/camera/processed` | Image pre-processing |
| `lidar_filter` | `/scan` → `/scan/filtered` | Point-cloud filtering |

## Notes

- Add C++ node sources under `src/` and uncomment the `add_executable` block in `CMakeLists.txt`.
- Store per-node YAML parameter files under `config/`.
