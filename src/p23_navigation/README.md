# p23_navigation

Navigation stack for Project P23, built on Nav2. Supports SLAM and pre-built-map localization modes.

## Directory Layout

```
p23_navigation/
├── config/
│   └── nav2_params.yaml    # Nav2 parameters (planners, controllers, costmaps, SLAM)
└── launch/
    └── navigation.launch.py  # Launch Nav2 bringup with configurable SLAM/localization
```

## Usage

```bash
# SLAM mode (default) with simulation clock
ros2 launch p23_navigation navigation.launch.py

# Localization only with a pre-built map
ros2 launch p23_navigation navigation.launch.py slam:=false map:=/path/to/map.yaml

# Physical robot (disable sim clock)
ros2 launch p23_navigation navigation.launch.py use_sim_time:=false slam:=false map:=/path/to/map.yaml
```

## Notes

- Tune `nav2_params.yaml` once the robot footprint (`footprint` under costmap params), sensor topics, and odometry are confirmed.
- Saved maps from SLAM can be stored under a `maps/` directory added to this package.
