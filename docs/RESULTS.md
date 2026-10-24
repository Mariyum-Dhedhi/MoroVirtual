# Results

## Included files

| Output | Description |
|---|---|
| Gazebo screenshot and video | Ground robot and crop-field scene; approximately 40 seconds of recorded simulation |
| RViz screenshot | Robot model, RGB/IR camera panels and LiDAR visualization |
| Planned-route animation | Interpolation of the 712 saved waypoints; approximately 3.21 km of planned path |
| Coverage map | Saved UAV scan output for an 80-plot area |
| Crop surface model | Saved height-map output from accumulated LiDAR data |

The project presentation reports 98.5% UAV coverage. The source point cloud is not included, so this percentage has not been reproduced from raw measurements here. The ground-robot analysis tools are included, but no complete measured navigation-accuracy report is available.

## Coverage calculation

`src/p23_uav/scripts/generate_coverage_map.py` rotates points into a field-aligned frame, crops them to the configured area and bins them into 1 m cells. Coverage is the fraction of cells with at least one return. The field mask, cell size and hit threshold affect the result; the percentage does not measure height-map accuracy.

## Recording a navigation run

Start the logger before navigation. Keep the trace CSV, reached-event CSV and matching row-centerline geometry. Run the analyzer and report waypoint error, localization error and cross-track error with counts and exclusions. Record the simulator version, scene, datum and controller settings with the run. Commands are in [RUNNING.md](RUNNING.md).

## Build checks

All five ROS packages configured, compiled and installed in a clean CMake build on 24 October 2026. The installed waypoint resource, utility syntax, centerline export, media decoding and documentation links were checked. Full colcon validation of this packaged workspace and a fresh live run remain pending.
