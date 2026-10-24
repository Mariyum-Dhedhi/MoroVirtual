# PhenoRob Field — Gazebo World (from real UAV survey data)

Generated from your 2023-05-17 UAV DEM + orthophoto (full field, ~174.6m x 63.9m,
UTM32N) and the `PhenorobField_2023_lla` plot shapefile.

## What's in here

```
worlds/phenorob_field.sdf              # the world: real terrain + real GPS anchor
models/field_terrain/
  model.sdf, model.config
  meshes/field_terrain.obj (+ .mtl)            # visual mesh: ~44.7k verts / 88.4k tris, textured
  meshes/field_terrain_collision.obj           # collision mesh: ~2.8k verts / 5.3k tris, untextured
  materials/textures/field_ortho.jpg           # real RGB orthophoto draped as texture
scripts/example_navsat_sensor_snippet.sdf   # GPS sensor block to drop into your robot's SDF
waypoints_all_242_plots.csv            # every shapefile plot: lat/lon + world XY
waypoints_80_in_flight_extent.csv      # only the 80 plots that fall inside this DEM/ortho's extent
```

## Terrain

- Built directly from your DEM: real elevation, relative relief ~9.2m across
  the field, resampled to ~0.5m vertex spacing (~349x128 grid) — a good
  balance of geometric fidelity vs. real-time sim performance. Increase/decrease
  `target_spacing` in the generation script if you want more/less detail.
- Textured with your actual orthophoto (resized to 4096px on the long edge, JPG),
  UV-mapped 1:1 to the mesh, so the simulated ground visually matches the real field.
- A few DEM nodata gaps (~27% of pixels were nodata, likely gaps near the field
  edges or occluded areas) were filled by nearest-valid-pixel before meshing —
  fine for driving/visual realism, but don't treat those areas as survey-accurate
  if you need precise elevation there.

## GPS

- `worlds/phenorob_field.sdf` sets `<spherical_coordinates>` to the field's real
  SW-corner lat/lon (50.6258°N, 6.9837°E), so world (0,0,0) = that real GPS point,
  and +X/+Y align with UTM easting/northing (true PhenoRob field near Campus
  Klein-Altendorf, Bonn).
- Drop `scripts/example_navsat_sensor_snippet.sdf` into your robot's SDF/URDF to
  get a working `navsat`/GPS sensor. Because of the anchor above, it will report
  real, meaningful lat/lon for this field, not arbitrary sim coordinates.
- Tune the Gaussian noise stddevs in that snippet to match whatever receiver
  you're emulating (consumer GPS ~1-3m, RTK ~1-2cm).
- The waypoint CSVs give you the real lat/lon **and** matching world-frame XY
  for every plot, so you can task the robot with real GPS goals (e.g. "drive to
  plot 137") and verify it converges in both GPS and world coordinates.

## Requirements assumed

- Modern Gazebo (`gz sim`, Harmonic or later) — the world uses `gz-sim-*` system
  plugins and SDF 1.9. For classic Gazebo you'd swap the plugins for `gazebo_ros`
  equivalents and use `hector_gazebo_ros_gps` for GPS; ask if you want that version instead.
- ROS 2 + `ros_gz_bridge` if you want the GPS topic in ROS (bridging command is in
  the snippet file).

## Running it

```bash
gz sim worlds/phenorob_field.sdf
```

The world now includes a placeholder 4-wheel rover (`models/simple_rover`) spawned
at (15, 15, 1.0) near the field's SW corner, with:
- Real physics: 8kg chassis + four 1kg wheels, correct inertia tensors, ODE/dartsim
  friction on the wheel collisions, gravity applying to it like any dynamic body.
- A skid-steer `DiffDrive` plugin listening on `/rover/cmd_vel` (`geometry_msgs/Twist`
  if bridged to ROS 2) and publishing odometry on `/rover/odometry`.
- The same GPS/NavSat sensor from `scripts/example_navsat_sensor_snippet.sdf`,
  publishing on `/rover/gps`, plus an IMU on `/rover/imu`.
- A ground-truth pose publisher, useful for sanity-checking GPS/odometry drift
  against the real simulated position.

Drive it manually once the world is running:

```bash
gz topic -t /rover/cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.5}, angular: {z: 0.2}'
```

Or bridge it into ROS 2 with `ros_gz_bridge` and drive it from `teleop_twist_keyboard`,
Nav2, or your own controller. Swap the `<include><uri>model://simple_rover</uri>`
block in `worlds/phenorob_field.sdf` for your own robot whenever it's ready — the
GPS/IMU sensor wiring in `scripts/example_navsat_sensor_snippet.sdf` is written to
be copy-pasted into any robot's SDF/URDF.

## Notes / caveats

- Only 80 of the shapefile's 242 plots fall inside this particular flight's
  imaged extent — the shapefile covers the broader trial field, of which your
  DEM/ortho is a sub-section. Both CSVs are included; use the 80-plot one if
  you want waypoints guaranteed to sit on textured, elevation-accurate ground.
- Collision uses a separate, coarser mesh (~2m vertex spacing, untextured) from
  the detailed visual mesh (~0.5m spacing, textured). This is standard practice —
  physics engines choke on dense, high-poly collision meshes — and is plenty
  for wheeled/legged ground robots. If your robot needs finer terrain-following
  (e.g. very small wheels), regenerate the collision mesh at a smaller spacing.
