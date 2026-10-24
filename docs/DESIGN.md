# Design notes

## Current setup

| Choice | Reason it is useful | Tradeoff / validation need |
|---|---|---|
| ROS 2 packages and explicit launch files | Separates robot model, environment, localization, control and logging | Topic, TF, QoS and startup assumptions must agree across packages |
| Georeferenced field geometry | Route points originate from actual plot layout | UTM, local datum, field rotation and simulator frames must be checked |
| Separate steering and drive controllers | Exposes four-wheel steering kinematics through standard joint interfaces | Wheel slip and steering transients can affect geometric tracking |
| GNSS/IMU localization | Provides a practical outdoor-navigation baseline | Does not eliminate sensor bias, datum error or startup sensitivity |
| Proportional waypoint follower | Small, understandable baseline for target-seeking experiments | No obstacle avoidance; constant-speed tracking and discrete targets can cause overshoot |
| Simulator ground-truth logging | Separates pose-estimation error from path-tracking error | Ground-truth and estimated poses must share coordinates and timing |
| Grid-based aerial coverage | Easy to inspect scan completeness spatially | Sensitive to grid size, crop mask, minimum-hit threshold and point density |

## Next experiments

These are proposed comparisons, not completed implementations:

1. **Control:** compare the current waypoint controller against Stanley or pure pursuit on identical routes. Log cross-track error, endpoint error, completion rate and time. Sweep speed and look-ahead/gain parameters.
2. **Localization:** vary GNSS noise and update rate; compare IMU-assisted estimates, optional dual-GNSS heading, and LiDAR/RTK refinement. Record error distributions against ground truth.
3. **Environment fidelity:** compare the selected default scene with denser reconstructed plots. Measure simulation speed, memory use, sensor realism and whether controller behavior changes.
4. **Coverage:** rerun aerial processing with multiple grid sizes and minimum point counts. Report the evaluation mask alongside the percentage so the metric is reproducible.
5. **Workflow:** replace fixed launch delays with readiness-based startup; add a stale-odometry timeout before increasing autonomy.
