## ConeSTELLATION SLAM I/O and TF behavior — Full source review (2025-08-24)

Scope: Reviewed all code under `cone_stellation/` (C++ `src/`, headers in `include/`, Python in `scripts/`). Findings below are code-grounded, not just docs.

### Inputs (Subscriptions and handling)
- **/cone/fused/ukf [custom_interface/msg/TrackedConeArray]**
  - Subscribed in `cone_slam_node.cpp` with BestEffort/Volatile QoS (depth=10).
  - Parsed via `ros_utils::from_ros_msg(...)` which:
    - Copies 2D positions (x,y) only, builds simple distance-based covariance, parses color string to enum.
  - Frame handling in `cone_callback(...)`:
    - Requires `last_odom_` to exist; otherwise returns early.
    - Looks up `base_link <- <sensor_frame>` TF using `tf_buffer_.lookupTransform("base_link", msg->header.frame_id, TimePointZero)`.
    - Transforms each cone from sensor frame into `base_link`; keeps observations in `base_link`-relative coordinates for the factor graph.
    - Sets current sensor pose from `last_odom_` (effectively `T_odom_base` copied into `sensor_pose`).
    - Preprocesses observations, then conditionally creates a keyframe and feeds `ConeMapping`.

- **/odometry/filtered [nav_msgs/msg/Odometry]**
  - Subscribed in `cone_slam_node.cpp` with BestEffort/Volatile QoS (depth≈100 intent).
  - Stored as `last_odom_`; also converted to `Eigen::Isometry3d T_odom_base`.
  - NOTE: Calls to drift manager inside `odom_callback` are commented out (see Drift section).

### Outputs (Topics and TF)
- Topics published by SLAM node:
  - **/slam/pose [geometry_msgs/PoseStamped]**: Latest optimized pose in `map` frame (from `gtsam::Pose2`).
  - **/slam/odometry [nav_msgs/Odometry]**: Odometry-like message with `header.frame_id = "odom"`, `child_frame_id = "base_link"`. No corresponding TF is sent (explicitly disabled to avoid conflict with EKF).
  - Visualization markers and keyframes via `SLAMVisualizer` helpers (frame `map`).

- TF published by SLAM node:
  - **map -> odom**: Published as identity.
    - Once at startup and then continuously by a 100 ms wall timer in the node constructor.
    - Purpose: keep TF tree complete while drift correction is disabled.
  - **map -> base_link_slam**: Published in `visualization_callback()` using the most recent optimized pose (child frame `base_link_slam`). For visualization/debug only.
  - ⚠️ **odom -> base_link**: Not published by SLAM; expected from EKF via `/odometry/filtered`.

### Drift correction (map -> odom) status
- `include/.../drift_correction_manager.hpp` implements a GLIM-style drift manager:
  - Buffers `T_odom_base(t)`; on SLAM update with `T_map_base(t)`, computes `T_map_odom = T_map_base * T_odom_base^-1` with interpolation.
- Current node wiring:
  - In `odom_callback(...)`: calls to `drift_manager_->add_odometry_pose(...)` are commented out.
  - In `visualization_callback(...)`: calls to update/query drift and publish `map->odom` are commented out.
  - Instead, the node sends identity `map->odom` at 10 Hz via a timer.
- Rationale (from code comments and debug logs): prevent circular dependency where `T_map_base` implicitly already included past drift, causing feedback when recomputing `T_map_odom`.

### QoS and timing
- Subscriptions: BestEffort + Volatile for both cones and odometry; cones depth=10, odom intended high-rate (100 Hz class) with depth≈100.
- Publishers: BestEffort + Volatile.
- Timers: 100 ms visualization timer; 100 ms identity `map->odom` TF timer.

### Related Python (sim/testing) notes
- `scripts/dummy_publisher_node.py` can publish `map->odom` (identity) when `publish_map_to_odom=True` for standalone testing. When running SLAM, this must be set to False to avoid TF conflicts with the SLAM node’s own `map->odom` identity publisher.
- The dummy also publishes simulated odom and GPS topics for EKF tests; SLAM does not consume those TFs directly beyond `/odometry/filtered`.

### Verdict (as-is behavior)
- SLAM ingests cones from `/cone/fused/ukf`, converts to `base_link`-relative, builds/optimizes the map, and publishes visualization plus `/slam/pose` and `/slam/odometry` topics.
- The node currently publishes an identity `map->odom` at 10 Hz and does not compute drift-corrected `map->odom` from SLAM. All drift-manager hooks are present but disabled in code.
- `odom->base_link` is provided by EKF; SLAM does not publish it.

### Where in code
- `src/cone_stellation/ros/cone_slam_node.cpp`: subscriptions, TF broadcasters, identity `map->odom` timer, `map->base_link_slam` broadcast, disabled drift hooks.
- `include/cone_stellation/util/drift_correction_manager.hpp`: drift logic implementation.
- `include/cone_stellation/util/ros_utils.hpp`: `TrackedConeArray` conversion and visualization helpers.
- `scripts/dummy_publisher_node.py`: optional TF and topic publishers for simulation/testing.
