# Husky Simulation — Localization Benchmarking & Pure Pursuit Testbed

Simulation environment built on top of the [Clearpath Husky](https://github.com/husky/husky) robot platform. The Husky model is used purely as a simulation testbed — it provides a well-tuned differential-drive robot with realistic physics properties (mass, inertia, friction, sensor noise) so we can focus on developing and validating our own algorithms rather than modelling a robot from scratch.

The project hosts two workflows:

- **Pose-estimation / sensor-fusion benchmarking (current focus)** — compares wheel odometry, IMU dead reckoning, and a dual EKF (local odom-frame + global map-frame) against Gazebo ground truth, with selectable IMU noise profiles. This is the foundation for the ongoing adaptive sensor-fusion research.
- **Pure Pursuit GPS path-following controller** — a path follower driven by a GPS-derived pose. Note: it was written for a *dual-GPS antenna* setup; the current simulation publishes a *single* GPS (`/navsat/fix`), so the pure_pursuit GPS pipeline needs remapping or the dual antennas re-enabled to run (see the pure_pursuit section).

## Tested Environment

| Component | Version |
|-----------|---------|
| ROS       | Noetic  |
| Ubuntu    | 20.04   |
| Gazebo    | Default bundled with ROS Noetic |

---

## Project Structure

```
husky/
├── husky_description/      Robot URDF model, meshes, and sensor definitions
├── husky_gazebo/            Gazebo worlds and robot spawning
├── husky_control/           Controllers (diff-drive, teleop, twist mux config)
├── husky_msgs/              Custom ROS messages (HuskyStatus)
├── husky_navigation/        ROS navigation stack configs (move_base, amcl, gmapping)
├── husky_viz/               Pre-configured RViz visualization files
├── husky_testing_utils/     Pose estimation benchmarking tools & GPS utilities
├── pure_pursuit/            Pure pursuit path-following controller (our algorithm)
├── demo/                    Demo assets
├── extras/                  Third-party dependencies bundled in-tree
│   ├── twist_mux/                  Velocity command multiplexer
│   ├── interactive_marker_twist_server/  RViz interactive marker control
│   ├── velodyne_description/       Velodyne VLP-16 URDF
│   ├── velodyne_gazebo_plugins/    Velodyne Gazebo simulation plugin
│   ├── realsense2_description/     Intel RealSense D435 URDF
│   ├── realsense_gazebo_plugin/    RealSense Gazebo simulation plugin
│   ├── cpr_onav_description/       Clearpath OutdoorNav URDF (unused, disabled)
│   ├── fath_pivot_mount_description/  Pivot mount for FLIR camera (unused)
│   └── flir_camera_description/    FLIR camera URDF (unused, disabled)
└── README.md
```

---

## Package Documentation

### husky_description

**What it does:** Defines the complete robot model (URDF/Xacro) including chassis, wheels, and all sensor mounts. This is the foundation — every other package depends on it for the robot definition.

**Robot specs (from URDF):**
- Mass: 46.034 kg
- Base dimensions: 0.99m x 0.57m x 0.25m
- Wheel radius: 0.165m, wheel width: 0.114m
- Wheelbase: 0.512m, track width: 0.555m
- Drive type: 4-wheel differential drive

**Sensors enabled by default:**
| Sensor | Mount | Topic |
|--------|-------|-------|
| Velodyne VLP-16 LiDAR | backpack_link (offset z=0.26m) | `/points` (PointCloud2) |
| Intel RealSense D435 | base_link (x=0.385, z=0.316) | `/realsense/...` |
| IMU (simulated) | base_link | `/imu/data` (50 Hz) |
| GPS (simulated) | base_link | `/navsat/fix`, `/navsat/vel` (10 Hz) |

**GPS reference datum:** lat=17.50969, lon=78.27728 (Hyderabad area)

> **Note:** The robot now carries a **single** GPS receiver at `base_link` (publishing `/navsat/fix`). The earlier dual GNSS antennas (`gps_left`/`gps_right` → `/sensors/gps_1/fix`, `/sensors/gps_2/fix`) have been removed — the `gnss_antenna` macro is commented out in `husky.urdf.xacro`.

**IMU noise profiles:** Gazebo IMU plugin noise is selected at launch by a named profile in `husky_description/config/imu_profiles.yaml` (`highend` / `mid` / `lowend`; higher tier = less noise). Pick one with the `imu_profile` launch arg or the `HUSKY_IMU_PROFILE` env var, e.g. `imu_profile:=lowend`. Default is `mid`.

**Sensors available but disabled:**
- SICK LMS1XX 2D LiDAR (set `HUSKY_LMS1XX_ENABLED=1`)
- Hokuyo UST-10 2D LiDAR (set `HUSKY_UST10_ENABLED=1`)
- FLIR Blackfly camera (set `HUSKY_BLACKFLY=1`)
- Secondary Velodyne/RealSense (set respective `_SECONDARY_ENABLED=1`)

**Key files:**
- `urdf/husky.urdf.xacro` — Main robot definition, sensor configuration, Gazebo plugins (IMU, single GPS)
- `urdf/wheel.urdf.xacro` — Wheel geometry, inertia, friction (mu1=mu2=1.0)
- `urdf/accessories/vlp16_mount.urdf.xacro` — Velodyne mount macro
- `urdf/accessories/gnss_antenna.urdf.xacro` — Dual GPS antenna mount macro (currently disabled in the URDF)
- `config/imu_profiles.yaml` — IMU noise profiles (`highend`/`mid`/`lowend`) loaded by the URDF
- `launch/description.launch` — Loads the `robot_description` parameter via xacro (forwards `imu_profile`)

---

### husky_gazebo

**What it does:** Provides Gazebo world files and launch files to spawn the Husky robot in simulation. This is what you launch first to start any simulation session.

**Available worlds:**

| Launch command | World |
|----------------|-------|
| `roslaunch husky_gazebo husky_custom_world.launch launch_type:=agriculture` | Agricultural field (`agriculture.world`) |
| `roslaunch husky_gazebo husky_custom_world.launch launch_type:=barrels` | Barrel obstacles (default, `barrels.world`) |
| `roslaunch husky_gazebo empty_world.launch` | Empty flat ground (`empty.world`) |
| `roslaunch husky_gazebo playpen.launch` | Clearpath enclosed playpen (`clearpath_playpen.world`) |

Other world assets present: `worlds/waypoint.world`. Additional launch helpers: `husky_playpen.launch`, `multi_husky_playpen.launch`, `realsense.launch`.

**What `spawn_husky.launch` does (called internally):**
1. Includes `husky_control/control.launch` (loads robot description + spawns controllers + localization)
2. Includes `husky_control/teleop.launch` (keyboard/joystick control)
3. Optionally includes `realsense.launch` (if `HUSKY_REALSENSE_ENABLED=1`)
4. Spawns the Husky URDF model into Gazebo at position (x, y, z, yaw) — all default to 0

**Spawn position args:** You can override with `x:=5.0 y:=3.0 yaw:=1.57`

**IMU profile passthrough:** `empty_world.launch` → `spawn_husky.launch` → `control.launch` → `description.launch` all forward the `imu_profile` arg (defaulting from `HUSKY_IMU_PROFILE`), so `roslaunch husky_gazebo empty_world.launch imu_profile:=highend` reaches the URDF.

**Key files:**
- `launch/husky_custom_world.launch` — Entry point, selects world by `launch_type` arg
- `launch/spawn_husky.launch` — Robot spawning and controller bringup
- `worlds/agriculture.world`, `worlds/barrels.world` — Gazebo world definitions

---

### husky_control

**What it does:** Configures and launches all the controllers that make the robot move. Handles the diff-drive controller, velocity multiplexing (twist_mux), teleop input, and the dual EKF localization stack.

**Launch chain:**
```
control.launch
  ├── Loads husky_description (robot model, forwards imu_profile)
  ├── Spawns husky_velocity_controller (diff-drive)
  ├── Spawns husky_joint_publisher (joint states)
  ├── If ENABLE_EKF=true (default):
  │     ├── ekf_localization_local  (odom frame: wheel odom + IMU -> /odometry/filtered)
  │     ├── ekf_localization_global (map frame: wheel odom + IMU + GPS -> /odometry/filtered_map)
  │     └── gps_to_odom_node.py     (/navsat/fix -> /odometry/gps for the global EKF)
  ├── Else (ENABLE_EKF=false):
  │     └── truth_tf_world_to_base_link_pub.py (ground truth TF: world -> base_link)
  ├── robot_state_publisher (URDF TF tree)
  ├── interactive_marker_twist_server (RViz drag control)
  └── twist_mux (velocity priority arbitration)

teleop.launch
  ├── joy_node (joystick input, if enabled)
  ├── teleop_twist_joy (joy_teleop/cmd_vel)
  └── teleop_twist_keyboard (kb_teleop/cmd_vel)
```

**Velocity multiplexing (twist_mux) priorities:**

| Priority | Source | Topic | Timeout |
|----------|--------|-------|---------|
| 255 | Emergency stop (lock) | `e_stop` | 0.0s |
| 10 | Joystick | `joy_teleop/cmd_vel` | 0.5s |
| 9 | Keyboard | `kb_teleop/cmd_vel` | 0.5s |
| 8 | Interactive markers | `twist_marker_server/cmd_vel` | 0.5s |
| 1 | External (algorithm) | `husky/cmd_vel` | 0.5s |

The pure_pursuit node publishes to `husky/cmd_vel` (priority 1), so joystick/keyboard can always override it for manual intervention.

**Diff-drive controller (control.yaml):**
- Publish rate: 50 Hz
- Max linear velocity: 1.0 m/s
- Max angular velocity: 2.0 rad/s
- Linear acceleration limit: 3.0 m/s^2
- Angular acceleration limit: 6.0 rad/s^2
- Wheel separation multiplier: 1.875
- Odometry TF: disabled (uses ground truth or EKF instead)

**Localization — dual EKF (enabled by default; set `ENABLE_EKF=false` for ground-truth mode):**

Two `robot_localization` EKF instances run together, both in 2D mode at 50 Hz:

- **Local EKF** (`ekf_localization_local`, `localization_local.yaml`) — `world_frame=odom`. Fuses wheel odometry + IMU. Publishes `/odometry/filtered` and the `odom -> base_link` TF.
- **Global EKF** (`ekf_localization_global`, `localization_global.yaml`) — `world_frame=map`. Fuses wheel odometry + IMU + GPS. Publishes `/odometry/filtered_map` (remapped from `odometry/filtered`) and the `map -> odom` TF.
- **gps_to_odom** (`gps_to_odom_node.py`) — converts `/navsat/fix` (lat/lon) to `/odometry/gps` (x,y in the `map` frame) via a flat-Earth approximation, independent of odom/IMU. Feeds the global EKF as `odom1`. Reference datum is passed as `ref_lat`/`ref_lon` node params.

> The EKF replaces the older single-EKF + `navsat_transform_node` design. `config/localization.yaml` and `config/navsat_transform.yaml` remain in the tree but are **legacy / unused** — `control.launch` no longer references them.

**Ground-truth mode (set `ENABLE_EKF=false`):**
- `truth_tf_world_to_base_link_pub.py` subscribes to `/gazebo/model_states`
- Publishes TF: `world -> base_link` at 50 Hz directly from Gazebo's ground truth

**Key files:**
- `config/control.yaml` — Diff-drive controller parameters
- `config/twist_mux.yaml` — Velocity priority configuration
- `config/localization_local.yaml` — Local EKF parameters (odom frame: odom + IMU)
- `config/localization_global.yaml` — Global EKF parameters (map frame: odom + IMU + GPS)
- `config/teleop_ps4.yaml` / `teleop_logitech.yaml` — Gamepad mappings
- `src/gps_to_odom_node.py` — Converts `/navsat/fix` to `/odometry/gps` for the global EKF
- `src/truth_tf_world_to_base_link_pub.py` — Gazebo ground truth TF publisher (ground-truth mode)
- `config/localization.yaml`, `config/navsat_transform.yaml` — legacy single-EKF configs, no longer referenced

---

### pure_pursuit

**What it does:** Our custom pure pursuit path-following controller. Takes a GPS-derived robot pose and a predefined path, then computes velocity commands to follow the path.

> **Heads-up (GPS topic mismatch):** This pipeline was built for a dual-GPS antenna robot. `config/config.yaml` subscribes to `/sensors/gps_1/fix` and `/sensors/gps_2/fix`, but the current single-GPS simulation only publishes `/navsat/fix`. To run pure_pursuit as-is you must either remap/point the dual-GPS pose node at available topics, or re-enable the `gnss_antenna` macro in `husky.urdf.xacro`.

**Architecture:**
```
                    ┌─────────────────────┐
 GPS 1 (/fix) ─────┤                     │
                    │  dualGPS_robot_     ├──── /robot_mid_pose (PoseStamped)
 GPS 2 (/fix) ─────┤  pose_pub.py        ├──── /rover_mid/fix (NavSatFix)
                    └─────────────────────┘
                                                        │
                    ┌─────────────────────┐             │
                    │  path_publisher.py  ├──── /pure_pursuit/path (Path)
                    └─────────────────────┘             │
                                                        ▼
                    ┌─────────────────────────────────────────┐
                    │  pure_pursuit_node.py                    │
                    │                                         │
                    │  1. Find nearest waypoint on path       │
                    │  2. Compute lookahead point at L dist   │
                    │  3. Calculate alpha (heading error)     │
                    │  4. omega = (3*v*sin(alpha)) / L        │
                    │  5. Reduce speed on sharp turns         │
                    │                                         │
                    │  ├──── /husky/cmd_vel (Twist)           │
                    │  └──── /lookahead_arrow (Marker)        │
                    └─────────────────────────────────────────┘
```

**Nodes:**

1. **dualGPS_robot_pose_pub.py** — Dual GPS pose estimation
   - Subscribes to two GPS antenna topics (configurable in config.yaml)
   - Converts GPS lat/lon to local XY using the reference datum
   - Computes midpoint between antennas as robot position
   - Derives heading from the antenna-to-antenna vector: `atan2(dy, dx) - pi/2`
   - Adds 0.258m X-offset to compensate for antenna-to-base_link distance
   - Publishes: `/robot_mid_pose` (PoseStamped), `/rover_mid/fix` (NavSatFix)

2. **path_publisher.py** — Generates a predefined waypoint path
   - Creates a square-wave (boustrophedon) pattern
   - Default: 13m long rows, 1m row spacing, 4 rows, 1m waypoint spacing
   - Publishes latched: `/pure_pursuit/path` (Path, frame: world)

3. **pure_pursuit_node.py** — The controller
   - Lookahead distance: 0.5m (configurable)
   - Target velocity: 0.5 m/s (configurable)
   - Control rate: 10 Hz
   - Steering law: `omega = (3 * v * sin(alpha)) / lookahead_distance`
   - Speed reduction: 50% for moderate turns (>0.5 rad/s omega), 80% for sharp turns (>2.0 rad/s)
   - Goal tolerance: 0.5m from final waypoint
   - Publishes: `/husky/cmd_vel` (Twist), `/lookahead_arrow` (Marker for RViz)

**Configuration (config/config.yaml):**
```yaml
gps:
  ref_lat: 17.50969045486119
  ref_lon: 78.27728104672221
  topic_navsat_fix1: /sensors/gps_1/fix
  topic_navsat_fix2: /sensors/gps_2/fix
pose:
  frame_id: world
  publish_topic: /robot_mid_pose
```

**Key files:**
- `src/pure_pursuit_node.py` — Main controller logic
- `src/path_publisher.py` — Path generator
- `src/dualGPS_robot_pose_pub.py` — Dual GPS pose publisher
- `config/config.yaml` — GPS reference and topic configuration
- `rviz/pure_pursuit.rviz` — RViz visualization for debugging
- `launch/pure_pursuit.launch` — Starts path publisher + controller
- `launch/dual_gps_pose.launch` — Starts dual GPS pose node
- `launch/hhgps_husky.launch`, `launch/record_path_tester.launch` — Additional GPS/path-recording helpers

---

### husky_msgs

**What it does:** Defines the `HuskyStatus` custom ROS message for reporting real-robot hardware diagnostics (battery voltage, motor temps, driver currents, e-stop state, etc.).

**Note:** This message is designed for the physical Husky hardware. It is not published or consumed by anything in the simulation pipeline. It exists in the project because it is part of the upstream Husky repository.

---

### husky_navigation

**What it does:** Provides configurations for the standard ROS Navigation Stack — move_base, AMCL localization, GMapping SLAM, and frontier exploration. These use 2D laser scans and occupancy grid maps for autonomous navigation.

**Note:** This package is not used by the pure pursuit pipeline, which uses GPS-based pose estimation and a predefined path instead of the ROS nav stack. It is retained from the upstream Husky repo and may be useful if you want to test laser-based navigation in the future.

**Available launch files:**
- `amcl_demo.launch` — Localization on a known map using particle filter
- `gmapping_demo.launch` — Online SLAM (simultaneous mapping + localization)
- `move_base_mapless_demo.launch` — Reactive obstacle avoidance without a map
- `exploration_demo.launch` — Frontier-based autonomous exploration

---

### husky_viz

**What it does:** Contains pre-configured RViz layout files and launch helpers for visualizing the robot model, sensor data, and navigation state.

**RViz configs:**
- `rviz/robot.rviz` — Full operational view (TF, model, scan, odometry, goals)
- `rviz/model.rviz` — Robot structure only (meshes, joints)
- `rviz/nav.rviz` — Navigation-focused (costmaps, paths, laser scans)

**Note:** The pure_pursuit package has its own RViz config at `pure_pursuit/rviz/pure_pursuit.rviz` which is configured specifically for the GPS path-following workflow. The configs in husky_viz are general-purpose and can be useful for debugging other aspects of the simulation.

---

### husky_testing_utils

**What it does:** Provides a pose estimation benchmarking framework that compares multiple localization methods (wheel odometry, IMU dead reckoning, EKF) against Gazebo ground truth. Also contains a WGS84 geodesic GPS publisher.

**Benchmarking pipeline:**
```
┌──────────────┐   ┌──────────────────────┐   ┌────────────────────────┐
│ IMU Dead     │   │ Pose Benchmark       │   │ Pose Benchmark         │
│ Reckoning    │──►│ Recorder             │──►│ Analyzer               │
│ Node         │   │                      │   │                        │
│              │   │ Collects from:       │   │ Generates:             │
│ /imu/data    │   │  - Ground truth      │   │  - Position RMSE       │
│     ↓        │   │  - Wheel odometry    │   │  - Heading RMSE        │
│ /imu_only/   │   │  - IMU dead reckoning│   │  - Drift rate          │
│   odom       │   │  - EKF local+global  │   │  - Trajectory plots    │
└──────────────┘   └──────────────────────┘   └────────────────────────┘
```

**Nodes:**

1. **imu_dead_reckoning_node.py** — IMU-only pose estimation
   - Subscribes to `/imu/data` (50 Hz) and integrates acceleration/angular velocity to estimate pose
   - Applies configurable turn-on bias modeling (random bias sampled at startup) for accelerometer and gyroscope
   - Transforms body-frame accelerations to world-frame using quaternion rotation, compensates gravity
   - Publishes: `/imu_only/odom` (Odometry)

2. **pose_benchmark_recorder_node.py** — Multi-source data collection
   - Simultaneously records ground truth, wheel odometry, IMU dead reckoning, the local EKF, and the global EKF
   - Outputs a timestamped CSV with **16 columns**: `timestamp, gt_x, gt_y, gt_yaw, odom_x, odom_y, odom_yaw, imu_x, imu_y, imu_yaw, ekf_x, ekf_y, ekf_yaw, ekf_global_x, ekf_global_y, ekf_global_yaw`
   - `ekf_*` = local EKF (`/odometry/filtered`); `ekf_global_*` = global EKF (`/odometry/filtered_map`)
   - Configurable sample rate (default 50 Hz) and duration (default: until Ctrl+C)
   - Results saved to `results/benchmark_YYYYMMDD_HHMMSS.csv`

3. **pose_benchmark_analyzer.py** — Offline analysis and visualization
   - Computes per-method metrics (Odometry, IMU dead reckoning, EKF local, EKF Global): position RMSE, heading RMSE, max errors, drift rate (m/m)
   - Generates plots: `trajectory_comparison.png`, `position_error.png`, `heading_error.png`, `rmse_summary.png`
   - Usage: `python3 pose_benchmark_analyzer.py [--input <csv>] [--output <dir>]`
   - Auto-detects latest CSV in `results/` if no input specified

4. **gps_pub_wh.py** — WGS84 geodesic dual-GPS publisher
   - Similar to `dualGPS_robot_pose_pub.py` in pure_pursuit but uses geodesic math (WGS84 ellipsoid) instead of flat-earth approximation
   - Subscribes to `/sensors/gps_1/fix`, `/sensors/gps_2/fix` (dual-GPS — **legacy** w.r.t. the current single-GPS sim, which publishes `/navsat/fix`)
   - Publishes: `/rover_mid/fix` (NavSatFix), `/heading` (Float64)

**IMU noise profiles:**

Gazebo IMU plugin noise is selected by a named profile in `husky_description/config/imu_profiles.yaml`, chosen via the `imu_profile` launch arg or the `HUSKY_IMU_PROFILE` env var (higher tier = better IMU = less noise). Default is `mid`.

| Profile | accel_noise | accel_drift | rate_noise | rate_drift | heading_noise | heading_drift | turnon_accel | turnon_gyro |
|---------|-------------|-------------|------------|------------|---------------|---------------|--------------|-------------|
| highend | 0.005 | 0.005 | 0.005 | 0.005 | 0.005 | 0.005 | 0.0 | 0.0 |
| mid | 0.02 | 0.01 | 0.005 | 0.002 | 0.005 | 0.005 | 0.01 | 0.005 |
| lowend | 0.08 | 0.04 | 0.03 | 0.015 | 0.02 | 0.02 | 0.03 | 0.015 |

The `accel_*`/`rate_*`/`heading_*` columns are the Gazebo plugin noise (set by `imu_profiles.yaml`). The `turnon_*` columns are the dead-reckoning node's turn-on bias (`turnon_bias_accel`/`turnon_bias_gyro` in `config/benchmark.yaml`) — set these manually to match the chosen profile.

**Configuration (config/benchmark.yaml):**
```yaml
benchmark:
  duration: 0.0           # 0 = record until Ctrl+C
  robot_name: "husky"
  sample_rate: 50.0       # Hz
imu_dead_reckoning:
  gravity: 9.81
  initial_x: 0.0
  initial_y: 0.0
  initial_yaw: 0.0
  turnon_bias_accel: 0.02   # m/s^2, std dev of random bias
  turnon_bias_gyro: 0.01    # rad/s, std dev of random bias
```

**Key files:**
- `src/imu_dead_reckoning_node.py` — IMU-only dead reckoning estimator
- `src/pose_benchmark_recorder_node.py` — Multi-source pose data recorder
- `src/pose_benchmark_analyzer.py` — Offline analysis and plot generation
- `src/gps_pub_wh.py` — WGS84 geodesic GPS publisher
- `config/benchmark.yaml` — Benchmark parameters and IMU bias config
- `launch/pose_benchmark.launch` — Launches dead reckoning + recorder
- `results/` — Output directory for CSV data and analysis plots

---

### extras/

Third-party packages bundled in-tree because they are build/runtime dependencies.

| Package | Used by | Purpose |
|---------|---------|---------|
| `twist_mux` | husky_control | Multiplexes cmd_vel topics by priority |
| `interactive_marker_twist_server` | husky_control | Drag-to-drive control in RViz |
| `velodyne_description` | husky_description | VLP-16 URDF and meshes |
| `velodyne_gazebo_plugins` | husky_gazebo | Simulates VLP-16 point cloud in Gazebo |
| `realsense2_description` | husky_description | RealSense D435 URDF and meshes |
| `realsense_gazebo_plugin` | husky_gazebo | Simulates RealSense depth camera in Gazebo |
| `cpr_onav_description` | (disabled) | Clearpath OutdoorNav — not used |
| `fath_pivot_mount_description` | (disabled) | Pivot mount for FLIR — not used |
| `flir_camera_description` | (disabled) | FLIR Blackfly camera — not used |

---

## How to Run

### 1. Workspace Setup

```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
git clone git@github.com:akrbot/husky.git
```

### 2. Install Gazebo Models

```bash
cd ~/ros_ws/src
git clone git@github.com:akrbot/gazebo_models.git
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros_ws/src/gazebo_models' >> ~/.bashrc
source ~/.bashrc
```

### 3. Build

```bash
cd ~/ros_ws
catkin_make
source devel/setup.bash
```

### 4. Launch the Simulation

```bash
# Terminal 1: Start Gazebo with the agriculture world
roslaunch husky_gazebo husky_custom_world.launch launch_type:=agriculture
```

### 5. Run Pure Pursuit

> **Prerequisite:** The dual-GPS pose node expects `/sensors/gps_1/fix` + `/sensors/gps_2/fix`. The current single-GPS sim publishes only `/navsat/fix`, so remap the pose node's topics or re-enable the `gnss_antenna` macro in `husky.urdf.xacro` before running this pipeline.

```bash
# Terminal 2: Start dual-GPS pose estimation
roslaunch pure_pursuit dual_gps_pose.launch

# Terminal 3: Publish the path
rosrun pure_pursuit path_publisher.py

# Terminal 4: Start the controller
rosrun pure_pursuit pure_pursuit_node.py
```

Or use the combined launch:
```bash
# Terminal 2: GPS pose
roslaunch pure_pursuit dual_gps_pose.launch

# Terminal 3: Path + controller
roslaunch pure_pursuit pure_pursuit.launch
```

### 6. Run Pose Estimation Benchmark

```bash
# Terminal 1: Start Gazebo with EKF enabled
ENABLE_EKF=true roslaunch husky_gazebo empty_world.launch

# Terminal 2: Start benchmarking (IMU dead reckoning + recorder)
roslaunch husky_testing_utils pose_benchmark.launch

# Terminal 3: Drive the robot (teleop, pure pursuit, or any other method)
roslaunch husky_control teleop.launch
```

Drive the robot around, then press Ctrl+C in Terminal 2 to stop recording. Analyze the results:

```bash
python3 $(rospack find husky_testing_utils)/src/pose_benchmark_analyzer.py
```

**With a specific IMU noise profile (e.g. low-cost `lowend`):**

```bash
# Terminal 1: Launch with the lowend IMU profile (env-var style)
HUSKY_IMU_PROFILE=lowend ENABLE_EKF=true roslaunch husky_gazebo empty_world.launch

# ...or as a launch arg:
# roslaunch husky_gazebo empty_world.launch imu_profile:=lowend

# Terminal 2: Launch benchmark (update turnon_bias in benchmark.yaml to match the profile)
roslaunch husky_testing_utils pose_benchmark.launch
```

Available profiles: `highend`, `mid` (default), `lowend` — defined in `husky_description/config/imu_profiles.yaml`.

### 7. Visualize in RViz (optional)

```bash
rviz -d $(rospack find pure_pursuit)/rviz/pure_pursuit.rviz
```

---

## ROS Topic Map

### Simulation core (from Gazebo + controllers)

| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/gazebo/model_states` | ModelStates | Gazebo | Ground truth poses of all models |
| `/imu/data` | Imu | hector_gazebo_ros_imu | Simulated IMU (50 Hz) |
| `/navsat/fix` | NavSatFix | hector_gazebo_ros_gps | Single GPS receiver at base_link (10 Hz) |
| `/navsat/vel` | Vector3Stamped | hector_gazebo_ros_gps | GPS velocity |
| `/points` | PointCloud2 | velodyne_gazebo_plugin | VLP-16 point cloud |
| `/joint_states` | JointState | controller_manager | Wheel joint positions/velocities |
| `/husky_velocity_controller/odom` | Odometry | diff_drive_controller | Wheel odometry |
| `/tf` | TFMessage | robot_state_publisher + others | Full TF tree |

### Pure pursuit pipeline

> Legacy dual-GPS topics — the dual-GPS pose node expects `/sensors/gps_1/fix` + `/sensors/gps_2/fix`, which the current single-GPS sim does not publish (see the pure_pursuit section).

| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/robot_mid_pose` | PoseStamped | dualGPS_robot_pose_pub | Fused robot pose from dual GPS |
| `/rover_mid/fix` | NavSatFix | dualGPS_robot_pose_pub | Averaged GPS fix |
| `/pure_pursuit/path` | Path | path_publisher | Waypoint path to follow |
| `/husky/cmd_vel` | Twist | pure_pursuit_node | Velocity commands (into twist_mux) |
| `/lookahead_arrow` | Marker | pure_pursuit_node | RViz visualization of lookahead |

### Benchmarking pipeline

| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/imu_only/odom` | Odometry | imu_dead_reckoning_node | IMU-only dead reckoning pose estimate |
| `/odometry/gps` | Odometry | gps_to_odom_node | GPS lat/lon converted to x,y in map frame |
| `/odometry/filtered` | Odometry | ekf_localization_local | Local EKF: wheel odom + IMU (odom frame) |
| `/odometry/filtered_map` | Odometry | ekf_localization_global | Global EKF: wheel odom + IMU + GPS (map frame) |

### Control (twist_mux inputs -> output)

| Topic | Type | Priority | Description |
|-------|------|----------|-------------|
| `joy_teleop/cmd_vel` | Twist | 10 | Joystick input |
| `kb_teleop/cmd_vel` | Twist | 9 | Keyboard input |
| `twist_marker_server/cmd_vel` | Twist | 8 | RViz interactive marker |
| `husky/cmd_vel` | Twist | 1 | Algorithm output (pure pursuit) |
| `husky_velocity_controller/cmd_vel` | Twist | (output) | Final command sent to diff-drive |

---

## TF Tree

The world-frame chain depends on the localization mode:

- **EKF mode (default, `ENABLE_EKF=true`):** `map → odom → base_link`
  - global EKF publishes `map → odom`, local EKF publishes `odom → base_link`
- **Ground-truth mode (`ENABLE_EKF=false`):** `world → base_link` (from Gazebo ground truth)

```
map  (EKF mode)              world  (ground-truth mode)
  └── odom                     └── base_link
        └── base_link
              ├── base_footprint
              ├── inertial_link
              ├── imu_link
              ├── front_left_wheel_link
              ├── front_right_wheel_link
              ├── rear_left_wheel_link
              ├── rear_right_wheel_link
              ├── top_plate_link
              │     └── backpack_link
              │           └── velodyne
              └── realsense_camera_link
                    └── realsense_camera_depth_frame
                          └── realsense_camera_depth_optical_frame
```
