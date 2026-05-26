# Husky Simulation - Pure Pursuit Algorithm Testing

Simulation environment built on top of the [Clearpath Husky](https://github.com/husky/husky) robot platform. The Husky model is used purely as a simulation testbed — it provides a well-tuned differential-drive robot with realistic physics properties (mass, inertia, friction, sensor noise) so we can focus on developing and validating our own algorithms rather than modelling a robot from scratch.

The primary algorithm under test is a **Pure Pursuit GPS-based path-following controller** that uses dual-GPS antenna pose estimation.

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
├── husky_testing_utils/     GPS publisher utility for testing
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
| Dual GPS antennas | base_link (left: y=+0.35, right: y=-0.35) | `/sensors/gps_1/fix`, `/sensors/gps_2/fix` (10 Hz) |

**GPS reference datum:** lat=17.50969, lon=78.27728 (Hyderabad area)

**Sensors available but disabled:**
- SICK LMS1XX 2D LiDAR (set `HUSKY_LMS1XX_ENABLED=1`)
- Hokuyo UST-10 2D LiDAR (set `HUSKY_UST10_ENABLED=1`)
- FLIR Blackfly camera (set `HUSKY_BLACKFLY=1`)
- Secondary Velodyne/RealSense (set respective `_SECONDARY_ENABLED=1`)

**Key files:**
- `urdf/husky.urdf.xacro` — Main robot definition, sensor configuration, Gazebo plugins
- `urdf/wheel.urdf.xacro` — Wheel geometry, inertia, friction (mu1=mu2=1.0)
- `urdf/accessories/vlp16_mount.urdf.xacro` — Velodyne mount macro
- `urdf/accessories/gnss_antenna.urdf.xacro` — GPS antenna mount macro
- `launch/description.launch` — Loads the robot_description parameter via xacro

---

### husky_gazebo

**What it does:** Provides Gazebo world files and launch files to spawn the Husky robot in simulation. This is what you launch first to start any simulation session.

**Available worlds:**

| Launch command | World |
|----------------|-------|
| `roslaunch husky_gazebo husky_custom_world.launch launch_type:=agriculture` | Agricultural field |
| `roslaunch husky_gazebo husky_custom_world.launch launch_type:=barrels` | Barrel obstacles (default) |
| `roslaunch husky_gazebo empty_world.launch` | Empty flat ground |
| `roslaunch husky_gazebo playpen.launch` | Clearpath enclosed playpen |

**What `spawn_husky.launch` does (called internally):**
1. Includes `husky_control/control.launch` (loads robot description + spawns controllers)
2. Includes `husky_control/teleop.launch` (keyboard/joystick control)
3. Spawns the Husky URDF model into Gazebo at position (x, y, z, yaw) — all default to 0
4. Starts `gps_joint_state_publisher` for the GPS antenna joints

**Spawn position args:** You can override with `x:=5.0 y:=3.0 yaw:=1.57`

**Key files:**
- `launch/husky_custom_world.launch` — Entry point, selects world by `launch_type` arg
- `launch/spawn_husky.launch` — Robot spawning and controller bringup
- `worlds/agriculture.world`, `worlds/barrels.world` — Gazebo world definitions

---

### husky_control

**What it does:** Configures and launches all the controllers that make the robot move. Handles the diff-drive controller, velocity multiplexing (twist_mux), teleop input, and optional EKF localization.

**Launch chain:**
```
control.launch
  ├── Loads husky_description (robot model)
  ├── Spawns husky_velocity_controller (diff-drive)
  ├── Spawns husky_joint_publisher (joint states)
  ├── Publishes ground truth TF: world -> base_link (if EKF disabled)
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

**EKF localization (disabled by default, set `ENABLE_EKF=true`):**
- Fuses wheel odometry + IMU
- 2D mode, 50 Hz
- NavSat transform for GPS->odom frame conversion

**Ground truth mode (default):**
- `truth_tf_world_to_base_link_pub.py` subscribes to `/gazebo/model_states`
- Publishes TF: `world -> base_link` at 50 Hz directly from Gazebo's ground truth

**Key files:**
- `config/control.yaml` — Diff-drive controller parameters
- `config/twist_mux.yaml` — Velocity priority configuration
- `config/localization.yaml` — EKF parameters (when enabled)
- `config/navsat_transform.yaml` — GPS transform config (reference datum)
- `config/teleop_ps4.yaml` / `teleop_logitech.yaml` — Gamepad mappings
- `src/truth_tf_world_to_base_link_pub.py` — Gazebo ground truth TF publisher

---

### pure_pursuit

**What it does:** Our custom pure pursuit path-following controller. Takes a GPS-derived robot pose and a predefined path, then computes velocity commands to follow the path.

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

**What it does:** Contains `gps_pub_wh.py` — a GPS publisher node that computes heading and midpoint from two GPS antennas using geodesic calculations (geographiclib).

Similar to `dualGPS_robot_pose_pub.py` in pure_pursuit but uses geodesic math (WGS84 ellipsoid) instead of flat-earth approximation. Publishes `/rover_mid/fix` and `/heading`.

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

### 6. Visualize in RViz (optional)

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
| `/sensors/gps_1/fix` | NavSatFix | hector_gazebo_ros_gps | Left GPS antenna (10 Hz) |
| `/sensors/gps_2/fix` | NavSatFix | hector_gazebo_ros_gps | Right GPS antenna (10 Hz) |
| `/navsat/vel` | TwistStamped | hector_gazebo_ros_gps | GPS velocity |
| `/points` | PointCloud2 | velodyne_gazebo_plugin | VLP-16 point cloud |
| `/joint_states` | JointState | controller_manager | Wheel joint positions/velocities |
| `/husky_velocity_controller/odom` | Odometry | diff_drive_controller | Wheel odometry |
| `/tf` | TFMessage | robot_state_publisher + others | Full TF tree |

### Pure pursuit pipeline

| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/robot_mid_pose` | PoseStamped | dualGPS_robot_pose_pub | Fused robot pose from dual GPS |
| `/rover_mid/fix` | NavSatFix | dualGPS_robot_pose_pub | Averaged GPS fix |
| `/pure_pursuit/path` | Path | path_publisher | Waypoint path to follow |
| `/husky/cmd_vel` | Twist | pure_pursuit_node | Velocity commands (into twist_mux) |
| `/lookahead_arrow` | Marker | pure_pursuit_node | RViz visualization of lookahead |

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

```
world
  └── base_link                      (ground truth from Gazebo, or EKF)
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
        ├── realsense_camera_link
        │     └── realsense_camera_depth_frame
        │           └── realsense_camera_depth_optical_frame
        ├── gps_left_link
        └── gps_right_link
```
