# Project Context: AI-Assisted Adaptive Sensor Fusion for Robot Localisation
---

## What this project is

An MTech research project building an **adaptive EKF** where sensor measurement noise covariances (the R matrix) are dynamically estimated by a learned neural network, rather than being fixed at design time. The neural network acts as a **per-sensor reliability estimator**, outputting a weight in [0, 1] for each sensor at each timestep, indicating how much the EKF should trust that sensor right now.

**One-line pitch:** Replace the fixed R matrix in a standard EKF with a learned, real-time reliability estimator that degrades gracefully under sensor faults.

---

## Tech stack

- **Robot framework:** ROS 1 (simulation in Gazebo)
- **Localisation baseline:** `robot_localization` ROS package (EKF node)
- **Sensors (current):** wheel odometry + IMU
- **Sensors (planned):** add GPS as third modality
- **Neural network (current plan):** MLP reliability estimator
- **Neural network (future plan):** upgrade to LSTM to capture temporal fault signatures
- **Language:** Python (PyTorch for ML, rclpy for ROS nodes)
- **Ground truth:** Gazebo simulation ground truth pose

---

## Current project status

- [x] Simulation environment set up in Gazebo
- [x] Benchmark data collected: 3 runs, CSV format with columns:
  `timestamp, gt_x, gt_y, gt_yaw, odom_x, odom_y, odom_yaw, imu_x, imu_y, imu_yaw, ekf_x, ekf_y, ekf_yaw`
- [x] Baseline RMSE benchmarked across all 3 runs
- [ ] Sensor fault injection scenarios (in progress)
- [ ] Dataset generation pipeline
- [ ] MLP reliability estimator (not started)
- [ ] Adaptive EKF integration (not started)
- [ ] Paper writing

---

## Benchmark results (current baseline)

All figures are position RMSE in metres against Gazebo ground truth.

| Run | Duration | Odometry RMSE | IMU dead-reckoning RMSE | EKF RMSE |
|-----|----------|--------------|------------------------|----------|
| 1   | 70s      | 7.671 m      | **0.117 m**            | 2.940 m  |
| 2   | 88s      | 7.403 m      | **0.290 m**            | 2.783 m  |
| 3   | 58s      | 5.816 m      | **1.082 m**            | 2.254 m  |

**Key finding:** The stock `robot_localization` EKF performs *worse* than IMU dead-reckoning alone. Root cause: the process noise covariance matrices (Q/R) are not tuned — the EKF is over-trusting the badly drifting wheel odometry (likely wheel slip in sim), which poisons the fused estimate.

**Action required before AI-EKF work:** Tune `robot_localization` YAML — increase odometry measurement noise covariance so the EKF down-weights the bad odom signal. The EKF must beat IMU dead-reckoning on clean data before it can serve as a meaningful baseline.

---

## Architecture overview

```
Sensor streams (odom, IMU, GPS)
        │
        ▼
┌─────────────────────────┐
│   Feature Extractor      │   Per-sensor observable features
│   (windowed, ~1-3s)      │   — no ground truth used here
└────────────┬────────────┘
             │
             ▼
┌─────────────────────────┐
│  MLP Reliability         │   Output: weight_odom, weight_imu,
│  Estimator (→ LSTM)      │   weight_gps ∈ [0,1]
└────────────┬────────────┘
             │
             ▼
┌─────────────────────────┐
│  Adaptive EKF            │   R matrix updated each step
│  (robot_localization     │   using predicted reliability weights
│   or custom node)        │
└─────────────────────────┘
             │
             ▼
        Localisation estimate
```

**Hard fallback (important):** If EKF innovation (predicted vs measured state gap) exceeds a threshold, override MLP weights with low-confidence values regardless of what the model says. Prevents confident wrong predictions from crashing the filter.

---

## MLP input features (what the model sees at inference)

The model is trained on ground-truth-labelled data but at inference only sees these observable signals — no ground truth required.

### Odometry features
- Left/right wheel velocity disagreement (asymmetric slip indicator)
- Commanded velocity vs measured velocity ratio
- Sudden velocity discontinuities (delta between consecutive readings)
- Zero velocity while IMU reports motion (wheel lock/slip)

### IMU features
- Gyro bias drift rate (how fast bias is changing)
- Accelerometer magnitude deviation from 9.81 m/s² when near-stationary
- High-frequency vibration power (motor noise coupling)
- Message timestamp jitter / dropout rate
- Temperature (if available)

### GPS features (when added)
- Number of satellites
- HDOP / PDOP dilution of precision
- Velocity estimate vs IMU integration disagreement
- Sudden position jump magnitude
- Time since last valid fix

### Cross-sensor features (most discriminative)
- IMU-predicted position vs odom-predicted position over short window
  - Agreement → both likely reliable
  - Divergence → at least one failing; individual features disambiguate which

---

## Label generation (training data only)

```python
# For each timestep t, compute windowed RMSE per sensor
window = 2.0  # seconds
odom_rmse_t = rmse(odom_positions[t-window:t], gt_positions[t-window:t])
imu_rmse_t  = rmse(imu_positions[t-window:t],  gt_positions[t-window:t])

# Convert errors to soft reliability weights (lower error = higher weight)
inv_odom = 1.0 / (odom_rmse_t + eps)
inv_imu  = 1.0 / (imu_rmse_t  + eps)
total    = inv_odom + inv_imu

label_odom = inv_odom / total   # ∈ (0, 1), sums to 1 across sensors
label_imu  = inv_imu  / total
```

Use **soft labels** (continuous), not hard 0/1 — trains significantly better.

---

## Simulation fault injection scenarios

Build a dataset with coverage across fault types. Target split:

| Scenario type         | Proportion | Notes |
|-----------------------|------------|-------|
| Clean (all nominal)   | 30%        | Baseline; model should output ~equal weights |
| Odometry fault only   | 25%        | IMU is the reference |
| IMU fault only        | 25%        | Odometry is the reference |
| Both degraded / mixed | 20%        | Model should output low confidence on both |

### Odometry fault types
- Wheel slip: reduce friction coefficient in Gazebo material properties
- Encoder noise: add Gaussian noise to wheel tick counts in plugin
- Asymmetric slip: one wheel only (surface edge simulation)
- Stuck wheel: zero velocity on one encoder

### IMU fault types
- Gyro bias injection: add constant offset to gyro Z (thermal drift sim)
- Random walk amplification: increase noise density parameter
- Vibration coupling: add sinusoidal noise at motor frequencies
- Spike/outlier: occasional large values (EMI simulation)

### GPS fault types (for later)
- Signal blockage: stop publishing or set covariance = ∞
- Multipath: add slow structured bias (reflections)
- NLOS: random large jumps + normal operation
- Gradual drift: slow systematic error

### Environmental scenarios (important for generalisation)
- Sharp turns (odometry degrades during turning)
- Rough terrain (both sensors degrade, different signatures)
- Long straight runs (IMU drift accumulates)
- Start/stop cycles

---

## Research novelty positioning

The base approach (MLP weights → adaptive EKF R matrix) exists in literature (2018–2020 onwards). The novelty is being positioned as a combination of:

1. **Systematic failure-mode taxonomy and generalisation study** — characterise which injected faults produce learnable feature signatures and which don't; evaluate where learned reliability estimation fails
2. **Uncertainty-aware reliability output** — instead of point-estimate weights, output mean + variance (MC Dropout or small ensemble), propagate uncertainty into EKF covariance update

**Target venue:** ICRA workshop, IROS, IEEE SSRR, or IEEE RA-L (dataset/benchmark framing as fallback)

**Paper framing:** Not "we built this system" but "we systematically evaluate learned sensor reliability estimation under diverse failure modes and propose an uncertainty-aware output formulation that better characterises the filter's confidence"

**Literature to survey:** Search "adaptive EKF deep learning covariance estimation", "neural network sensor reliability estimation fusion", "learned measurement noise covariance robot localisation" — papers 2020–2025.

---

## Known issues / things to fix

1. **EKF baseline is misconfigured** — must tune `robot_localization` YAML before any AI-EKF work. The EKF must beat IMU dead-reckoning on clean runs first.
2. **Odometry is unusually bad** — verify Gazebo wheel plugin configuration, check wheel radii and wheelbase match URDF and EKF config.
3. **No fault injection yet** — entire dataset generation pipeline still to build.

---

## File/topic reference

| Item | Detail |
|------|--------|
| Benchmark CSVs | `benchmark_20260526_184208.csv`, `benchmark_20260526_190444.csv` |
| CSV columns | `timestamp, gt_x, gt_y, gt_yaw, odom_x, odom_y, odom_yaw, imu_x, imu_y, imu_yaw, ekf_x, ekf_y, ekf_yaw` |
| Sampling rate | 50 Hz |
| Coordinate frame | All estimates start from same origin (0,0) at t=0 |
| IMU | First 4 rows may be NaN (initialisation delay) |