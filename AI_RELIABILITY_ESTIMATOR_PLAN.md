# AI Sensor Reliability Estimator — Development Plan
> Research-grounded plan for the AI-assisted adaptive sensor fusion component of the MTech project.
> Pair this with CONTEXT.md. Feed both to your AI agent.
> Last updated: May 2026

---

## 0. Where this sits in the literature (so you know what's novel)

The core idea — a neural network that outputs sensor noise covariance / trust weights feeding a Kalman filter — is **established**, not novel on its own. Key reference points:

- **Neural covariance adaptation for KF** (Springer 2021) — NN predicts Q and R from input time series, validated *on a Clearpath Husky dataset* (the same robot you use). This is the closest prior work to your base idea.
- **KalmanNet** (Revach et al., IEEE TSP 2022) — RNN learns the Kalman *gain* directly instead of the covariances. The canonical "deep learning inside the KF flow" paper. Read this one carefully.
- **End-to-end learning multi-sensor fusion** (arXiv 2025) — a TCN estimates velocity + noise covariance, then a sigmoid maps NN output to a [0,1] weight used in measurement fusion via element-wise multiplication on the residual. **This is almost exactly your "0 to 1 reliability weight" idea** — so your instinct is sound and current, but it means the plain version is already published.
- **Eigenvalue-based degradation detection + dynamic covariance weighting** (Sensors 2025) — LiDAR/IMU/RTK-GNSS fused with real-time per-sensor quality evaluation and "smooth fusion via covariance weighting." Directly comparable to your reliability-weighting goal.
- **Fault-tolerant multi-sensor fusion / IMM Kalman filters** (RAS 2022, others) — classical (non-learned) fault detection where sensor data is *intentionally corrupted* to test detection, and per-sensor weights reveal which sensor failed. This is the classical baseline your learned approach must beat.

**Takeaway for novelty:** Don't pitch "NN predicts sensor reliability → EKF." That's done. Pitch one or both of:
1. **Systematic fault-mode generalisation study** — a controlled benchmark of *which* simulated fault types produce learnable signatures and where learned reliability estimation breaks. Nobody has done this cleanly for ground-robot odom+IMU+GPS.
2. **Uncertainty-aware reliability output** — output a distribution (mean + variance) over each weight, propagate that into the EKF covariance, with a residual-based hard fallback for unseen faults.

Your competitive advantage: you have a *clean, controlled simulation* with selectable IMU noise profiles and ground truth — ideal for the controlled study angle that real-world papers can't easily do.

---

## 1. What the AI model outputs (decide this first — everything else follows)

You have three architecture choices, in increasing ambition. Pick ONE to start. Recommendation: **Option A first**, because it integrates cleanly with your existing `robot_localization` EKF and is the cleanest baseline-to-beat experiment.

### Option A (RECOMMENDED START): Per-sensor reliability weight → scales EKF measurement covariance R
- **Output:** one scalar per sensor per timestep, `w ∈ [0,1]` via sigmoid. `w=1` = fully trust, `w=0` = ignore.
- **How it feeds the filter:** scale the sensor's measurement noise covariance: `R_effective = R_base / (w + ε)`. Low weight → huge covariance → EKF ignores that sensor. (This is the standard "dynamic R" mechanism from the literature.)
- **Outputs needed (current sensors):** `w_odom`, `w_imu`. Later add `w_gps`.
- **Pro:** Plugs into a custom EKF node; the weight is interpretable; easy to validate.
- **Con:** Decoupled from filter training (two-stage), so it's the "already published" version unless you add the uncertainty/fault-study angle.

### Option B: Full covariance matrix prediction
- **Output:** the diagonal (or full) elements of each sensor's R matrix directly.
- More expressive, harder to train, harder to interpret. Skip unless Option A plateaus.

### Option C: Uncertainty-aware weights (THE NOVELTY EXTENSION)
- **Output:** for each sensor, a mean weight `μ_w` AND a variance `σ_w²` (via MC Dropout at inference, or a small 3–5 model ensemble, or direct heteroscedastic output).
- **How it feeds the filter:** `R_effective = R_base / (μ_w + ε) + k·σ_w²` — uncertainty inflates the covariance further.
- This is the version worth publishing. Build Option A first, then extend to C.

---

## 2. Input features (what the model sees — NO ground truth at inference)

The model trains on ground-truth labels but at inference uses ONLY observable signals. Use a **sliding window** of features (e.g. last 1–2 s = 50–100 samples at 50 Hz), not instantaneous values — fault signatures are temporal. This is also why the LSTM upgrade is justified.

### Odometry features (from `/husky_velocity_controller/odom`)
- Linear & angular velocity (vx, wz)
- Commanded vs measured velocity ratio (subscribe to `husky_velocity_controller/cmd_vel` too): slip indicator
- Velocity discontinuity: |v(t) − v(t−1)|
- Wheel velocity asymmetry if individual wheel speeds are accessible (from `/joint_states`)

### IMU features (from `/imu/data`)
- Angular velocity (wz), linear acceleration (ax, ay)
- Accel magnitude deviation from 9.81 when near-stationary (bias/scale fault indicator)
- High-frequency component / variance of accel over the window (vibration coupling)
- Gyro reading variance over window
- (Optional) the active IMU noise profile is known in sim — useful as a training signal but NOT available at real inference, so keep it out of the feature vector; use it only to verify the model learned the right thing.

### GPS features (when added — from `/navsat/fix` and `/odometry/gps`)
- Reported covariance from the NavSatFix message (the sim publishes this)
- Position jump magnitude between consecutive fixes
- GPS-derived velocity vs IMU-integrated velocity disagreement
- Time since last fix / dropout flag

### Cross-sensor features (MOST discriminative — emphasise these)
- Short-window position disagreement: IMU-dead-reckoned displacement vs odom displacement.
  - Agreement → both probably fine.
  - Disagreement → at least one is failing; the per-sensor features above disambiguate which.
- EKF innovation sequence (the predicted-vs-measured residual the filter already computes) — high innovation magnitude for a sensor is a strong, classical fault signal. You can log this from the EKF.

**Feature vector shape:** `[window_length, n_features]` for the LSTM, or flattened `[window_length × n_features]` for the MLP.

---

## 3. Label generation (training data only — uses ground truth)

You have `gt_*` columns in every benchmark CSV. Generate **soft labels** (continuous, not 0/1):

```python
import numpy as np

def windowed_position_error(est_xy, gt_xy, t_idx, window_samples):
    lo = max(0, t_idx - window_samples)
    e = np.linalg.norm(est_xy[lo:t_idx+1] - gt_xy[lo:t_idx+1], axis=1)
    return np.sqrt(np.mean(e**2))  # windowed RMSE

def reliability_labels(odom_xy, imu_xy, gt_xy, t_idx, window_samples=50, eps=1e-3):
    e_odom = windowed_position_error(odom_xy, gt_xy, t_idx, window_samples)
    e_imu  = windowed_position_error(imu_xy,  gt_xy, t_idx, window_samples)
    inv_o, inv_i = 1.0/(e_odom+eps), 1.0/(e_imu+eps)
    total = inv_o + inv_i
    return {'w_odom': inv_o/total, 'w_imu': inv_i/total}  # sum to 1
```

Notes:
- The window for the label should roughly match the input feature window.
- When you add GPS, extend to three-way normalisation.
- Soft labels train far better than hard fault/no-fault flags and let the EKF blend gracefully.
- Sanity check: in a clean run both weights ≈ 0.5; in an odom-fault run `w_odom` should drop sharply.

---

## 4. Dataset generation via fault injection

Your sim already has the perfect lever: **`imu_profiles.yaml` (highend/mid/lowend)** plus configurable turn-on bias. Use it. You need *variety in which sensor fails*, not just "everything noisy."

### Target dataset composition

| Scenario class | Share | Reference sensor for labels |
|----------------|-------|------------------------------|
| Clean (all nominal, `highend` IMU) | 25% | balanced weights |
| Odometry fault only | 25% | IMU is the good reference |
| IMU fault only (`lowend` profile + bias) | 25% | odom is the good reference |
| GPS fault only (after GPS added) | 15% | odom+IMU reference |
| Mixed / multiple degraded | 10% | low confidence everywhere |

### How to inject each fault in YOUR sim

**IMU faults (easiest — you already have the knobs):**
- Use `imu_profile:=lowend` for high-noise runs.
- Increase `turnon_bias_accel` / `turnon_bias_gyro` in `benchmark.yaml` for bias faults.
- For mid-run spikes/dropouts: add a small node that republishes `/imu/data` with injected spikes or dropped messages on a schedule (cleaner than editing the plugin).

**Odometry faults:**
- Wheel slip: lower wheel friction `mu1`/`mu2` in `wheel.urdf.xacro` (currently 1.0) for a slip run.
- Encoder noise / asymmetry: a republisher node on `/husky_velocity_controller/odom` that adds bias or scales one side.
- Stuck/locked: zero out velocity for a time window via the republisher.

**GPS faults (after GPS integration):**
- Blockage: stop publishing `/navsat/fix` (or `/odometry/gps`) for a window.
- Multipath/NLOS: republisher adds slow bias or occasional large jumps to `/odometry/gps`.
- Realistic covariance: you already learned the lesson — keep GPS drift realistic (~0.3, not 0.0001) so faults are meaningful.

**Trajectories (vary these across runs for generalisation):**
- Straight runs (IMU drift accumulates), tight turns (odom slip), figure-8, the boustrophedon path your `path_publisher.py` already makes, random teleop. Drive 60–90 s each.

### Recommended dataset scale
- ~40–60 runs total, 60–90 s each at 50 Hz → ~150k–250k labelled timesteps. Plenty for an MLP; adequate for a small LSTM.
- **Split by RUN, not by timestep** — never let windows from the same run leak across train/val/test, or you'll get a falsely optimistic result. Hold out entire runs (and ideally at least one *fault type combination*) for test.

### Reuse your existing recorder
Your `pose_benchmark_recorder_node.py` already logs the 16 columns including `ekf_*` and `ekf_global_*`. Extend it to also log the raw features you need (cmd_vel, IMU raw accel/gyro, GPS covariance, EKF innovations). One extra recorder column set = no new pipeline.

---

## 5. Model architecture

### Stage 1 — MLP (start here)
- Input: flattened window `[window × n_features]`.
- 2–3 hidden layers (e.g. 128 → 64 → 32), ReLU/ELU, dropout 0.1–0.2.
- Output head: `n_sensors` units → sigmoid (Option A) for independent weights, OR softmax if you want them to sum to 1.
- Loss: MSE or binary cross-entropy against soft labels. (KL divergence if using softmax + summed labels.)

### Stage 2 — LSTM (the planned upgrade)
- Input: sequence `[window, n_features]`.
- 1–2 LSTM layers (hidden 64–128) → final hidden state → dense → sigmoid output.
- Captures temporal fault signatures (gradual drift vs sudden spike) more naturally than the windowed MLP. This is a legitimate, expected progression and worth an ablation (MLP vs LSTM) in the paper.

### Stage 3 — Uncertainty extension (novelty)
- MC Dropout: keep dropout active at inference, run N forward passes, take mean + variance of each weight.
- OR a small deep ensemble (3–5 models, different seeds).
- Output `μ_w, σ_w²` per sensor.

---

## 6. EKF integration

`robot_localization` doesn't expose per-timestep dynamic R easily, so you have two paths:

**Path 1 (recommended for clean experiments): custom lightweight EKF node.**
- Implement a 2D EKF (state: x, y, yaw, vx, vy, wz) in Python/C++.
- At each measurement update, scale R by the model's predicted weight.
- Full control, easy to log innovations, matches the literature's "dynamic R" mechanism.

**Path 2 (reuse robot_localization): dynamic covariance on the message.**
- Republish each sensor's Odometry/IMU message with its `covariance` field scaled by `1/(w+ε)` before it reaches the EKF.
- `robot_localization` honours per-message covariances, so this injects your weights without forking the package. Less control over innovations but far less code.

Either way, keep the **residual-based hard fallback**: if EKF innovation for a sensor exceeds a threshold (e.g. Mahalanobis distance gate, which is the classical method from the fault-tolerance papers), clamp that sensor's weight low regardless of the model. This protects against unseen fault types — and is itself a defensible design contribution.

---

## 7. Evaluation (this is where the paper is won)

Your baselines are already produced by `pose_benchmark_analyzer.py`: Odometry, IMU DR, EKF Local, EKF Global. Add your method as a 5th column.

**Primary metrics (you already compute these):** Position RMSE, Yaw RMSE, Max error, Drift rate (m/m).

**The experiments that make it a paper:**
1. **Per-fault-type breakdown** — report RMSE for each method under each fault class (clean / odom-fault / IMU-fault / GPS-fault / mixed). Show your method wins *especially* under faults, where fixed-covariance EKF suffers.
2. **Generalisation boundary** — train on a subset of fault types, test on a *held-out* fault type. Show where the learned model degrades. This honesty IS the contribution (the systematic-study angle).
3. **Ablation** — MLP vs LSTM; with vs without cross-sensor features; with vs without the residual fallback; with vs without the uncertainty output.
4. **Weight interpretability** — plot predicted `w_odom`, `w_imu` over time during a fault run; show the weight drops when the fault is injected. Strong, intuitive figure for a paper.
5. **First fix the EKF baseline** — recall the EKF currently underperforms IMU DR because the stock R is mistuned. Tune it so the *fixed-covariance* EKF is a fair, strong baseline; otherwise reviewers will say you beat a strawman.

**Success criterion:** Your AI-EKF should beat ALL of {odom, IMU DR, EKF local, EKF global} on aggregate RMSE, and the margin should be largest under fault conditions.

---

## 8. Build order (concrete milestones)

1. **Fix the fixed-covariance EKF baseline** (tune local + global R/Q) so comparisons are fair.
2. **Extend the recorder** to log raw features + EKF innovations alongside the existing 16 columns.
3. **Write fault-injection republisher nodes** (IMU spike/dropout, odom slip/bias, GPS blockage/multipath).
4. **Run the dataset campaign** — 40–60 runs across fault classes & trajectories. Split by run.
5. **Offline: feature extraction + label generation** scripts (windowed).
6. **Train the MLP (Option A)**, validate that weights behave sensibly on held-out runs.
7. **Build the custom EKF node (Path 1)** with dynamic R + residual fallback; integrate the trained MLP.
8. **Benchmark** against the 4 baselines; produce per-fault-type tables and weight-over-time plots.
9. **Upgrade to LSTM**, re-benchmark, ablate.
10. **Add uncertainty output (Option C)**, re-benchmark — this is the headline novelty result.
11. **Write up** with the framing from §0.

---

## 9. Key references to cite / read

- Revach et al., *KalmanNet: NN-Aided Kalman Filtering for Partially Known Dynamics*, IEEE TSP 2022 (arXiv:2107.10043) — the canonical learned-KF paper.
- *Neural Network Adaptation of the Kalman Filter for Odometry Fusion*, Springer 2021 — NN predicts Q/R, validated on Husky.
- *End-to-End Learning-Based Multi-Sensor Fusion for AV Localization*, arXiv:2503.05088 (2025) — TCN + sigmoid [0,1] residual weighting; closest to your reliability-weight idea.
- *Adaptive Multi-Sensor Fusion with Eigenvalue-Based Degradation Detection*, Sensors 2025 (PMC12986856) — dynamic covariance weighting on sensor quality.
- *A fault-tolerant sensor fusion in mobile robots using multiple-model Kalman filters*, Robotics & Autonomous Systems 2022 — classical fault detection with intentional corruption (your non-learned baseline & methodology reference).
- Mortada et al., *Recursive KalmanNet*, arXiv:2506.11639 (2025) — uncertainty quantification in learned KF; supports your Option C framing.

Do a fresh Google Scholar / Semantic Scholar pass on: "learned measurement noise covariance EKF", "neural adaptive Kalman filter sensor reliability", "fault tolerant deep learning sensor fusion mobile robot" — confirm nobody published your exact fault-study framing before you commit.
