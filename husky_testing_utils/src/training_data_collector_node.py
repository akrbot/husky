#!/usr/bin/env python3
"""
training_data_collector_node.py
================================
Collects data for training the AI sensor reliability estimator.

The CSV has two clearly separated sections:

  SECTION A — MODEL INPUT FEATURES (17 columns)
  -----------------------------------------------
  These are the ONLY columns fed to the AI model at training AND inference.
  All are raw sensor signals — no EKF outputs, no ground truth.
  Matches exactly the feature table from the project spec.

  SECTION B — LABEL GENERATION COLUMNS (ground truth + per-sensor DR positions)
  -------------------------------------------------------------------------------
  Used OFFLINE by generate_training_dataset.py to compute soft reliability
  labels (windowed RMSE of each sensor vs ground truth).
  NEVER used as model inputs.

  This separation means:
    X  = Section A columns   (model sees this at training AND inference)
    Y  = computed from Section B columns  (only exists during training)

Usage
-----
  # Terminal 1 — launch Gazebo first
  ENABLE_EKF=false roslaunch husky_gazebo empty_world.launch

  # Terminal 2 — collect a clean run
  roslaunch husky_testing_utils collect_training_data.launch fault_label:=clean

  # Terminal 3 — drive the robot
  roslaunch husky_control teleop.launch

  # Ctrl+C in Terminal 2 when done.

Note: ENABLE_EKF=false is recommended for data collection — you don't need
the EKF running during collection, and it avoids confusion about which topics
are being recorded.  The IMU dead-reckoning node (for Section B) is started
by the launch file.

Feature index → column name mapping (Section A only)
------------------------------------------------------
 1  timestamp
 2  imu_ax          IMU linear acceleration x
 3  imu_ay          IMU linear acceleration y
 4  imu_az          IMU linear acceleration z
 5  imu_gx          IMU angular velocity x
 6  imu_gy          IMU angular velocity y
 7  imu_gz          IMU angular velocity z
 8  imu_cov         IMU measurement covariance (trace of orientation cov)
 9  odom_v          Odometry linear velocity
10  odom_w          Odometry angular velocity
11  odom_a          Odometry linear acceleration  (finite diff of odom_v)
12  odom_alpha      Odometry angular acceleration (finite diff of odom_w)
13  diff_imu_odom_w Difference: imu_gz − odom_w
14  diff_imu_odom_a Difference: imu_ax − odom_a  (forward axis)
15  diff_gps_odom_v Difference: GPS speed − odom_v  (0 if no GPS fix)
16  gps_hdop        GPS horizontal dilution of precision (99 if no fix)
17  gps_cov         GPS position covariance xx element  (999 if no fix)
"""

import os, csv, math
from datetime import datetime

import rospy
import tf.transformations as tft

from geometry_msgs.msg  import Twist, Vector3Stamped
from nav_msgs.msg       import Odometry
from sensor_msgs.msg    import Imu, NavSatFix
from gazebo_msgs.msg    import ModelStates


# ─────────────────────────────────────────────────────────────────────────────
# Helper
# ─────────────────────────────────────────────────────────────────────────────

def odom_to_xyyaw(msg):
    x   = msg.pose.pose.position.x
    y   = msg.pose.pose.position.y
    q   = msg.pose.pose.orientation
    _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return x, y, yaw


# ─────────────────────────────────────────────────────────────────────────────

class TrainingDataCollector:

    ROBOT_NAME = "husky"

    # Sentinel values used when GPS has no fix
    GPS_NO_FIX_HDOP = 99.0
    GPS_NO_FIX_COV  = 999.0
    GPS_NO_FIX_VEL  = 0.0

    def __init__(self):
        rospy.init_node("training_data_collector", anonymous=False)

        # ── Params ────────────────────────────────────────────────────────────
        self.fault_label = rospy.get_param("~fault_label",  "clean")
        self.sample_rate = rospy.get_param("~sample_rate",  50.0)
        output_dir       = rospy.get_param("~output_dir",
                               os.path.join(os.path.dirname(__file__),
                                            "../results"))
        os.makedirs(output_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(
            output_dir, f"training_{self.fault_label}_{stamp}.csv")

        # ── Section A state: raw sensor cache ─────────────────────────────────
        self._imu = dict(
            ax=0.0, ay=0.0, az=0.0,
            gx=0.0, gy=0.0, gz=0.0,
            cov=0.0,
            fresh=False)

        self._odom = dict(
            v=0.0, w=0.0,
            a=0.0, alpha=0.0,        # computed as finite differences
            prev_v=None, prev_w=None, prev_t=None,
            fresh=False)

        self._gps = dict(
            hdop=self.GPS_NO_FIX_HDOP,
            cov=self.GPS_NO_FIX_COV,
            speed=self.GPS_NO_FIX_VEL,
            fresh=False)

        # ── Section B state: position estimates for label generation only ──────
        self._gt     = dict(x=0.0, y=0.0, yaw=0.0, fresh=False)
        self._imu_dr = dict(x=0.0, y=0.0, yaw=0.0, fresh=False)  # /imu_only/odom
        self._odom_pos = dict(x=0.0, y=0.0, yaw=0.0)             # from /husky_velocity_controller/odom

        # ── Subscribers ───────────────────────────────────────────────────────

        # Section A
        rospy.Subscriber("/imu/data",
                         Imu, self._cb_imu, queue_size=5)

        rospy.Subscriber("/husky_velocity_controller/odom",
                         Odometry, self._cb_odom, queue_size=5)

        rospy.Subscriber("/navsat/fix",
                         NavSatFix, self._cb_gps_fix, queue_size=5)

        rospy.Subscriber("/navsat/vel",
                         Vector3Stamped, self._cb_gps_vel, queue_size=5)

        # Section B
        rospy.Subscriber("/gazebo/model_states",
                         ModelStates, self._cb_gt, queue_size=1)

        rospy.Subscriber("/imu_only/odom",
                         Odometry, self._cb_imu_dr, queue_size=5)

        # ── CSV setup ─────────────────────────────────────────────────────────
        self._csv_file   = open(self.csv_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._write_header()

        self._start_time = None
        self._row_count  = 0

        rospy.Timer(rospy.Duration(1.0 / self.sample_rate), self._write_row)
        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo(
            f"[Collector] fault_label='{self.fault_label}' → {self.csv_path}")
        rospy.loginfo(
            "[Collector] Waiting for IMU, odom, and ground truth...")

    # ── Section A callbacks ───────────────────────────────────────────────────

    def _cb_imu(self, msg):
        # Covariance: sensor_msgs/Imu.orientation_covariance is 3×3 row-major.
        # Use the trace (sum of diagonal) as a single scalar representing overall
        # orientation measurement uncertainty.
        cov_trace = (msg.orientation_covariance[0] +
                     msg.orientation_covariance[4] +
                     msg.orientation_covariance[8])
        self._imu.update(
            ax=msg.linear_acceleration.x,
            ay=msg.linear_acceleration.y,
            az=msg.linear_acceleration.z,
            gx=msg.angular_velocity.x,
            gy=msg.angular_velocity.y,
            gz=msg.angular_velocity.z,
            cov=cov_trace,
            fresh=True)

    def _cb_odom(self, msg):
        v_new = msg.twist.twist.linear.x
        w_new = msg.twist.twist.angular.z
        t_new = msg.header.stamp.to_sec()

        # Finite-difference accelerations
        a, alpha = 0.0, 0.0
        pv = self._odom["prev_v"]
        pw = self._odom["prev_w"]
        pt = self._odom["prev_t"]
        if pv is not None and pt is not None:
            dt = t_new - pt
            if dt > 1e-4:
                a     = (v_new - pv) / dt
                alpha = (w_new - pw) / dt

        x, y, yaw = odom_to_xyyaw(msg)

        self._odom.update(
            v=v_new, w=w_new, a=a, alpha=alpha,
            prev_v=v_new, prev_w=w_new, prev_t=t_new,
            fresh=True)
        # Also keep position for Section B labels
        self._odom_pos.update(x=x, y=y, yaw=yaw)

    def _cb_gps_fix(self, msg):
        # NavSatFix status: -1=no fix, 0=fix, 1=sbas fix, 2=gbas fix
        if msg.status.status < 0:
            self._gps.update(
                hdop=self.GPS_NO_FIX_HDOP,
                cov=self.GPS_NO_FIX_COV,
                fresh=False)
            return
        # position_covariance[0] = east variance (xx element)
        cov_xx = msg.position_covariance[0]
        # NavSatFix does not carry HDOP directly — the hector GPS plugin
        # does not populate it in NavSatFix.  We approximate from covariance:
        # hdop ≈ sqrt(cov_east) / 5.0  (rough mapping for sim; replace with
        # real HDOP from NavSatStatus if your GPS driver publishes it)
        hdop_approx = math.sqrt(max(cov_xx, 1e-9)) / 5.0
        self._gps.update(hdop=hdop_approx, cov=cov_xx, fresh=True)

    def _cb_gps_vel(self, msg):
        # /navsat/vel is a Vector3Stamped with ENU velocity (m/s)
        speed = math.sqrt(msg.vector.x**2 + msg.vector.y**2)
        self._gps.update(speed=speed)

    # ── Section B callbacks ───────────────────────────────────────────────────

    def _cb_gt(self, msg):
        try:
            idx = msg.name.index(self.ROBOT_NAME)
        except ValueError:
            return
        pose      = msg.pose[idx]
        q         = pose.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._gt.update(x=pose.position.x, y=pose.position.y,
                        yaw=yaw, fresh=True)

    def _cb_imu_dr(self, msg):
        x, y, yaw = odom_to_xyyaw(msg)
        self._imu_dr.update(x=x, y=y, yaw=yaw, fresh=True)

    # ── Timer: write one row ──────────────────────────────────────────────────

    def _write_row(self, _event):
        now = rospy.Time.now().to_sec()
        if self._start_time is None:
            self._start_time = now

        # Don't start writing until core topics have initialised
        if not (self._imu["fresh"]
                and self._odom["fresh"]
                and self._gt["fresh"]
                and self._imu_dr["fresh"]):
            return

        t   = now - self._start_time
        imu = self._imu
        odom = self._odom
        gps  = self._gps
        gt   = self._gt
        idr  = self._imu_dr
        opos = self._odom_pos

        # ── Section A: 17 model features ──────────────────────────────────────
        diff_imu_odom_w = imu["gz"]  - odom["w"]
        diff_imu_odom_a = imu["ax"]  - odom["a"]   # forward axis
        diff_gps_odom_v = gps["speed"] - odom["v"]

        sec_a = [
            round(t,               4),  # 1  timestamp
            round(imu["ax"],       6),  # 2  imu_ax
            round(imu["ay"],       6),  # 3  imu_ay
            round(imu["az"],       6),  # 4  imu_az
            round(imu["gx"],       6),  # 5  imu_gx
            round(imu["gy"],       6),  # 6  imu_gy
            round(imu["gz"],       6),  # 7  imu_gz
            round(imu["cov"],      9),  # 8  imu_cov
            round(odom["v"],       6),  # 9  odom_v
            round(odom["w"],       6),  # 10 odom_w
            round(odom["a"],       6),  # 11 odom_a
            round(odom["alpha"],   6),  # 12 odom_alpha
            round(diff_imu_odom_w, 6),  # 13 diff_imu_odom_w
            round(diff_imu_odom_a, 6),  # 14 diff_imu_odom_a
            round(diff_gps_odom_v, 6),  # 15 diff_gps_odom_v
            round(gps["hdop"],     4),  # 16 gps_hdop
            round(gps["cov"],      6),  # 17 gps_cov
        ]

        # ── Section B: label generation columns ───────────────────────────────
        sec_b = [
            # Ground truth
            round(gt["x"],    6), round(gt["y"],    6), round(gt["yaw"],  6),
            # IMU dead-reckoning position  (from /imu_only/odom)
            round(idr["x"],   6), round(idr["y"],   6), round(idr["yaw"], 6),
            # Odometry position  (integrated wheel encoder)
            round(opos["x"],  6), round(opos["y"],  6), round(opos["yaw"],6),
            # Fault tag
            self.fault_label,
        ]

        self._csv_writer.writerow(sec_a + sec_b)
        self._row_count += 1

        if self._row_count % 500 == 0:
            rospy.loginfo(
                f"[Collector] {self._row_count} rows | "
                f"{round(t,1)}s | fault={self.fault_label}")

    # ── Header ────────────────────────────────────────────────────────────────

    def _write_header(self):
        section_a = [
            # ── Section A: model input features (17) ──────────────────────
            "timestamp",
            "imu_ax", "imu_ay", "imu_az",      # raw IMU accel
            "imu_gx", "imu_gy", "imu_gz",      # raw IMU gyro
            "imu_cov",                          # IMU orientation cov trace
            "odom_v", "odom_w",                 # odom velocity
            "odom_a", "odom_alpha",             # odom acceleration (finite diff)
            "diff_imu_odom_w",                  # cross: gyro_z - odom_w
            "diff_imu_odom_a",                  # cross: imu_ax - odom_a
            "diff_gps_odom_v",                  # cross: gps_speed - odom_v
            "gps_hdop",                         # GPS quality
            "gps_cov",                          # GPS position covariance
        ]
        section_b = [
            # ── Section B: label generation only (never model inputs) ─────
            "gt_x", "gt_y", "gt_yaw",
            "imu_dr_x", "imu_dr_y", "imu_dr_yaw",
            "odom_pos_x", "odom_pos_y", "odom_pos_yaw",
            "fault_label",
        ]
        self._csv_writer.writerow(section_a + section_b)

    def _on_shutdown(self):
        self._csv_file.flush()
        self._csv_file.close()
        rospy.loginfo(
            f"[Collector] Done. {self._row_count} rows → {self.csv_path}")


if __name__ == "__main__":
    try:
        TrainingDataCollector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass