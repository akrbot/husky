#!/usr/bin/env python3
"""Calibrate the diff_drive wheel_separation_multiplier for the skid-steer Husky.

The Husky is a 4-wheel skid-steer driven by diff_drive_controller (a 2-wheel
model), so its wheel odometry reports the wrong yaw rate during turns. The
`wheel_separation_multiplier` in control.yaml compensates for this. This node
finds the value that makes wheel-odom rotation match reality, in *this* sim:

  1. spins the robot in place for a fixed time,
  2. integrates the yaw from wheel odometry and from Gazebo ground truth,
  3. prints the recommended multiplier:

         new_multiplier = current_multiplier * (odom_yaw / ground_truth_yaw)

Run with the sim unpaused and clear space around the robot (it spins in place).

Note: a single multiplier cannot be exact for a skid-steer across all motions -
an in-place spin (linear_speed=0) is maximum skid, while an arc (linear_speed>0)
skids less and is closer to normal driving. Calibrate with a motion profile that
matches how you actually drive.

Params:
  ~cmd_vel_topic       (default: joy_teleop/cmd_vel)  highest-priority twist_mux input
  ~odom_topic          (default: /husky_velocity_controller/odom)
  ~gt_topic            (default: /gazebo/model_states)
  ~robot_name          (default: husky)
  ~current_multiplier  (default: read from the controller param, else 1.0)
  ~linear_speed        (default: 0.0 m/s)   >0 drives an arc instead of a pure spin
  ~angular_speed       (default: 0.6 rad/s)
  ~duration            (default: 15.0 s)
"""

import math
import threading

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class YawIntegrator:
    """Accumulates total (unwrapped) rotation from a stream of yaw readings."""

    def __init__(self):
        self.lock = threading.Lock()
        self.total = 0.0
        self.last = None
        self.have_data = False

    def add(self, yaw):
        with self.lock:
            if self.last is not None:
                self.total += wrap(yaw - self.last)
            self.last = yaw
            self.have_data = True

    def reset(self):
        with self.lock:
            self.total = 0.0
            self.last = None

    def total_rotation(self):
        with self.lock:
            return self.total

    def ready(self):
        with self.lock:
            return self.have_data


class WheelSeparationCalibrator:
    def __init__(self):
        self.cmd_topic = rospy.get_param("~cmd_vel_topic", "joy_teleop/cmd_vel")
        self.odom_topic = rospy.get_param("~odom_topic", "/husky_velocity_controller/odom")
        self.gt_topic = rospy.get_param("~gt_topic", "/gazebo/model_states")
        self.robot_name = rospy.get_param("~robot_name", "husky")
        self.linear_speed = float(rospy.get_param("~linear_speed", 0.0))
        self.angular_speed = float(rospy.get_param("~angular_speed", 0.6))
        self.duration = float(rospy.get_param("~duration", 15.0))
        # current multiplier: prefer the value the controller actually loaded
        self.current_mult = float(rospy.get_param(
            "~current_multiplier",
            rospy.get_param("/husky_velocity_controller/wheel_separation_multiplier", 1.0)))

        self.odom_yaw = YawIntegrator()
        self.gt_yaw = YawIntegrator()

        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=50)
        rospy.Subscriber(self.gt_topic, ModelStates, self.gt_cb, queue_size=50)

    def odom_cb(self, msg):
        self.odom_yaw.add(yaw_from_quat(msg.pose.pose.orientation))

    def gt_cb(self, msg):
        try:
            i = msg.name.index(self.robot_name)
        except ValueError:
            rospy.logwarn_throttle(
                5.0, "model '%s' not in %s" % (self.robot_name, self.gt_topic))
            return
        self.gt_yaw.add(yaw_from_quat(msg.pose[i].orientation))

    def _publish_twist(self, wz, vx=0.0):
        t = Twist()
        t.linear.x = vx
        t.angular.z = wz
        self.cmd_pub.publish(t)

    def _stop(self):
        for _ in range(10):
            self._publish_twist(0.0)
            rospy.sleep(0.05)

    def run(self):
        rospy.loginfo("Waiting for odometry and ground truth data...")
        r = rospy.Rate(20)
        while not rospy.is_shutdown() and not (self.odom_yaw.ready() and self.gt_yaw.ready()):
            r.sleep()
        if rospy.is_shutdown():
            return

        # start fresh from this moment
        self.odom_yaw.reset()
        self.gt_yaw.reset()
        # let the integrators capture a baseline sample
        rospy.sleep(0.3)
        self.odom_yaw.reset()
        self.gt_yaw.reset()

        motion = "arc (vx=%.2f m/s, wz=%.2f rad/s)" % (self.linear_speed, self.angular_speed) \
            if self.linear_speed > 0.0 else "in-place spin at %.2f rad/s" % self.angular_speed
        rospy.loginfo("Driving %s for %.1f s (cmd: %s)...", motion, self.duration, self.cmd_topic)
        start = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < self.duration:
            self._publish_twist(self.angular_speed, self.linear_speed)
            r.sleep()

        self._stop()
        rospy.sleep(0.3)  # let the last messages land

        odom_rot = self.odom_yaw.total_rotation()
        gt_rot = self.gt_yaw.total_rotation()
        self.report(odom_rot, gt_rot)

    def report(self, odom_rot, gt_rot):
        line = "=" * 64
        rospy.loginfo("\n%s\nWHEEL SEPARATION CALIBRATION RESULT\n%s", line, line)
        rospy.loginfo("wheel-odom rotated : %+.4f rad  (%.2f turns)",
                      odom_rot, odom_rot / (2 * math.pi))
        rospy.loginfo("ground-truth rot.  : %+.4f rad  (%.2f turns)",
                      gt_rot, gt_rot / (2 * math.pi))
        rospy.loginfo("current multiplier : %.4f", self.current_mult)

        if abs(gt_rot) < 0.2:
            rospy.logwarn("Ground-truth rotation is tiny (%.4f rad). The robot barely "
                          "moved - check the cmd_vel topic / twist_mux priority / e_stop, "
                          "then rerun.", gt_rot)
            return
        if abs(odom_rot) < 1e-6:
            rospy.logwarn("No wheel-odom rotation captured - is %s publishing?",
                          self.odom_topic)
            return

        ratio = odom_rot / gt_rot
        recommended = self.current_mult * ratio
        rospy.loginfo("odom/ground-truth  : %.4f", ratio)
        rospy.loginfo("%s", line)
        rospy.loginfo(">> RECOMMENDED wheel_separation_multiplier: %.3f", recommended)
        rospy.loginfo("   (set this in husky_control/config/control.yaml, then relaunch)")
        if ratio > 1.0:
            rospy.loginfo("   wheel odom over-reported rotation -> multiplier increased")
        else:
            rospy.loginfo("   wheel odom under-reported rotation -> multiplier decreased")
        rospy.loginfo("%s", line)


def main():
    rospy.init_node("calibrate_wheel_separation")
    calib = WheelSeparationCalibrator()
    try:
        calib.run()
    finally:
        calib._stop()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
