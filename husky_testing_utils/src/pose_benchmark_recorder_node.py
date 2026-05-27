#!/usr/bin/env python3

import os
import csv
import rospy
from datetime import datetime
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class PoseBenchmarkRecorder:
    def __init__(self):
        rospy.init_node('pose_benchmark_recorder', anonymous=False)

        self.output_dir = rospy.get_param('~output_dir', '')
        if not self.output_dir:
            import rospkg
            pkg_path = rospkg.RosPack().get_path('husky_testing_utils')
            self.output_dir = os.path.join(pkg_path, 'results')

        self.duration = rospy.get_param('~duration', 0.0)
        self.robot_name = rospy.get_param('~robot_name', 'husky')
        self.sample_rate = rospy.get_param('~sample_rate', 50.0)

        os.makedirs(self.output_dir, exist_ok=True)

        self.latest_gt = None
        self.latest_odom = None
        self.latest_imu = None
        self.latest_ekf = None
        self.latest_ekf_global = None

        self.data_rows = []
        self.start_time = None

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.gt_callback)
        rospy.Subscriber('/husky_velocity_controller/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/imu_only/odom', Odometry, self.imu_callback)
        rospy.Subscriber('/odometry/filtered', Odometry, self.ekf_callback)
        rospy.Subscriber('/odometry/filtered_map', Odometry, self.ekf_global_callback)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.sample_rate), self.sample_callback)

        if self.duration > 0:
            rospy.Timer(rospy.Duration(self.duration), self.duration_callback, oneshot=True)

        rospy.on_shutdown(self.save_csv)
        rospy.loginfo("Benchmark recorder started (rate=%.1f Hz, output=%s)", self.sample_rate, self.output_dir)

    def gt_callback(self, msg):
        try:
            idx = msg.name.index(self.robot_name)
            self.latest_gt = msg.pose[idx]
        except ValueError:
            pass

    def odom_callback(self, msg):
        self.latest_odom = msg.pose.pose

    def imu_callback(self, msg):
        self.latest_imu = msg.pose.pose

    def ekf_callback(self, msg):
        self.latest_ekf = msg.pose.pose

    def ekf_global_callback(self, msg):
        self.latest_ekf_global = msg.pose.pose

    def extract_xyyaw(self, pose):
        if pose is None:
            return (float('nan'), float('nan'), float('nan'))
        q = pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return (pose.position.x, pose.position.y, yaw)

    def sample_callback(self, event):
        if self.start_time is None:
            self.start_time = rospy.Time.now()

        t = (rospy.Time.now() - self.start_time).to_sec()
        gt = self.extract_xyyaw(self.latest_gt)
        odom = self.extract_xyyaw(self.latest_odom)
        imu = self.extract_xyyaw(self.latest_imu)
        ekf = self.extract_xyyaw(self.latest_ekf)
        ekf_global = self.extract_xyyaw(self.latest_ekf_global)

        self.data_rows.append([t] + list(gt) + list(odom) + list(imu) + list(ekf) + list(ekf_global))

    def duration_callback(self, event):
        rospy.loginfo("Benchmark duration (%.1f s) reached, shutting down.", self.duration)
        rospy.signal_shutdown("Duration reached")

    def save_csv(self):
        if not self.data_rows:
            rospy.logwarn("No data recorded.")
            return

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(self.output_dir, 'benchmark_{}.csv'.format(timestamp))

        header = [
            'timestamp',
            'gt_x', 'gt_y', 'gt_yaw',
            'odom_x', 'odom_y', 'odom_yaw',
            'imu_x', 'imu_y', 'imu_yaw',
            'ekf_x', 'ekf_y', 'ekf_yaw',
            'ekf_global_x', 'ekf_global_y', 'ekf_global_yaw'
        ]

        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(self.data_rows)

        rospy.loginfo("Saved %d rows to %s", len(self.data_rows), filename)


if __name__ == '__main__':
    try:
        node = PoseBenchmarkRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
