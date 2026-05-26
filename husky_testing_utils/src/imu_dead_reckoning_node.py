#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_matrix, quaternion_from_euler


class IMUDeadReckoning:
    def __init__(self):
        rospy.init_node('imu_dead_reckoning', anonymous=False)

        self.gravity = rospy.get_param('~gravity', 9.81)
        self.initial_x = rospy.get_param('~initial_x', 0.0)
        self.initial_y = rospy.get_param('~initial_y', 0.0)
        self.initial_yaw = rospy.get_param('~initial_yaw', 0.0)
        self.position_clamp = 1000.0

        self.x = self.initial_x
        self.y = self.initial_y
        self.vx = 0.0
        self.vy = 0.0
        self.yaw = self.initial_yaw
        self.prev_time = None

        self.odom_pub = rospy.Publisher('/imu_only/odom', Odometry, queue_size=10)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        rospy.loginfo("IMU dead reckoning node started (gravity=%.3f)", self.gravity)

    def imu_callback(self, msg):
        current_time = msg.header.stamp

        if self.prev_time is None:
            self.prev_time = current_time
            return

        dt = (current_time - self.prev_time).to_sec()
        if dt <= 0 or dt > 0.1:
            self.prev_time = current_time
            return

        self.prev_time = current_time

        q = msg.orientation
        (_, _, self.yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        a_body = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        a_world = R @ a_body
        a_world[2] -= self.gravity

        ax = a_world[0]
        ay = a_world[1]

        self.vx += ax * dt
        self.vy += ay * dt
        self.x += self.vx * dt
        self.y += self.vy * dt

        self.x = np.clip(self.x, -self.position_clamp, self.position_clamp)
        self.y = np.clip(self.y, -self.position_clamp, self.position_clamp)

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q_out = quaternion_from_euler(0.0, 0.0, self.yaw)
        odom.pose.pose.orientation = Quaternion(*q_out)

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = msg.angular_velocity.z

        self.odom_pub.publish(odom)


if __name__ == '__main__':
    try:
        node = IMUDeadReckoning()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
