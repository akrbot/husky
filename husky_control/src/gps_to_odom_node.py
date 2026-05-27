#!/usr/bin/env python3
"""Convert NavSatFix (lat/lon) to Odometry (x,y) in map frame.

Independent GPS position sensor — no dependency on odometry or IMU.
Uses flat-Earth approximation (valid for small areas < 10km).
"""
import math
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry


class GpsToOdom:
    def __init__(self):
        rospy.init_node('gps_to_odom')

        self.ref_lat = rospy.get_param('~ref_lat')
        self.ref_lon = rospy.get_param('~ref_lon')
        self.ref_lat_rad = math.radians(self.ref_lat)
        self.meters_per_deg_lat = 111319.0
        self.meters_per_deg_lon = 111319.0 * math.cos(self.ref_lat_rad)

        self.pub = rospy.Publisher('odometry/gps', Odometry, queue_size=10)
        rospy.Subscriber('/navsat/fix', NavSatFix, self.cb)
        rospy.loginfo("gps_to_odom: ref=(%.8f, %.8f), publishing /odometry/gps", self.ref_lat, self.ref_lon)

    def cb(self, msg):
        d_east = (msg.longitude - self.ref_lon) * self.meters_per_deg_lon
        d_north = (msg.latitude - self.ref_lat) * self.meters_per_deg_lat

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = d_east
        odom.pose.pose.position.y = d_north

        cov = msg.position_covariance[0] if msg.position_covariance[0] > 0 else 1.0
        odom.pose.covariance[0] = cov
        odom.pose.covariance[7] = cov
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = 1e6

        self.pub.publish(odom)


if __name__ == '__main__':
    GpsToOdom()
    rospy.spin()
