#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

class DualGPSPoseNode:
    METERS_PER_DEG_LAT = 111320.0

    def __init__(self):
        rospy.init_node("dual_gps_pose_node")

        # --- Load parameters from YAML ---
        self.ref_lat = rospy.get_param("/gps/ref_lat", 0.0)
        self.ref_lon = rospy.get_param("/gps/ref_lon", 0.0)
        self.topic_left = rospy.get_param("/gps/topic_navsat_fix1", "/navsat/fix1")
        self.topic_right = rospy.get_param("/gps/topic_navsat_fix2", "/navsat/fix2")
        self.frame_id = rospy.get_param("/pose/frame_id", "map")
        self.publish_topic = rospy.get_param("/pose/publish_topic", "/robot_mid_pose")

        self.meters_per_deg_lon = self.METERS_PER_DEG_LAT * math.cos(math.radians(self.ref_lat))

        self.left_gps = None
        self.right_gps = None

        # --- Setup publisher and subscribers ---
        self.pose_pub = rospy.Publisher(self.publish_topic, PoseStamped, queue_size=10)
        rospy.Subscriber(self.topic_left, NavSatFix, self.left_cb)
        rospy.Subscriber(self.topic_right, NavSatFix, self.right_cb)

        rospy.loginfo(f"[DualGPSPoseNode] Started with REF_LAT={self.ref_lat}, REF_LON={self.ref_lon}")
        rospy.loginfo(f"Front GPS topic: {self.topic_left}")
        rospy.loginfo(f"Back  GPS topic: {self.topic_right}")
        rospy.loginfo(f"Publishing pose on: {self.publish_topic}")

    # --- GPS to local XY conversion ---
    def gps_to_xy(self, lat, lon):
        x = (lat - self.ref_lat) * self.METERS_PER_DEG_LAT
        y = (self.ref_lon - lon) * self.meters_per_deg_lon

        return x, y

    # --- Compute robot midpoint and yaw ---
    def compute_pose(self):
        if self.left_gps is None or self.right_gps is None:
            return None

        fx, fy = self.gps_to_xy(self.left_gps.latitude, self.left_gps.longitude)
        bx, by = self.gps_to_xy(self.right_gps.latitude, self.right_gps.longitude)

        mx, my = (fx + bx) / 2.0, (fy + by) / 2.0
        yaw = math.atan2(fy - by, fx - bx) - math.pi / 2 #offset by 90 degrees because the gps points are in left and right
        # normalize to [-pi, pi]
        yaw = (yaw + math.pi) % (2 * math.pi) - math.pi

        yaw_deg = math.degrees(yaw)


        # Add 0.25814 meters to the x coordinate 
        # since antenna is -0.25814 meters away from the center (base_link)
        # check husky.urdf.xacro

        mx = mx + 0.25814 
        return mx, my, yaw

    def publish_pose(self):
        result = self.compute_pose()
        if result is None:
            return

        mx, my, yaw = result
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = mx
        pose_msg.pose.position.y = my

        q = quaternion_from_euler(0, 0, yaw)
        pose_msg.pose.orientation.x, pose_msg.pose.orientation.y = q[0], q[1]
        pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = q[2], q[3]

        self.pose_pub.publish(pose_msg)

    # --- Callbacks ---
    def left_cb(self, msg):
        self.left_gps = msg
        self.publish_pose()

    def right_cb(self, msg):
        self.right_gps = msg
        self.publish_pose()


if __name__ == "__main__":
    try:
        node = DualGPSPoseNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
