#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
import tf
import geometry_msgs.msg


class GroundTruthTFPublisher:
    def __init__(self):
        rospy.init_node('ground_truth_tf_publisher')

        self.robot_name = rospy.get_param('~robot_name', 'husky')  # change to match Gazebo model name
        self.frame_id = rospy.get_param('~base_frame_id', 'base_link')
        self.broadcaster = tf.TransformBroadcaster()

        self.last_stamp = None

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        rospy.on_shutdown(self.on_shutdown)

    def model_states_callback(self, msg):
        if not rospy.is_shutdown():
            try:
                index = msg.name.index(self.robot_name)
            except ValueError:
                rospy.logwarn_throttle(5.0, f"Model '{self.robot_name}' not found in /gazebo/model_states")
                return

            pose = msg.pose[index]

            now = rospy.Time.now()

            # Avoid publishing duplicate timestamp TFs
            if self.last_stamp is not None and now == self.last_stamp:
                return
            self.last_stamp = now

            try:
                self.broadcaster.sendTransform(
                    (pose.position.x, pose.position.y, pose.position.z),
                    (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                    now,
                    self.frame_id,
                    "world"
                )
            except rospy.exceptions.ROSException as e:
                rospy.logwarn(f"TF publish failed: {e}")

    def on_shutdown(self):
        rospy.loginfo("Shutting down TF publisher cleanly.")


if __name__ == '__main__':
    try:
        node = GroundTruthTFPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
