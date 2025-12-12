#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
import math
import numpy as np

class PurePursuitController:
    def __init__(self):
        rospy.init_node('pure_pursuit_controller', anonymous=True)

        # Load parameters from a YAML file or set them here
        self.lookahead_distance = rospy.get_param("~lookahead_distance", 0.5)
        self.target_velocity = rospy.get_param("~target_velocity", 0.5)
        self.robot_name = rospy.get_param("~robot_name", "X100") # Ensure this matches your Gazebo model name

        self.robot_pose = None
        self.path_waypoints = []

        # ROS Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/husky/cmd_vel', Twist, queue_size=1)
        
        # Subscribe to Gazebo model states for ground truth pose
        # rospy.Subscriber('/gazebo/model_states', ModelStates, self.pose_callback_gazebo)

        rospy.Subscriber('/robot_mid_pose', PoseStamped, self.robot_pose_callback)
        
        # You could also subscribe to a path topic if you want to publish paths dynamically
        rospy.Subscriber('/pure_pursuit/path', Path, self.path_callback)

        self.arrow_pub = rospy.Publisher("/lookahead_arrow", Marker, queue_size=10)



        self.last_waypoint_index = 0

        # Set the control loop frequency (Hz)
        self.control_rate = rospy.Rate(10) # 10 Hz is a good starting point for testing


        self.v_min = 0.1  # Minimum speed to avoid stopping
        self.v_max = self.target_velocity

    def robot_pose_callback(self, msg):
        """
        Callback to update the robot's pose from the PoseStamped message.
        """
        self.robot_pose = msg.pose
        # rospy.loginfo(f"Updated robot pose: {self.robot_pose.position.x}, {self.robot_pose.position.y}, {self.robot_pose.orientation.z}")

    def pose_callback_gazebo(self, msg):
        try:
            robot_index = msg.name.index(self.robot_name)
            self.robot_pose = msg.pose[robot_index]
        except ValueError:
            rospy.logwarn_once(f"Robot '{self.robot_name}' not found in Gazebo model states.")

    def path_callback(self, msg):
        self.path_waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        rospy.loginfo(f"Received path with {len(self.path_waypoints)} waypoints.")


    def find_nearest_waypoint(self):
        """
        Get the index of the waypoint closest to the vehicle.
        """
        if not self.path_waypoints:
            return None
        
        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y
        
        current_xy = np.array([robot_x, robot_y])
        waypoints_xy = np.array(self.path_waypoints)
        
        # Calculate the squared distance to all waypoints
        distances_squared = np.sum((current_xy - waypoints_xy)**2, axis=1)
        
        # Find the index of the minimum distance
        nearest_idx = np.argmin(distances_squared)
        
        return nearest_idx

    # This uses the function above to find the goal
    def get_lookahead_goal(self):
        """
        Finds the lookahead point on the path, updates the last_waypoint_index,
        and checks if the end goal has been reached.
        """
        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y
        
        lookahead_point = None
        distance = None
        reached_end = False # Initialize the flag to False
        
        # Start searching from the last known point on the path
        for i in range(self.last_waypoint_index, len(self.path_waypoints)):
            waypoint_x, waypoint_y = self.path_waypoints[i]
            distance = math.hypot(waypoint_x - robot_x, waypoint_y - robot_y)
            
            # Find the first point that is at or beyond the lookahead distance
            if distance >= self.lookahead_distance:
                lookahead_point = (waypoint_x, waypoint_y)
                # THIS IS THE KEY UPDATE:
                # We found a lookahead point, so we set the last_waypoint_index
                # to its current index 'i' to prevent the next search from starting
                # from the beginning of the path.
                self.last_waypoint_index = i
                rospy.loginfo_throttle(1, f"New Lookahead Goal Found at index: {i}")
                return lookahead_point, distance, reached_end

        # If no point is far enough, use the last point of the path as the target
        if len(self.path_waypoints) > 0:
            lookahead_point = self.path_waypoints[-1]
            rospy.loginfo_throttle(1, f"Using the last waypoint as goal at index: {len(self.path_waypoints) - 1}")
            
            # Check if the robot has reached the final waypoint within a tolerance
            distance_to_final = math.hypot(lookahead_point[0] - robot_x, lookahead_point[1] - robot_y)
            if distance_to_final < 0.5: # 0.5 meters is a common tolerance
                reached_end = True
        
        return lookahead_point, distance, reached_end
    

    def publish_goal(self, goal_x, goal_y, robot_x, robot_y):
        # Publish arrow from robot to lookahead goal
        arrow_marker = Marker()
        arrow_marker.header.stamp = rospy.Time.now()
        arrow_marker.header.frame_id = "world"  # or "odom"

        arrow_marker.ns = "lookahead"
        arrow_marker.id = 0
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD

        # Arrow points (start = robot, end = goal)
        start_point = Point()
        start_point.x = robot_x
        start_point.y = robot_y
        start_point.z = 0.0

        end_point = Point()
        end_point.x = goal_x
        end_point.y = goal_y
        end_point.z = 0.0

        arrow_marker.points.append(start_point)
        arrow_marker.points.append(end_point)

        # Arrow styling
        arrow_marker.scale.x = 0.05  # shaft diameter
        arrow_marker.scale.y = 0.1   # head diameter
        arrow_marker.scale.z = 0.1   # head length
        arrow_marker.color.r = 1.0
        arrow_marker.color.g = 0.0
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 1.0
        self.arrow_pub.publish(arrow_marker)

    def run_pure_pursuit(self):
        if self.robot_pose is None or not self.path_waypoints:
            rospy.loginfo_throttle(1, "Waiting for robot pose or waypoints...")
            return

        # ---- PURE PURSUIT LOGIC IMPLEMENTATION ----

        # 1. Get the current robot pose (x, y, yaw)
        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y
        orientation_q = self.robot_pose.orientation
        (_, _, robot_yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        
        rospy.loginfo_throttle(1, f"Robot Pose: x={robot_x:.2f}, y={robot_y:.2f}, yaw={robot_yaw:.2f}")
        # 2. Find the nearest waypoint
        nearest_idx = self.find_nearest_waypoint()
        lookahead_goal, lookahead_distance, is_end = self.get_lookahead_goal() 
        rospy.loginfo_throttle(1, f"Nearest waypoint index: {nearest_idx}, nearest goal index: {lookahead_goal}")
        rospy.loginfo_throttle(1, f"Lookahead Goal: {lookahead_goal}, Lookahead Distance: {lookahead_distance}")

        if is_end:
            rospy.loginfo("Reached the end of the path. Stopping the robot.")
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
            return
        
        if lookahead_goal is None or lookahead_distance is None:
            return

        #COMPUTE ALPHA
        (goal_x, goal_y) = lookahead_goal
        # Publish lookahead goal for RViz
        self.publish_goal(goal_x, goal_y, robot_x, robot_y)

        # Calculate the angle to the goal relative to the robot's orientation
        # alpha is the angle between the robot's heading and the line to the goal
        # This is the pure pursuit angle
        alpha = math.atan2(goal_y - robot_y, goal_x - robot_x) - robot_yaw
        
        # Normalize alpha to be within [-pi, pi]
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi
        
        rospy.loginfo_throttle(1, f"Alpha: {alpha:.2f} radians, {math.degrees(alpha):.2f} degrees")

        # Curvature and angular velocity
        target_velocity = max(self.v_min, self.v_max * math.cos(alpha))
        angular_velocity = (3 * target_velocity * math.sin(alpha)) / self.lookahead_distance

        max_omega_for_speed = 2.0 # A configurable parameter for max turning rate
        mid_omega = 0.5 * max_omega_for_speed
        # Use the absolute value of omega to handle both left and right turns
        if abs(angular_velocity) > max_omega_for_speed:
            # Reduce linear velocity when turning sharply
            # The scaling factor can be tuned (e.g., 0.1, 0.2, etc.)
            linear_velocity = target_velocity * 0.2
        elif abs(angular_velocity) > mid_omega:
            # Reduce linear velocity when the turn is moderate
            linear_velocity = target_velocity * 0.5
        else:
            # Use the full target velocity when the turn is not sharp
            linear_velocity = target_velocity


        rospy.loginfo_throttle(1, f"Linear Velocity: {linear_velocity:.2f}, Angular Velocity: {angular_velocity:.2f}")


        # 3. Publish the velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_velocity
        cmd_vel.angular.z = angular_velocity
        
        self.cmd_vel_pub.publish(cmd_vel)
        rospy.loginfo_throttle(1, f"Published cmd_vel: linear={cmd_vel.linear.x}, angular={cmd_vel.angular.z}")


    def main_loop(self):
        while not rospy.is_shutdown():
            self.run_pure_pursuit()
            self.control_rate.sleep()

if __name__ == '__main__':
    try:
        controller = PurePursuitController()
                
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass