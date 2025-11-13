#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler

def generate_square_wave_path(start_x, start_y, path_length, row_height, num_rows):
    """
    Generates a list of PoseStamped waypoints for a square wave path with 1m spacing.
    
    Args:
        start_x (float): The starting x coordinate of the path.
        start_y (float): The starting y coordinate of the path.
        path_length (float): The length of each horizontal segment (l).
        row_height (float): The height of each vertical segment (h).
        num_rows (int): The number of horizontal segments (r).

    Returns:
        list[PoseStamped]: A list of PoseStamped objects representing the path.
    """
    path_waypoints = []
    current_x = start_x
    current_y = start_y
    
    # Add the initial starting point
    start_pose = PoseStamped()
    start_pose.header.frame_id = "world"
    start_pose.pose.position.x = current_x
    start_pose.pose.position.y = current_y
    start_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))
    path_waypoints.append(start_pose)

    for i in range(num_rows):
        # Generate waypoints for the horizontal segment
        horizontal_direction = 1.0 if i % 2 == 0 else -1.0
        for j in np.arange(1.0, path_length + 1.0, 1.0):
            current_x = start_x + (horizontal_direction * j)
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.pose.position.x = current_x
            pose.pose.position.y = current_y
            pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))
            path_waypoints.append(pose)
        
        # Update the start_x for the next row
        start_x = current_x

        # If it's not the last row, generate waypoints for the vertical segment
        if i < num_rows - 1:
            for k in np.arange(1.0, row_height + 1.0, 1.0):
                current_y = start_y + k
                pose = PoseStamped()
                pose.header.frame_id = "world"
                pose.pose.position.x = current_x
                pose.pose.position.y = current_y
                pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))
                path_waypoints.append(pose)

            # Update the start_y for the next row
            start_y = current_y
            
    rospy.loginfo(f"Generated a path with {len(path_waypoints)} waypoints.")
    return path_waypoints

def path_publisher_node():
    rospy.init_node('path_publisher', anonymous=True)
    
    path_pub = rospy.Publisher('/pure_pursuit/path', Path, queue_size=1, latch=True)
    
    start_x = rospy.get_param("~start_x", -1.0)
    start_y = rospy.get_param("~start_y", 0.0)
    path_length = rospy.get_param("~path_length", 13.0)
    row_height = rospy.get_param("~row_height", 1.0)
    num_rows = rospy.get_param("~num_rows", 4)
    
    waypoints = generate_square_wave_path(start_x, start_y, path_length, row_height, num_rows)
    
    path_msg = Path()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = "world"
    path_msg.poses = waypoints
    
    rospy.loginfo("Publishing the generated path...")
    path_pub.publish(path_msg)
    
    rospy.loginfo("Path published. Use Rviz to visualize the path topic: /pure_pursuit/path")

    rospy.spin()

if __name__ == '__main__':
    try:
        path_publisher_node()
    except rospy.ROSInterruptException:
        pass