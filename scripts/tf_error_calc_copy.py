#!/usr/bin/env python
import rospy
import math
import tf2_ros
import tf_conversions
import numpy as np
from geometry_msgs.msg import TransformStamped

def transform_to_matrix(t):
    """
    Convert a geometry_msgs/TransformStamped to a 4x4 transformation matrix.
    """
    # Translation matrix
    trans = tf_conversions.transformations.translation_matrix(
        (t.transform.translation.x,
         t.transform.translation.y,
         t.transform.translation.z)
    )
    
    # Rotation matrix (from quaternion)
    rot = tf_conversions.transformations.quaternion_matrix(
        (t.transform.rotation.x,
         t.transform.rotation.y,
         t.transform.rotation.z,
         t.transform.rotation.w)
    )
    
    # The resulting transform is trans * rot
    return np.dot(trans, rot)

def main():
    rospy.init_node("drone_goal_distance_node", anonymous=True)

    # Create a TF buffer and listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        try:
            # Lookup the needed transforms
            t_map_hl2   = tf_buffer.lookup_transform("map",  "hl2",   rospy.Time())
            t_map_goal  = tf_buffer.lookup_transform("map",  "goal",  rospy.Time())
            t_hl2_aruco = tf_buffer.lookup_transform("hl2",  "aruco", rospy.Time())
            t_aruco_drone = tf_buffer.lookup_transform("aruco", "base_link", rospy.Time())
            
            # Convert to 4x4 matrices
            T_map_hl2   = transform_to_matrix(t_map_hl2)
            T_map_goal  = transform_to_matrix(t_map_goal)
            T_hl2_aruco = transform_to_matrix(t_hl2_aruco)
            T_aruco_drone = transform_to_matrix(t_aruco_drone)
            
            # Compose transforms to get map->drone
            T_map_drone = T_map_hl2.dot(T_hl2_aruco).dot(T_aruco_drone)
            
            # Extract translation from map->drone
            drone_x, drone_y, drone_z = T_map_drone[0, 3], T_map_drone[1, 3], T_map_drone[2, 3]
            
            # Extract translation from map->goal
            goal_x, goal_y, goal_z = T_map_goal[0, 3], T_map_goal[1, 3], T_map_goal[2, 3]
            
            # Compute Euclidean distance
            distance = math.sqrt((drone_x - goal_x)**2 +
                                 (drone_y - goal_y)**2 +
                                 (drone_z - goal_z)**2)
            
            rospy.loginfo("Distance between drone and goal: %f", distance)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # If a lookup or connectivity error occurs, just keep trying
            pass

        rate.sleep()

if __name__ == '__main__':
    main()
