#!/usr/bin/env python

import rospy
import tf
import math
import numpy as np
import tf.transformations as tf_trans
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage

## Node that DOES NOT changes the frame convention at all!

# Variables to store transforms internally
aruco_drone_transform = None
aruco_hl2_transform = None
hl2_pose_transform = None
goal_pose_transform = None

tf_broadcaster = tf.TransformBroadcaster()

# Updates the D and P matrix values of the HL2 Camera Info topic
# Publishes on a new topic then
def camera_info_callback(camera_info_msg):
    camera_info_msg.D = [0.0355633167278393, -0.123584193169489, -0.000250953321849423, -0.000181034148553509, 0.155155222181198]
    camera_info_msg.P = [2992.533447265625, 0.0, 1917.8743896484375, 0.0, 0.0, 2994.665771484375, 1038.7142333984375, 0.0, 0.0, 0.0, 1.0, 0.0]
    hl2_camera_info_pub.publish(camera_info_msg)
    print("hl2 camera info published")

# Subscribes to the aruco_drone pose topic
# Inverts and saves the aruco_marker_frame -> drone transform
def aruco_drone_pose_callback(msg):
    global aruco_drone_transform

    marker_pose = msg.pose
    marker_position = (marker_pose.position.x, marker_pose.position.y, marker_pose.position.z)
    marker_orientation = (marker_pose.orientation.x, marker_pose.orientation.y, marker_pose.orientation.z, marker_pose.orientation.w)

    marker_transform = tf_trans.concatenate_matrices(
        tf_trans.translation_matrix(marker_position),
        tf_trans.quaternion_matrix(marker_orientation)
    )
    
    aruco_drone_transform = tf_trans.inverse_matrix(marker_transform)


    # Extract the translation and rotation from the inverse transformation matrix
    aruco_drone_translation = tf_trans.translation_from_matrix(aruco_drone_transform)
    aruco_drone_rotation = tf_trans.quaternion_from_matrix(aruco_drone_transform)

    aruco_drone_transform = {
        "translation": aruco_drone_translation.tolist(),
        "rotation": aruco_drone_rotation.tolist(),
        "timestamp": msg.header.stamp.to_sec()
    }

    # print("aruco-base_link transform recorded")

# Subscribes to the /Player0/camera/pose topic
# Saves the map -> hl2 transform
# Broadcasts
def hl2_pose_callback(msg):
    global hl2_pose_transform

    hl2_pose = msg.pose

    ## converting ROS to CV

    # hl2_pose = ros_to_cv_pose_stamped(msg).pose

    hl2_position = (hl2_pose.position.x, hl2_pose.position.y, hl2_pose.position.z)
    hl2_orientation = (hl2_pose.orientation.x, hl2_pose.orientation.y, hl2_pose.orientation.z, hl2_pose.orientation.w)

    # hl2_position = (hl2_pose.position.z, -hl2_pose.position.x, -hl2_pose.position.y)
    # hl2_orientation = (hl2_pose.orientation.x, hl2_pose.orientation.y, hl2_pose.orientation.z, hl2_pose.orientation.w)


    hl2_pose_transform = tf_trans.concatenate_matrices(
        tf_trans.translation_matrix(hl2_position),
        tf_trans.quaternion_matrix(hl2_orientation)
    )

    # Extract the translation and rotation from the transformation matrix
    hl2_translation = tf_trans.translation_from_matrix(hl2_pose_transform)
    hl2_rotation = tf_trans.quaternion_from_matrix(hl2_pose_transform)

    hl2_pose_transform = {
        "translation": hl2_translation.tolist(),
        "rotation": hl2_rotation.tolist(),
        "timestamp": msg.header.stamp.to_sec()
    }

# Subscribes to the /tello/command_pos_world topic
# Saves the map -> goal transform
def goal_pose_callback(msg):
    global goal_pose_transform

    goal_pose = msg.pose

     ## converting ROS to CV

    # goal_pose = ros_to_cv_pose_stamped(msg).pose

    goal_position = (goal_pose.position.x, goal_pose.position.y, goal_pose.position.z)
    goal_orientation = (goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w)

    goal_pose_transform = tf_trans.concatenate_matrices(
        tf_trans.translation_matrix(goal_position),
        tf_trans.quaternion_matrix(goal_orientation)
    )


    # Extract the translation and rotation from the transformation matrix
    goal_translation = tf_trans.translation_from_matrix(goal_pose_transform)
    goal_rotation = tf_trans.quaternion_from_matrix(goal_pose_transform)

    goal_pose_transform = {
        "translation": goal_translation.tolist(),
        "rotation": goal_rotation.tolist(),
        "timestamp": msg.header.stamp.to_sec()
    }

    # print("map-goal transform recorded")

# Subscribes to the aruco_hl2 pose topic
# Saves the hl2 -> aruco_marker_frame transform
def aruco_hl2_pose_callback(msg):
    global aruco_hl2_transform

    hl2_pose = msg.pose
    hl2_position = (hl2_pose.position.x, hl2_pose.position.y, hl2_pose.position.z)
    hl2_orientation = (hl2_pose.orientation.x, hl2_pose.orientation.y, hl2_pose.orientation.z, hl2_pose.orientation.w)
    
    aruco_hl2_transform = tf_trans.concatenate_matrices(
        tf_trans.translation_matrix(hl2_position),
        tf_trans.quaternion_matrix(hl2_orientation)
    )

    aruco_hl2_translation = tf_trans.translation_from_matrix(aruco_hl2_transform)
    aruco_hl2_rotation = tf_trans.quaternion_from_matrix(aruco_hl2_transform)

    aruco_hl2_transform = {
        "translation": aruco_hl2_translation.tolist(),
        "rotation": aruco_hl2_rotation.tolist(),
        "timestamp": msg.header.stamp.to_sec()
    }

    publish_transforms()

    # print("hl2-aruco transform recorded")

def publish_transforms():
    """
    Publishes the aruco -> goal and aruco -> base_link transforms 
    once all required transforms are available.
    """
    global aruco_drone_transform, aruco_hl2_transform, hl2_pose_transform, goal_pose_transform

    # Check if all the required transforms are not None
    if aruco_drone_transform and aruco_hl2_transform and hl2_pose_transform and goal_pose_transform:
        # Convert saved dictionary values back to transformation matrices

        hl2_to_aruco_matrix = tf_trans.concatenate_matrices(
            tf_trans.translation_matrix(aruco_hl2_transform['translation']),
            tf_trans.quaternion_matrix(aruco_hl2_transform['rotation'])
        )

        map_to_hl2_matrix = tf_trans.concatenate_matrices(
            tf_trans.translation_matrix(hl2_pose_transform['translation']),
            tf_trans.quaternion_matrix(hl2_pose_transform['rotation'])
        )

        map_to_goal_matrix = tf_trans.concatenate_matrices(
            tf_trans.translation_matrix(goal_pose_transform['translation']),
            tf_trans.quaternion_matrix(goal_pose_transform['rotation'])
        )

        # aruco_to_goal_matrix = tf_trans.inverse_matrix(map_to_goal_matrix) @ map_to_hl2_matrix @ hl2_to_aruco_matrix


        ## TESTING

        hl2_to_aruco_matrix_translation = tf_trans.translation_from_matrix(hl2_to_aruco_matrix)
        hl2_to_aruco_matrix_rotation = tf_trans.quaternion_from_matrix(hl2_to_aruco_matrix)

        tf_broadcaster.sendTransform(
            tuple(hl2_to_aruco_matrix_translation),  # Translation as a tuple
            tuple(hl2_to_aruco_matrix_rotation),     # Rotation as a tuple
            rospy.Time.now(),                    # Current time
            "aruco",                         # Child frame
            "hl2"                              # Parent frame
        )

        map_to_hl2_matrix_matrix_translation = tf_trans.translation_from_matrix(map_to_hl2_matrix)
        map_to_hl2_matrix_matrix_rotation = tf_trans.quaternion_from_matrix(map_to_hl2_matrix)

        tf_broadcaster.sendTransform(
            tuple(map_to_hl2_matrix_matrix_translation),  # Translation as a tuple
            tuple(map_to_hl2_matrix_matrix_rotation),     # Rotation as a tuple
            rospy.Time.now(),                    # Current time
            "hl2",                         # Child frame
            "map"                              # Parent frame
        )

        # Extract translation and rotation for aruco -> base_link
        aruco_base_link_translation = aruco_drone_transform['translation']
        aruco_base_link_rotation = aruco_drone_transform['rotation']

        # Publish the aruco -> base_link transform
        tf_broadcaster.sendTransform(
            tuple(aruco_base_link_translation),  # Translation as a tuple
            tuple(aruco_base_link_rotation),     # Rotation as a tuple
            rospy.Time.now(),                    # Current time
            "base_link",                         # Child frame
            "aruco"                              # Parent frame
        )

        # # Extract translation and rotation for map -> goal
        map_to_goal_matrix_translation = tf_trans.translation_from_matrix(map_to_goal_matrix)
        map_to_goal_matrix_rotation = tf_trans.quaternion_from_matrix(map_to_goal_matrix)

        # Publish the aruco -> goal transform
        tf_broadcaster.sendTransform(
            tuple(map_to_goal_matrix_translation),   # Translation as a tuple
            tuple(map_to_goal_matrix_rotation),      # Rotation as a tuple
            rospy.Time.now(),                # Current time
            "goal",                          # Child frame
            "map"                          # Parent frame
        )

        print("map -> hl2 and hl2 -> aruco transforms published")
    else:
        print("Waiting for all required transforms to be available...")

if __name__ == '__main__':
    rospy.init_node('drone_pose_with_respect_to_marker')

    # rospy.set_param('use_sim_time', True)

    # Publishers and subscribers
    hl2_camera_info_pub = rospy.Publisher('/Player0/camera/camera_info/updated', CameraInfo, queue_size=10)
    rospy.Subscriber('/Player0/camera/camera_info', CameraInfo, camera_info_callback) 
    
    rospy.Subscriber('/aruco_drone/pose', PoseStamped, aruco_drone_pose_callback)
    rospy.Subscriber('/aruco_hl2/pose', PoseStamped, aruco_hl2_pose_callback)
    
    rospy.Subscriber('/Player0/camera/pose', PoseStamped, hl2_pose_callback)
    rospy.Subscriber('/tello/command_pos_world', PoseStamped, goal_pose_callback)

    tf_pub = rospy.Publisher('/tf', TFMessage, queue_size=10)

    # Create a timer to call the publish_transforms method periodically (e.g., every 0.1 seconds)
    # rospy.Timer(rospy.Duration(1.0), publish_transforms)

    rospy.spin()
