#!/usr/bin/env python

import rospy
import tf
import tf.transformations as tf_trans
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped


# Node to subscribe to the aruco_marker_frame's pose wrt to the base_link of the drone
# And invert it to publish the drone's base_link pose wrt to the aruco marker

def camera_info_callback(camera_info_msg):
    # Update the D matrix with the new values
    camera_info_msg.D = [0.0355633167278393, -0.123584193169489, -0.000250953321849423, -0.000181034148553509, 0.155155222181198]
    # camera_info_msg.R = [0.0355633167278393, -0.123584193169489, -0.000250953321849423, -0.000181034148553509, 0.155155222181198, -0.000181034148553509, 0.155155222181198, -0.000181034148553509, 0.155155222181198]
    camera_info_msg.P = [2992.533447265625, 0.0, 1917.8743896484375, 0.0, 0.0, 2994.665771484375, 1038.7142333984375, 0.0, 0.0, 0.0, 1.0, 0.0]

    # Publish the modified CameraInfo message to a new topic
    hl2_camera_info_pub.publish(camera_info_msg)


def pose_callback(msg):
    # Get the pose from the message
    marker_pose = msg.pose

    # Extract the position and orientation
    marker_position = (marker_pose.position.x, marker_pose.position.y, marker_pose.position.z)
    marker_orientation = (marker_pose.orientation.x, marker_pose.orientation.y, marker_pose.orientation.z, marker_pose.orientation.w)

    # Convert to transformation matrix
    marker_transform = tf_trans.concatenate_matrices(
        tf_trans.translation_matrix(marker_position),
        tf_trans.quaternion_matrix(marker_orientation)
    )

    # Invert the transformation matrix
    inverted_transform = tf_trans.inverse_matrix(marker_transform)

    # Extract the inverted translation and rotation
    inverted_translation = tf_trans.translation_from_matrix(inverted_transform)
    inverted_rotation = tf_trans.quaternion_from_matrix(inverted_transform)

    # Publish the inverted transformation as a PoseStamped message
    inverted_pose = PoseStamped()
    inverted_pose.header.stamp = rospy.Time.now()
    inverted_pose.header.frame_id = "aruco_marker_frame"
    inverted_pose.pose.position.x = inverted_translation[0]
    inverted_pose.pose.position.y = inverted_translation[1]
    inverted_pose.pose.position.z = inverted_translation[2]
    inverted_pose.pose.orientation.x = inverted_rotation[0]
    inverted_pose.pose.orientation.y = inverted_rotation[1]
    inverted_pose.pose.orientation.z = inverted_rotation[2]
    inverted_pose.pose.orientation.w = inverted_rotation[3]

    inverted_pose_pub.publish(inverted_pose)

if __name__ == '__main__':
    rospy.init_node('drone_pose_with_respect_to_marker')
    inverted_pose_pub = rospy.Publisher('/inverted_pose', PoseStamped, queue_size=10)

    hl2_camera_info_pub = rospy.Publisher('/Player0/camera/camera_info/updated', CameraInfo, queue_size=10)
    
    rospy.Subscriber('/Player0/camera/camera_info', CameraInfo, camera_info_callback) 
    rospy.Subscriber('/aruco_drone/pose', PoseStamped, pose_callback)
    rospy.spin()
