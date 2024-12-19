#!/usr/bin/env python

import rospy
import tf
from tf.transformations import euler_from_quaternion
import numpy as np

def calculate_error(transform_1, transform_2):
    # Calculate Euclidean distance
    pos1 = np.array(transform_1[0])
    pos2 = np.array(transform_2[0])
    euclidean_distance = np.linalg.norm(pos1 - pos2)

    # Calculate rotational error
    rot1 = transform_1[1]
    rot2 = transform_2[1]

    # Convert quaternions to Euler angles
    euler1 = euler_from_quaternion(rot1)
    euler2 = euler_from_quaternion(rot2)

    # Calculate the angular difference
    rotational_error = np.linalg.norm(np.array(euler1) - np.array(euler2))

    return euclidean_distance, rotational_error

def tf_callback():
    try:
        # Lookup transforms for the two child frames
        transform_1 = listener.lookupTransform(parent_frame, child_frame_1, rospy.Time(0))
        transform_2 = listener.lookupTransform(parent_frame, child_frame_2, rospy.Time(0))

        # Calculate the error
        distance, rotation = calculate_error(transform_1, transform_2)

        rospy.loginfo("Euclidean Distance: {:.4f}, Rotational Error: {:.4f}".format(distance, rotation))

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logwarn("TF Exception: {}".format(e))

if __name__ == "__main__":
    rospy.init_node("tf_error_calculator", anonymous=True)

    # Parameters
    parent_frame = rospy.get_param("~parent_frame", "aruco")
    child_frame_1 = rospy.get_param("~child_frame_1", "goal")
    child_frame_2 = rospy.get_param("~child_frame_2", "base_link")

    # TF listener
    listener = tf.TransformListener()

    # Set a rate for the loop
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        tf_callback()
        rate.sleep()
