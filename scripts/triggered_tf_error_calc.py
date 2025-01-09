#!/usr/bin/env python

import rospy
import tf
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import String
import threading
import time
import os

# Function to calculate errors
def calculate_error(transform_1, transform_2):
    # Calculate Euclidean distance
    pos1 = np.array(transform_1[0])
    pos2 = np.array(transform_2[0])
    euclidean_distance = np.linalg.norm(pos1 - pos2)

    # Calculate positional error components
    x_error = abs(pos1[0] - pos2[0])
    y_error = abs(pos1[1] - pos2[1])
    z_error = abs(pos1[2] - pos2[2])

    # Calculate rotational error around the y-axis (pitch)
    rot1 = transform_1[1]
    rot2 = transform_2[1]

    # Convert quaternions to Euler angles
    euler1 = euler_from_quaternion(rot1)  # Roll, Pitch, Yaw
    euler2 = euler_from_quaternion(rot2)

    # Extract pitch (y-axis rotation) and calculate difference in degrees
    pitch1_deg = np.degrees(euler1[1])  # Pitch from transform_1
    pitch2_deg = np.degrees(euler2[1])  # Pitch from transform_2
    pitch_error_deg = abs(pitch1_deg - pitch2_deg)

    return euclidean_distance, pitch_error_deg, x_error, y_error, z_error

# Function to perform averaging and write to file
def calculate_averaged_error(listener, parent_frame, child_frame_1, child_frame_2, averaging_duration, file_path):
    start_time = time.time()
    errors = []

    while time.time() - start_time < averaging_duration:
        try:
            # Lookup transforms for the two child frames
            transform_1 = listener.lookupTransform(parent_frame, child_frame_1, rospy.Time(0))
            transform_2 = listener.lookupTransform(parent_frame, child_frame_2, rospy.Time(0))

            # Calculate the error
            distance, rotation, x_err, y_err, z_err = calculate_error(transform_1, transform_2)
            errors.append((distance, rotation, x_err, y_err, z_err))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("TF Exception: {}".format(e))

        time.sleep(0.1)  # Sampling at 10 Hz

    if errors:
        # Calculate averages
        avg_distance = np.mean([e[0] for e in errors])
        avg_rotation = np.mean([e[1] for e in errors])
        avg_x_error = np.mean([e[2] for e in errors])
        avg_y_error = np.mean([e[3] for e in errors])
        avg_z_error = np.mean([e[4] for e in errors])

        # Log results
        rospy.loginfo("Averaged Euclidean Distance: {:.4f}, Averaged Rotational Error: {:.4f}".format(avg_distance, avg_rotation))
        rospy.loginfo("Averaged Positional Errors - X: {:.4f}, Y: {:.4f}, Z: {:.4f}".format(avg_x_error, avg_y_error, avg_z_error))

        # Write to file
        with open(file_path, 'a') as file:
            file.write("Averaged Euclidean Distance: {:.4f}, Averaged Rotational Error: {:.4f}\n".format(avg_distance, avg_rotation))
            file.write("Averaged Positional Errors - X: {:.4f}, Y: {:.4f}, Z: {:.4f}\n".format(avg_x_error, avg_y_error, avg_z_error))
    else:
        rospy.logwarn("No valid data collected during the averaging period.")

# Thread function to listen for 's' input and trigger averaging
def key_listener(listener, parent_frame, child_frame_1, child_frame_2, averaging_duration, file_path):
    while not rospy.is_shutdown():
        key = input("Press 's' to start averaging (or 'q' to quit): ")
        if key.strip().lower() == "q":
            rospy.signal_shutdown("User requested shutdown.")
            break
        elif key.strip().lower() == "s":
            rospy.loginfo("Starting error averaging for {} seconds...".format(averaging_duration))
            calculate_averaged_error(listener, parent_frame, child_frame_1, child_frame_2, averaging_duration, file_path)
        else:
            rospy.loginfo("Invalid input. Press 's' to start averaging or 'q' to quit.")

if __name__ == "__main__":
    rospy.init_node("tf_error_calculator", anonymous=True)

    # Parameters
    parent_frame = rospy.get_param("~parent_frame", "aruco")
    child_frame_1 = rospy.get_param("~child_frame_1", "goal")
    child_frame_2 = rospy.get_param("~child_frame_2", "drone")
    averaging_duration = rospy.get_param("~averaging_duration", 5.0)  # Averaging duration in seconds

    # File path for error logging
    file_path = rospy.get_param("~error_log_file", "error_log.txt")

    # Ensure the file exists
    if not os.path.exists(file_path):
        with open(file_path, 'w') as file:
            file.write("Error Log\n")
            file.write("===========\n")

    # TF listener
    listener = tf.TransformListener()

    # Start key listener in a separate thread
    key_listener_thread = threading.Thread(target=key_listener, args=(listener, parent_frame, child_frame_1, child_frame_2, averaging_duration, file_path))
    key_listener_thread.daemon = True
    key_listener_thread.start()

    # Keep the node running
    rospy.spin()