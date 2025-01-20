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

# Function to average the aruco -> drone transform
def average_transform(listener, parent_frame, child_frame, averaging_duration):
    start_time = time.time()
    positions = []
    orientations = []

    while time.time() - start_time < averaging_duration:
        try:
            # Lookup transform
            transform = listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))
            positions.append(np.array(transform[0]))
            orientations.append(np.array(transform[1]))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("TF Exception: {}".format(e))
        time.sleep(0.1)  # Sampling at 10 Hz

    if positions and orientations:
        # Calculate averages
        avg_position = np.mean(positions, axis=0)
        avg_orientation = np.mean(orientations, axis=0)
        return avg_position.tolist(), avg_orientation.tolist()
    else:
        rospy.logwarn("No valid data collected for averaging.")
        return None, None

# Function to calculate and log all details
def log_transform_and_errors(listener, parent_frame, goal_frame, drone_frame, averaging_duration, file_path):
    try:
        # Lookup aruco -> goal transform
        goal_transform = listener.lookupTransform(parent_frame, goal_frame, rospy.Time(0))
        goal_position, goal_orientation = goal_transform[0], goal_transform[1]

        # Average aruco -> drone transform
        drone_position, drone_orientation = average_transform(listener, parent_frame, drone_frame, averaging_duration)

        if drone_position is None or drone_orientation is None:
            rospy.logwarn("Averaging failed. Skipping this log.")
            return

        # Calculate errors
        euclidean_distance, rotational_error_deg, x_error, y_error, z_error = calculate_error(
            (goal_position, goal_orientation),
            (drone_position, drone_orientation)
        )

        # Log to file
        with open(file_path, 'a') as file:
            # Add a new section in the log
            file.write("\nNew Measurement\n")
            file.write("================\n")

            # Log aruco -> goal transform
            file.write("aruco -> goal transform:\n")
            file.write("  Position: {}\n".format(goal_position))
            file.write("  Orientation (Quaternion): {}\n".format(goal_orientation))

            # Log averaged aruco -> drone transform
            file.write("Averaged aruco -> drone transform:\n")
            file.write("  Position: {}\n".format(drone_position))
            file.write("  Orientation (Quaternion): {}\n".format(drone_orientation))

            # Log errors
            file.write("Errors:\n")
            file.write("  Positional Errors - X: {:.4f}, Y: {:.4f}, Z: {:.4f}\n".format(x_error, y_error, z_error))
            file.write("  Euclidean Distance Error: {:.4f}\n".format(euclidean_distance))
            file.write("  Rotational Error (Degrees): {:.4f}\n".format(rotational_error_deg))

        rospy.loginfo("Data successfully logged.")
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logwarn("TF Exception: {}".format(e))

# Thread function to listen for 's' input and trigger logging
def key_listener(listener, parent_frame, goal_frame, drone_frame, averaging_duration, file_path):
    while not rospy.is_shutdown():
        key = input("Press 's' to log data (or 'q' to quit): ")
        if key.strip().lower() == "q":
            rospy.signal_shutdown("User requested shutdown.")
            break
        elif key.strip().lower() == "s":
            rospy.loginfo("Logging data...")
            with open(file_path, 'a') as file:
                file.write("\n")  # Append a new line when 's' is pressed
            log_transform_and_errors(listener, parent_frame, goal_frame, drone_frame, averaging_duration, file_path)
        else:
            rospy.loginfo("Invalid input. Press 's' to log data or 'q' to quit.")

if __name__ == "__main__":
    rospy.init_node("tf_error_calculator", anonymous=True)

    # Parameters
    parent_frame = rospy.get_param("~parent_frame", "aruco")
    goal_frame = rospy.get_param("~goal_frame", "goal")
    drone_frame = rospy.get_param("~drone_frame", "drone")
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
    key_listener_thread = threading.Thread(target=key_listener, args=(listener, parent_frame, goal_frame, drone_frame, averaging_duration, file_path))
    key_listener_thread.daemon = True
    key_listener_thread.start()

    # Keep the node running
    rospy.spin()
