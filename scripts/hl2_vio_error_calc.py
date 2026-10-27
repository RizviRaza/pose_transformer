#!/usr/bin/env python3
import rospy
import sys

from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer
from math import sqrt
from datetime import datetime

# Global variables to store the latest and previous poses
global_player_pose_latest = None
global_player_pose_prev = None
global_aruco_pose_latest = None
global_aruco_pose_prev = None

# Open log file
log_file = None

def initialize_log_file():
    global log_file
    current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    file_name = f"hl2_vio_error_{current_time}.txt"
    log_file = open(file_name, 'w')
    # log_file.write("New Measurement\n")


def close_log_file():
    if log_file:
        log_file.close()

def log_to_file(message):
    if log_file:
        log_file.write(message + "\n")

def calculate_distance(pose1, pose2):
    """
    Calculate Euclidean distance between two PoseStamped positions.
    """
    if pose1 is None or pose2 is None:
        return None

    x1, y1, z1 = pose1.pose.position.x, pose1.pose.position.y, pose1.pose.position.z
    x2, y2, z2 = pose2.pose.position.x, pose2.pose.position.y, pose2.pose.position.z

    return sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

def pose_callback(player_pose, aruco_pose):
    global global_player_pose_latest, global_player_pose_prev
    global global_aruco_pose_latest, global_aruco_pose_prev

    # Update the previous poses
    global_player_pose_prev = global_player_pose_latest
    global_aruco_pose_prev = global_aruco_pose_latest

    # Update the latest poses
    global_player_pose_latest = player_pose
    global_aruco_pose_latest = aruco_pose

    # Calculate distances
    player_distance = calculate_distance(global_player_pose_prev, global_player_pose_latest)
    aruco_distance = calculate_distance(global_aruco_pose_prev, global_aruco_pose_latest)

    # Log the distances and poses
    if player_distance is not None and aruco_distance is not None:
        player_poses_log = f"Player previous pose: {global_player_pose_prev}, Player latest pose: {global_player_pose_latest}"
        aruco_poses_log = f"Aruco previous pose: {global_aruco_pose_prev}, Aruco latest pose: {global_aruco_pose_latest}"
        player_distance_log = f"Player pose distance: {player_distance}"
        aruco_distance_log = f"Aruco pose distance: {aruco_distance}"

        # rospy.loginfo(player_poses_log)
        # rospy.loginfo(aruco_poses_log)
        rospy.loginfo(player_distance_log)
        rospy.loginfo(aruco_distance_log)

        log_to_file("---New Measurement---")
        # log_to_file(player_poses_log)
        # log_to_file(aruco_poses_log)

        log_to_file(f"Player previous pose: {global_player_pose_prev}")
        log_to_file(f"Player latest pose: {global_player_pose_latest}")
        log_to_file(f"Aruco previous pose: {global_aruco_pose_prev}")
        log_to_file(f"Aruco latest pose: {global_aruco_pose_latest}")

        log_to_file(player_distance_log)
        log_to_file(aruco_distance_log)

        # global_player_pose_latest = None
        # global_player_pose_prev = None
        # global_aruco_pose_latest = None
        # global_aruco_pose_prev = None

def main():
    rospy.init_node('hl2_vio_error_node', anonymous=True)
    rospy.loginfo("Initializing node...")

    # Initialize log file
    initialize_log_file()

    # Wait for /clock if using sim_time
    if rospy.get_param('/use_sim_time', True):
        rospy.loginfo("Waiting for /clock topic...")
        try:
            rospy.wait_for_message("/clock", Clock, timeout=10.0)
            rospy.loginfo("/clock topic is now available.")
        except rospy.ROSException:
            rospy.logerr("Timed out waiting for /clock. Exiting node.")
            return

    # Set up subscribers
    player_pose_sub = Subscriber('/Player0/camera/pose', PoseStamped)
    aruco_pose_sub = Subscriber('/aruco_hl2/pose', PoseStamped)
    rospy.loginfo("Subscribers initialized.")

    # Synchronize the messages
    ats = ApproximateTimeSynchronizer([player_pose_sub, aruco_pose_sub],
                                       queue_size=1,
                                       slop=1.0)
    ats.registerCallback(pose_callback)
    rospy.loginfo("Pose synchronizer initialized.")

    rospy.loginfo("Pose sync node running.")
    rospy.spin()

if __name__ == '__main__':
    original_settings = None
    try:
        # Save the original terminal settings
        import termios
        original_settings = termios.tcgetattr(sys.stdin)

        # Set stdin to non-blocking
        import tty
        tty.setcbreak(sys.stdin.fileno())
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down hl2 vio error node.")
    finally:
        # Restore the terminal settings
        if original_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)
        # Close the log file
        close_log_file()

