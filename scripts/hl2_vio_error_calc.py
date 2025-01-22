#!/usr/bin/env python3
import rospy
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer
from math import sqrt

# Global variables to store the latest and previous poses
global_player_pose_latest = None
global_player_pose_prev = None
global_aruco_pose_latest = None
global_aruco_pose_prev = None

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

    # if player_distance is not None and aruco_distance is not None:
    #     global_player_pose_latest = None
    #     global_player_pose_prev = None
    #     global_aruco_pose_latest = None
    #     global_aruco_pose_prev = None

    rospy.loginfo("Player pose distance: %s", player_distance)
    rospy.loginfo("Aruco pose distance: %s", aruco_distance)

def main():
    rospy.init_node('pose_sync_node', anonymous=True)
    rospy.loginfo("Initializing node...")

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
                                       queue_size=10,
                                       slop=1.0)
    ats.registerCallback(pose_callback)
    rospy.loginfo("Pose synchronizer initialized.")

    rospy.loginfo("Pose sync node running.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down pose sync node.")
