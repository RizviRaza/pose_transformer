#!/usr/bin/env python3
import rospy
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer

# Global variables to store poses
global_player_pose = None
global_aruco_pose = None

def pose_callback(player_pose, aruco_pose):
    global global_player_pose, global_aruco_pose
    global_player_pose = player_pose
    global_aruco_pose = aruco_pose
    rospy.loginfo("Synchronized poses received:")
    rospy.loginfo("Player Pose: %s", player_pose)
    rospy.loginfo("Aruco Pose: %s", aruco_pose)

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
