#!/usr/bin/env python3
import rospy
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer
from math import sqrt
import sys
import select

# Global variables to store the latest poses and saved poses
global_drone_pose_latest = None
global_aruco_pose_latest = None
pose_1_drone = None
pose_1_aruco = None
pose_2_drone = None
pose_2_aruco = None

def calculate_distance(pose1, pose2):
    """
    Calculate Euclidean distance between two PoseStamped positions.
    """
    if pose1 is None or pose2 is None:
        return None

    x1, y1, z1 = pose1.pose.position.x, pose1.pose.position.y, pose1.pose.position.z
    x2, y2, z2 = pose2.pose.position.x, pose2.pose.position.y, pose2.pose.position.z

    return sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

def save_pose(pose, label):
    rospy.loginfo(f"{label} saved: {pose}")
    
def handle_key_presses():
    global pose_1_drone, pose_1_aruco, pose_2_drone, pose_2_aruco

    rospy.loginfo("Press 'a' to save pose_1, 's' to save pose_2.")

    while not rospy.is_shutdown():
        if select.select([sys.stdin], [], [], 0.1)[0]:  # Non-blocking input
            key = sys.stdin.read(1)
            if key == 'a':
                # Save current poses as pose_1
                pose_1_drone = global_drone_pose_latest
                pose_1_aruco = global_aruco_pose_latest
                save_pose(pose_1_drone, "Drone Pose 1")
                save_pose(pose_1_aruco, "Aruco Pose 1")

            elif key == 's':
                # Save current poses as pose_2
                pose_2_drone = global_drone_pose_latest
                pose_2_aruco = global_aruco_pose_latest

            if pose_1_drone and pose_2_drone and pose_1_aruco and pose_2_aruco:
                # Calculate distances once both pose_1 and pose_2 are populated
                drone_distance = calculate_distance(pose_1_drone, pose_2_drone)
                aruco_distance = calculate_distance(pose_1_aruco, pose_2_aruco)

                rospy.loginfo(f"Distance between Drone Pose 1 and Pose 2: {drone_distance}")
                rospy.loginfo(f"Distance between Aruco Pose 1 and Pose 2: {aruco_distance}")

                pose_1_drone = None
                pose_1_aruco = None
                pose_2_drone = None
                pose_2_aruco = None

def pose_callback(drone_pose, aruco_pose):
    global global_drone_pose_latest, global_aruco_pose_latest

    # Update the latest poses
    global_drone_pose_latest = drone_pose
    global_aruco_pose_latest = aruco_pose

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
    drone_pose_sub = Subscriber('/mavros/local_position/pose', PoseStamped)
    aruco_pose_sub = Subscriber('/aruco_drone/pose', PoseStamped)
    rospy.loginfo("Subscribers initialized.")

    # Synchronize the messages
    ats = ApproximateTimeSynchronizer([drone_pose_sub, aruco_pose_sub],
                                       queue_size=10,
                                       slop=1.0)
    ats.registerCallback(pose_callback)
    rospy.loginfo("Pose synchronizer initialized.")

    rospy.loginfo("Pose sync node running.")

    # Start key press handling in a separate thread
    from threading import Thread
    key_thread = Thread(target=handle_key_presses)
    key_thread.daemon = True
    key_thread.start()

    rospy.spin()

if __name__ == '__main__':
    try:
        # Set stdin to non-blocking
        import termios
        import tty
        tty.setcbreak(sys.stdin.fileno())
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down pose sync node.")
    finally:
        # Reset stdin to normal
        import termios
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
