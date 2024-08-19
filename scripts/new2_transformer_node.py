#!/usr/bin/env python

import rospy
import tf
import tf.transformations as tf_trans
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage

# Variables to store transforms internally
aruco_drone_transform = None
aruco_hl2_transform = None
hl2_pose_transform = None
goal_pose_transform = None

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
    print("aruco-base_link transform recorded")

# Subscribes to the /Player0/camera/pose topic
# Saves the map -> hl2 transform
def hl2_pose_callback(msg):
    global hl2_pose_transform

    hl2_pose = msg.pose
    hl2_position = (hl2_pose.position.x, hl2_pose.position.y, hl2_pose.position.z)
    hl2_orientation = (hl2_pose.orientation.x, hl2_pose.orientation.y, hl2_pose.orientation.z, hl2_pose.orientation.w)

    hl2_pose_transform = tf_trans.concatenate_matrices(
        tf_trans.translation_matrix(hl2_position),
        tf_trans.quaternion_matrix(hl2_orientation)
    )
    print("map-hl2 transform recorded")

# Subscribes to the /tello/command_pos_world topic
# Saves the map -> goal transform
def goal_pose_callback(msg):
    global goal_pose_transform

    goal_pose = msg.pose
    goal_position = (goal_pose.position.x, goal_pose.position.y, goal_pose.position.z)
    goal_orientation = (goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w)

    goal_pose_transform = tf_trans.concatenate_matrices(
        tf_trans.translation_matrix(goal_position),
        tf_trans.quaternion_matrix(goal_orientation)
    )

    print("map-goal transform recorded")

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

    print("hl2-aruco transform recorded")

def publish_transforms(event):
    print("Checking transforms")

    # Ensure all transforms are initialized (i.e., not None)
    if hl2_pose_transform is not None and aruco_drone_transform is not None and goal_pose_transform is not None:

        rospy.loginfo("Publishing transforms")

        # Calculate map -> base_link
        map_to_base_link_transform = tf_trans.concatenate_matrices(hl2_pose_transform, aruco_drone_transform)

        # Extract the translation and rotation from map -> base_link
        map_to_base_link_translation = tf_trans.translation_from_matrix(map_to_base_link_transform)
        map_to_base_link_rotation = tf_trans.quaternion_from_matrix(map_to_base_link_transform)

        # Publish the map -> base_link transform
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = "map"
        transform_stamped.child_frame_id = "base_link"
        transform_stamped.transform.translation.x = map_to_base_link_translation[0]
        transform_stamped.transform.translation.y = map_to_base_link_translation[1]
        transform_stamped.transform.translation.z = map_to_base_link_translation[2]
        transform_stamped.transform.rotation.x = map_to_base_link_rotation[0]
        transform_stamped.transform.rotation.y = map_to_base_link_rotation[1]
        transform_stamped.transform.rotation.z = map_to_base_link_rotation[2]
        transform_stamped.transform.rotation.w = map_to_base_link_rotation[3]

        tf_msg = TFMessage([transform_stamped])
        tf_pub.publish(tf_msg)

        print("map-base_link transform published")

        # Publish the map -> goal transform
        goal_translation = tf_trans.translation_from_matrix(goal_pose_transform)
        goal_rotation = tf_trans.quaternion_from_matrix(goal_pose_transform)

        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = "map"
        transform_stamped.child_frame_id = "goal"
        transform_stamped.transform.translation.x = goal_translation[0]
        transform_stamped.transform.translation.y = goal_translation[1]
        transform_stamped.transform.translation.z = goal_translation[2]
        transform_stamped.transform.rotation.x = goal_rotation[0]
        transform_stamped.transform.rotation.y = goal_rotation[1]
        transform_stamped.transform.rotation.z = goal_rotation[2]
        transform_stamped.transform.rotation.w = goal_rotation[3]

        tf_msg = TFMessage([transform_stamped])
        tf_pub.publish(tf_msg)

    else:
        rospy.logwarn("Transforms are not yet initialized")

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
    rospy.Timer(rospy.Duration(1.0), publish_transforms)

    rospy.spin()
