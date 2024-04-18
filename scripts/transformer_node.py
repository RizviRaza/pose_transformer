#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseStamped

def drone_callback(pose_stamped):
    # Invert the pose
    br = tf.TransformBroadcaster()
    inv_translation = (-pose_stamped.pose.position.x, -pose_stamped.pose.position.y, -pose_stamped.pose.position.z)
    inv_rotation = tf.transformations.quaternion_inverse([pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w])
    # Broadcast the new transform
    br.sendTransform(inv_translation,
                     inv_rotation,
                     rospy.Time.now(),
                     "aruco_marker",  # New parent frame: what was the child frame
                     "drone_marker_pose_frame")  # New child frame: what was the parent frame


    # Publish the inverted pose as a PoseStamped message
    inv_pose_stamped = PoseStamped()
    inv_pose_stamped.header.stamp = rospy.Time.now()
    inv_pose_stamped.header.frame_id = "aruco_marker"
    inv_pose_stamped.pose.position.x, inv_pose_stamped.pose.position.y, inv_pose_stamped.pose.position.z = inv_translation
    inv_pose_stamped.pose.orientation.x = inv_rotation[0]
    inv_pose_stamped.pose.orientation.y = inv_rotation[1]
    inv_pose_stamped.pose.orientation.z = inv_rotation[2]
    inv_pose_stamped.pose.orientation.w = inv_rotation[3]

    drone_pos_pub.publish(inv_pose_stamped)


def hl2_callback(pose_stamped):
    # Invert the pose
    br = tf.TransformBroadcaster()
    inv_translation = (-pose_stamped.pose.position.x, -pose_stamped.pose.position.y, -pose_stamped.pose.position.z)
    inv_rotation = tf.transformations.quaternion_inverse([pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w])
    # Broadcast the new transform
    br.sendTransform(inv_translation,
                     inv_rotation,
                     rospy.Time.now(),
                     "aruco_marker",  # New parent frame: what was the child frame
                     "hl2_marker_pose_frame")  # New child frame: what was the parent frame


    # Publish the inverted pose as a PoseStamped message
    inv_pose_stamped = PoseStamped()
    inv_pose_stamped.header.stamp = rospy.Time.now()
    inv_pose_stamped.header.frame_id = "aruco_marker"
    inv_pose_stamped.pose.position.x, inv_pose_stamped.pose.position.y, inv_pose_stamped.pose.position.z = inv_translation
    inv_pose_stamped.pose.orientation.x = inv_rotation[0]
    inv_pose_stamped.pose.orientation.y = inv_rotation[1]
    inv_pose_stamped.pose.orientation.z = inv_rotation[2]
    inv_pose_stamped.pose.orientation.w = inv_rotation[3]

    hl2_pos_pub.publish(inv_pose_stamped)

if __name__ == '__main__':
    rospy.init_node('camera_pose_publisher')
    # Create a publisher for PoseStamped
    drone_pos_pub = rospy.Publisher('/drone/camera_pose_in_marker_frame', PoseStamped, queue_size=10)
    rospy.Subscriber("/aruco_drone/pose", PoseStamped, drone_callback)

    hl2_pos_pub = rospy.Publisher('/hl2/camera_pose_in_marker_frame', PoseStamped, queue_size=10)
    rospy.Subscriber("/aruco_hl2/pose", PoseStamped, hl2_callback)

    
    

    rospy.spin()