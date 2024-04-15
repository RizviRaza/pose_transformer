#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_inverse

def pose_callback(pose_msg):
    global tf_buffer, pose_pub

    newPose = pose_msg

    try:
        # Transform the pose from world frame to vicon/world frame
        transformed_pose = tf_buffer.transform(newPose, 'vicon/world', rospy.Duration(1.0))

        new_pose_msg = PoseStamped()

        # Copy the header from the transformed_pose
        new_pose_msg.header = transformed_pose.header

        # Set the frame ID to 'vicon/world'
        new_pose_msg.header.frame_id = 'vicon/world'

        # Copy the pose from the transformed_pose
        new_pose_msg.pose = transformed_pose.pose

        # Publish the new PoseStamped message
        pose_pub.publish(new_pose_msg)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr("Transformation error: %s", e)


def odom_callback(odom_msg):
    """
    Callback for odometry messages. This function should transform the odometry
    data from its original frame to the /vicon/world frame and then publish the
    transformed data.
    """
    global tf_pub, tf_buffer

    transform = TransformStamped()

    # Populate the transform message
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = 'vicon/world'  # Parent frame ID
    transform.child_frame_id = 'world'  # Child frame ID

    # Get the pose from odometry message
    pose = odom_msg.pose.pose
    translation = pose.position
    orientation = pose.orientation

    # Fill translation
    transform.transform.translation.x = translation.x
    transform.transform.translation.y = translation.y
    transform.transform.translation.z = translation.z

    # Fill rotation
    (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    quaternion = quaternion_from_euler(roll, pitch, yaw)

    # Invert the rotation
    inv_quaternion = quaternion_inverse(quaternion)
    transform.transform.rotation.x = inv_quaternion[0]
    transform.transform.rotation.y = inv_quaternion[1]
    transform.transform.rotation.z = inv_quaternion[2]
    transform.transform.rotation.w = inv_quaternion[3]

    # Publish the transform
    tf_pub.sendTransform(transform)

def main():
    rospy.init_node('pose_transformer_node')
    global tf_pub, tf_buffer, pose_pub

    tf_pub = tf2_ros.TransformBroadcaster()

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Initialize the publisher for transformed poses
    pose_pub = rospy.Publisher('transformed_pose', PoseStamped, queue_size=10)

    # Subscribing the PLayer odom in 'world' frame
    rospy.Subscriber('/Player0/head/odom', Odometry, odom_callback)

    # # subscribing the tello command pose in 'world' frame
    rospy.Subscriber('/tello/command_pos_world', PoseStamped, pose_callback)


    rate = rospy.Rate(5)  # Adjust the rate as needed

    while not rospy.is_shutdown():
        try:
            # Lookup the transformation from world to vicon/world
            trans = tf_buffer.lookup_transform('vicon/world', 'world', rospy.Time())

            rospy.loginfo("Transformation of the world frame with respect to vicon/world: %s", trans)
            # Process the transform as needed
            # ...

            rate.sleep()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

if __name__ == '__main__':
    main()
