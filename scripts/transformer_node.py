#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

def handle_vicon_transform(msg):
    # Placeholder for handling Vicon messages
    pass

def handle_odom_msg(msg):
    # Placeholder for handling odometry messages
    pass

if __name__ == '__main__':
    rospy.init_node('pose_transformer_node')

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    rospy.Subscriber('/vicon/mini_drone/mini_drone', TransformStamped, handle_vicon_transform)
    rospy.Subscriber('/Player0/head/odom', Odometry, handle_odom_msg)

    rospy.spin()
