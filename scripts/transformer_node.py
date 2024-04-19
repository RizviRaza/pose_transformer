#!/usr/bin/env python
import rospy
import tf
import message_filters
from geometry_msgs.msg import PoseStamped

global tello_translation
global tello_rotation

def drone_callback(pose_stamped):
    # Invert the pose
    # br = tf.TransformBroadcaster()
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

def combined_callback(hl2_pose, world_pose):

    # Invert the pose
    # br = tf.TransformBroadcaster()
    inv_translation = (-hl2_pose.pose.position.x, -hl2_pose.pose.position.y, -hl2_pose.pose.position.z)
    inv_rotation = tf.transformations.quaternion_inverse([hl2_pose.pose.orientation.x, hl2_pose.pose.orientation.y, hl2_pose.pose.orientation.z, hl2_pose.pose.orientation.w])
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

    try:
        # Wait for the transformation from hl2_marker_pose_frame to aruco_marker to become available
        listener.waitForTransform("aruco_marker", "hl2_marker_pose_frame", rospy.Time(0), rospy.Duration(1.0))
        (trans, rot) = listener.lookupTransform("aruco_marker", "hl2_marker_pose_frame", rospy.Time(0))

        # Calculate the transformation from world to aruco_marker
        world_to_aruco_trans = (world_pose.pose.position.x - trans[0],
                                world_pose.pose.position.y - trans[1],
                                world_pose.pose.position.z - trans[2])
        world_to_aruco_rot = tf.transformations.quaternion_multiply(
            [world_pose.pose.orientation.x,
             world_pose.pose.orientation.y,
             world_pose.pose.orientation.z,
             world_pose.pose.orientation.w],
            tf.transformations.quaternion_inverse(rot))

        # Optionally, broadcast this transformation
        br.sendTransform(world_to_aruco_trans,
                         world_to_aruco_rot,
                         rospy.Time.now(),
                         "aruco_marker",
                         "world")
        
        print("Transform published from aruco_marker to world")

        # Compute transformation from "world" to "tello_marker_pose_frame"
        # listener.waitForTransform("world", "tello_marker_pose_frame", rospy.Time.now(), rospy.Duration(1.0))
        # (trans_world_tello, rot_world_tello) = listener.lookupTransform("world", "tello_marker_pose_frame", rospy.Time(0))

        trans_world_tello = tello_translation
        rot_world_tello = tello_rotation
        (trans_aruco_world, rot_aruco_world) = listener.lookupTransform("aruco_marker", "world", rospy.Time(0))

        # Project transformation from "world" to "tello_marker_pose_frame" to the "aruco_marker" frame
        trans_aruco_tello = tf.transformations.concatenate_matrices(
            tf.transformations.translation_matrix(trans_aruco_world),
            tf.transformations.inverse_matrix(tf.transformations.translation_matrix(trans_world_tello))
        )
        trans_aruco_tello = tf.transformations.translation_from_matrix(trans_aruco_tello)
        rot_aruco_tello = tf.transformations.quaternion_multiply(rot_aruco_world, tf.transformations.quaternion_inverse(rot_world_tello))

        # Broadcast this final transformation
        br.sendTransform(trans_aruco_tello,
                         rot_aruco_tello,
                         rospy.Time.now(),
                         "aruco_marker",
                         "tello_marker_pose_frame")
        
        print("Transform published from aruco_marker to tello_marker_pose_frame")

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("TF exception: %s", e)

def tello_command_pos_callback(pose_stamped):
    global tello_translation
    global tello_rotation
    # Broadcast the received pose as a transform
    tello_translation = (pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z)
    tello_rotation = (pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w)
    br.sendTransform(tello_translation,
                     tello_rotation,
                     rospy.Time.now(),
                     "world",
                     "tello_marker_pose_frame")
    print("Broadcasted Tello's transform from the world frame.")

if __name__ == '__main__':
    rospy.init_node('camera_pose_publisher')

    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    
    rospy.Subscriber("/aruco_drone/pose", PoseStamped, drone_callback)
    rospy.Subscriber("/tello/command_pos", PoseStamped, tello_command_pos_callback)

    hl2_pose_sub = message_filters.Subscriber('/aruco_hl2/pose', PoseStamped)
    world_pose_sub = message_filters.Subscriber('/Player0/camera/pose', PoseStamped)

    ts = message_filters.ApproximateTimeSynchronizer([hl2_pose_sub, world_pose_sub], queue_size=10, slop=1, allow_headerless=True)
    ts.registerCallback(combined_callback)

    drone_pos_pub = rospy.Publisher('/drone/camera_pose_in_marker_frame', PoseStamped, queue_size=10)
    hl2_pos_pub = rospy.Publisher('/hl2/camera_pose_in_marker_frame', PoseStamped, queue_size=10)

    rospy.spin()