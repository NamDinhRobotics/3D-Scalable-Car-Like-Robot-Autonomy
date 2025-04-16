#!/usr/bin/python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import copy
import threading
import numpy as np
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry

# Global variables with lock for thread safety
lock = threading.Lock()
cur_odom_to_baselink = None
cur_map_to_odom = None

# ANSI color codes
GREEN = '\033[92m'
RED = '\033[91m'
RESET = '\033[0m'

def pose_to_mat(odom_msg):
    """Convert an Odometry message's PoseWithCovariance to a 4x4 transformation matrix."""
    pose = odom_msg.pose  # Access the PoseWithCovariance
    trans = tf.transformations.translation_matrix([pose.pose.position.x,
                                                  pose.pose.position.y,
                                                  pose.pose.position.z])
    rot = tf.transformations.quaternion_matrix([pose.pose.orientation.x,
                                               pose.pose.orientation.y,
                                               pose.pose.orientation.z,
                                               pose.pose.orientation.w])
    return np.matmul(trans, rot)

def transform_fusion():
    """Fuse odometry data and publish localization."""
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(FREQ_PUB_LOCALIZATION)
    
    while not rospy.is_shutdown():
        with lock:
            cur_odom = copy.copy(cur_odom_to_baselink)
            cur_map = copy.copy(cur_map_to_odom)
        
        if cur_odom is None or cur_map is None:
            rospy.logwarn("Waiting for odometry data...")
            rate.sleep()
            continue

        T_map_to_odom = pose_to_mat(cur_map)
        br.sendTransform(tf.transformations.translation_from_matrix(T_map_to_odom),
                         tf.transformations.quaternion_from_matrix(T_map_to_odom),
                         rospy.Time.now(),
                         child_frame, map_frame)

        localization = Odometry()
        T_odom_to_base_link = pose_to_mat(cur_odom)
        T_map_to_base_link = np.matmul(T_map_to_odom, T_odom_to_base_link)
        xyz = tf.transformations.translation_from_matrix(T_map_to_base_link)
        quat = tf.transformations.quaternion_from_matrix(T_map_to_base_link)
        localization.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
        localization.twist = cur_odom.twist

        localization.header.stamp = cur_odom.header.stamp
        localization.header.frame_id = map_frame
        localization.child_frame_id = base_frame
        pub_localization.publish(localization)
        
        rate.sleep()

def odom_callback(odom_msg):
    """Callback for odometry messages with green-colored logging."""
    global cur_odom_to_baselink
    # Log message details in green
    rospy.loginfo(f"{GREEN}Received Odometry message:{RESET}")
    rospy.loginfo(f"{GREEN}  Timestamp: {odom_msg.header.stamp}{RESET}")
    rospy.loginfo(f"{GREEN}  Frame ID: {odom_msg.header.frame_id}{RESET}")
    rospy.loginfo(f"{GREEN}  Child Frame ID: {odom_msg.child_frame_id}{RESET}")
    rospy.loginfo(f"{GREEN}  Pose Position: x={odom_msg.pose.pose.position.x:.3f}, "
                  f"y={odom_msg.pose.pose.position.y:.3f}, z={odom_msg.pose.pose.position.z:.3f}{RESET}")
    rospy.loginfo(f"{GREEN}  Pose Orientation: x={odom_msg.pose.pose.orientation.x:.3f}, "
                  f"y={odom_msg.pose.pose.orientation.y:.3f}, z={odom_msg.pose.pose.orientation.z:.3f}, "
                  f"w={odom_msg.pose.pose.orientation.w:.3f}{RESET}")
    with lock:
        cur_odom_to_baselink = odom_msg

def map_to_odom_callback(map_to_odom_msg):
    """Callback for map-to-odom messages with red-colored logging and /localization_lr publishing."""
    global cur_map_to_odom
    # Log message details in red
    rospy.loginfo(f"{RED}Received map_to_odom message:{RESET}")
    rospy.loginfo(f"{RED}  Timestamp: {map_to_odom_msg.header.stamp}{RESET}")
    rospy.loginfo(f"{RED}  Frame ID: {map_to_odom_msg.header.frame_id}{RESET}")
    rospy.loginfo(f"{RED}  Child Frame ID: {map_to_odom_msg.child_frame_id}{RESET}")
    rospy.loginfo(f"{RED}  Pose Position: x={map_to_odom_msg.pose.pose.position.x:.3f}, "
                  f"y={map_to_odom_msg.pose.pose.position.y:.3f}, z={map_to_odom_msg.pose.pose.position.z:.3f}{RESET}")
    rospy.loginfo(f"{RED}  Pose Orientation: x={map_to_odom_msg.pose.pose.orientation.x:.3f}, "
                  f"y={map_to_odom_msg.pose.pose.orientation.y:.3f}, z={map_to_odom_msg.pose.pose.orientation.z:.3f}, "
                  f"w={map_to_odom_msg.pose.pose.orientation.w:.3f}{RESET}")
    
    with lock:
        cur_map_to_odom = map_to_odom_msg
        cur_odom = copy.copy(cur_odom_to_baselink)
    
    # Publish to /localization_lr if odom data is available
    if cur_odom is not None:
        localization_lr = Odometry()
        T_map_to_odom = pose_to_mat(map_to_odom_msg)
        T_odom_to_base_link = pose_to_mat(cur_odom)
        T_map_to_base_link = np.matmul(T_map_to_odom, T_odom_to_base_link)
        xyz = tf.transformations.translation_from_matrix(T_map_to_base_link)
        quat = tf.transformations.quaternion_from_matrix(T_map_to_base_link)
        localization_lr.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
        localization_lr.twist = cur_odom.twist

        localization_lr.header.stamp = map_to_odom_msg.header.stamp  # Use map_to_odom timestamp
        localization_lr.header.frame_id = map_frame
        localization_lr.child_frame_id = base_frame
        pub_localization_lr.publish(localization_lr)
        rospy.loginfo(f"{RED}Published to /localization_lr with stamp: {map_to_odom_msg.header.stamp}{RESET}")

if __name__ == '__main__':
    # Configuration parameters
    FREQ_PUB_LOCALIZATION = rospy.get_param('~publish_frequency', 50)
    map_frame = rospy.get_param('~map_frame', 'map')
    child_frame = rospy.get_param('~child_frame', 'camera_init')
    base_frame = rospy.get_param('~base_frame', 'body')

    # Initialize node
    rospy.init_node('transform_fusion')
    rospy.loginfo('Transform Fusion Node Inited...')

    # Set up subscribers
    rospy.Subscriber('/Odometry', Odometry, odom_callback, queue_size=1)
    rospy.Subscriber('/map_to_odom', Odometry, map_to_odom_callback, queue_size=1)

    # Set up publishers
    pub_localization = rospy.Publisher('/localization', Odometry, queue_size=1)
    pub_localization_lr = rospy.Publisher('/localization_lr', Odometry, queue_size=1)

    # Start fusion thread
    threading.Thread(target=transform_fusion, daemon=True).start()

    # Keep node running
    rospy.spin()
