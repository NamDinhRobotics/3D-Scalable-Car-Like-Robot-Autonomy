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
import message_filters

# Global variables with lock for thread safety
lock = threading.Lock()
cur_odom_to_baselink = None
cur_map_to_odom = None

def pose_to_mat(pose_msg):
    """Convert a Pose message to a 4x4 transformation matrix."""
    trans = tf.transformations.translation_matrix([pose_msg.pose.pose.position.x,
                                                  pose_msg.pose.pose.position.y,
                                                  pose_msg.pose.pose.position.z])
    rot = tf.transformations.quaternion_matrix([pose_msg.pose.pose.orientation.x,
                                               pose_msg.pose.pose.orientation.y,
                                               pose_msg.pose.pose.orientation.z,
                                               pose_msg.pose.pose.orientation.w])
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

def callback(odom_msg, map_to_odom_msg):
    """Callback for synchronized odometry messages."""
    global cur_odom_to_baselink, cur_map_to_odom
    with lock:
        cur_odom_to_baselink = odom_msg
        cur_map_to_odom = map_to_odom_msg

if __name__ == '__main__':
    # Configuration parameters
    FREQ_PUB_LOCALIZATION = rospy.get_param('~publish_frequency', 50)
    map_frame = rospy.get_param('~map_frame', 'map')
    child_frame = rospy.get_param('~child_frame', 'camera_init')
    base_frame = rospy.get_param('~base_frame', 'body')

    # Initialize node
    rospy.init_node('transform_fusion')
    rospy.loginfo('Transform Fusion Node Inited...')

    # Set up subscribers with time synchronization
    odom_sub = message_filters.Subscriber('/Odometry', Odometry)
    map_to_odom_sub = message_filters.Subscriber('/map_to_odom', Odometry)
    ts = message_filters.ApproximateTimeSynchronizer([odom_sub, map_to_odom_sub], queue_size=10, slop=0.1)
    ts.registerCallback(callback)

    # Set up publisher
    pub_localization = rospy.Publisher('/localization', Odometry, queue_size=1)

    # Start fusion thread
    threading.Thread(target=transform_fusion, daemon=True).start()

    # Keep node running
    rospy.spin()
