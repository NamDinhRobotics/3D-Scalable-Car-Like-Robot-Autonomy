#!/usr/bin/python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import copy
import _thread
import time

import open3d as o3d
import rospy
import ros_numpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import numpy as np
import tf
import tf.transformations

# Global variables to store the map, initialization state, transformation, and current sensor data
global_map = None  # Stores the global point cloud map
initialized = False  # Flag to indicate if the system is initialized
T_map_to_odom = np.eye(4)  # 4x4 transformation matrix from map to odom frame
cur_odom = None  # Stores the current odometry message
cur_scan = None  # Stores the current point cloud scan

# Converts a ROS Pose message to a 4x4 transformation matrix
def pose_to_mat(pose_msg):
    """
    Converts a ROS Pose message (position and orientation) into a 4x4 transformation matrix.
    Combines translation (from position) and rotation (from quaternion orientation).
    """
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),  # Translation matrix
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),  # Rotation matrix
    )

# Converts a ROS PointCloud2 message to a NumPy array
def msg_to_array(pc_msg):
    """
    Converts a ROS PointCloud2 message to a NumPy array of shape (N, 3) containing x, y, z coordinates.
    """
    pc_array = ros_numpy.numpify(pc_msg)  # Convert PointCloud2 to structured NumPy array
    pc = np.zeros([len(pc_array), 3])  # Initialize array for x, y, z
    pc[:, 0] = pc_array['x']  # Extract x coordinates
    pc[:, 1] = pc_array['y']  # Extract y coordinates
    pc[:, 2] = pc_array['z']  # Extract z coordinates
    return pc

# Performs ICP registration between a scan and a map at a given scale
def registration_at_scale(pc_scan, pc_map, initial, scale):
    """
    Performs Iterative Closest Point (ICP) registration between a scan and a map at a specified scale.
    Args:
        pc_scan: Source point cloud (current scan)
        pc_map: Target point cloud (global map or submap)
        initial: Initial 4x4 transformation guess
        scale: Voxel size scaling factor for downsampling
    Returns:
        transformation: Refined 4x4 transformation matrix
        fitness: ICP fitness score (higher is better)
    """
    result_icp = o3d.pipelines.registration.registration_icp(
        voxel_down_sample(pc_scan, SCAN_VOXEL_SIZE * scale),  # Downsampled scan
        voxel_down_sample(pc_map, MAP_VOXEL_SIZE * scale),    # Downsampled map
        1.0 * scale,  # Maximum correspondence distance
        initial,  # Initial transformation
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),  # Point-to-point ICP
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20)  # Convergence criteria
    )
    return result_icp.transformation, result_icp.fitness

# Computes the inverse of a 4x4 SE(3) transformation matrix
def inverse_se3(trans):
    """
    Computes the inverse of a 4x4 SE(3) transformation matrix.
    Args:
        trans: 4x4 transformation matrix (rotation + translation)
    Returns:
        trans_inverse: Inverse 4x4 transformation matrix
    """
    trans_inverse = np.eye(4)  # Initialize 4x4 identity matrix
    trans_inverse[:3, :3] = trans[:3, :3].T  # Inverse rotation (transpose of rotation matrix)
    trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])  # Inverse translation
    return trans_inverse

# Publishes a point cloud as a ROS PointCloud2 message
def publish_point_cloud(publisher, header, pc):
    """
    Publishes a point cloud as a ROS PointCloud2 message.
    Args:
        publisher: ROS publisher object
        header: ROS message header (includes frame_id and timestamp)
        pc: NumPy array of shape (N, 3) or (N, 4) containing point cloud data
    """
    # Create structured NumPy array for x, y, z, and optionally intensity
    data = np.zeros(len(pc), dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('intensity', np.float32),
    ])
    data['x'] = pc[:, 0]  # Assign x coordinates
    data['y'] = pc[:, 1]  # Assign y coordinates
    data['z'] = pc[:, 2]  # Assign z coordinates
    if pc.shape[1] == 4:
        data['intensity'] = pc[:, 3]  # Assign intensity if available
    msg = ros_numpy.msgify(PointCloud2, data)  # Convert to PointCloud2 message
    msg.header = header  # Set message header
    publisher.publish(msg)  # Publish message

# Crops the global map to points within the LiDAR's field of view (FOV)
def crop_global_map_in_FOV(global_map, pose_estimation, cur_odom):
    """
    Crops the global map to retain only points within the LiDAR's field of view (FOV).
    Args:
        global_map: Open3D point cloud of the global map
        pose_estimation: Estimated 4x4 transformation from map to odom
        cur_odom: Current odometry message
    Returns:
        global_map_in_FOV: Open3D point cloud containing points within the FOV
    """
    # Compute transformation from odom to base_link
    T_odom_to_base_link = pose_to_mat(cur_odom)
    # Compute transformation from map to base_link
    T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
    # Compute inverse transformation (base_link to map)
    T_base_link_to_map = inverse_se3(T_map_to_base_link)

    # Transform global map points to base_link frame
    global_map_in_map = np.array(global_map.points)  # Extract points
    global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])  # Homogeneous coordinates
    global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T  # Transform to base_link

    # Filter points within the FOV
    if FOV > 3.14:  # Ring-shaped LiDAR (e.g., 360-degree)
        indices = np.where(
            (global_map_in_base_link[:, 0] < FOV_FAR) &  # Within max distance
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)  # Within FOV angle
        )
    else:  # Non-ring LiDAR (forward-facing)
        indices = np.where(
            (global_map_in_base_link[:, 0] > 0) &  # Positive x (forward)
            (global_map_in_base_link[:, 0] < FOV_FAR) &  # Within max distance
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)  # Within FOV angle
        )

    # Create Open3D point cloud for points within FOV
    global_map_in_FOV = o3d.geometry.PointCloud()
    global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))

    # Publish the FOV point cloud as a ROS message
    header = cur_odom.header
    header.frame_id = 'map'
    publish_point_cloud(pub_submap, header, np.array(global_map_in_FOV.points)[::10])  # Downsample for publishing

    return global_map_in_FOV

# Performs global localization by aligning the current scan with the global map
def global_localization(pose_estimation):
    """
    Performs global localization by aligning the current scan with the global map using ICP.
    Args:
        pose_estimation: Initial 4x4 transformation guess from map to odom
    Returns:
        bool: True if localization is successful (fitness score > threshold), False otherwise
    """
    global global_map, cur_scan, cur_odom, T_map_to_odom
    rospy.loginfo('Global localization by scan-to-map matching......')

    # Create a copy of the current scan to ensure thread safety
    scan_tobe_mapped = copy.copy(cur_scan)

    tic = time.time()  # Start timing

    # Crop the global map to the LiDAR's FOV
    global_map_in_FOV = crop_global_map_in_FOV(global_map, pose_estimation, cur_odom)

    # Coarse registration (larger voxel size)
    transformation, _ = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5)

    # Fine registration (smaller voxel size)
    transformation, fitness = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=transformation, scale=1)

    toc = time.time()  # End timing
    rospy.loginfo('Time: {}'.format(toc - tic))

    # Update map-to-odom transformation if localization is successful
    if fitness > LOCALIZATION_TH:
        T_map_to_odom = transformation  # Update global transformation

        # Publish the map-to-odom transformation as an Odometry message
        map_to_odom = Odometry()
        xyz = tf.transformations.translation_from_matrix(T_map_to_odom)  # Extract translation
        quat = tf.transformations.quaternion_from_matrix(T_map_to_odom)  # Extract quaternion
        map_to_odom.pose.pose = Pose(Point(*xyz), Quaternion(*quat))  # Set pose
        map_to_odom.header.stamp = cur_odom.header.stamp  # Set timestamp
        map_to_odom.header.frame_id = 'map'  # Set frame
        pub_map_to_odom.publish(map_to_odom)
        return True
    else:
        # Log failure if fitness score is too low
        rospy.logwarn('Not match!!!!')
        rospy.logwarn('{}'.format(transformation))
        rospy.logwarn('fitness score:{}'.format(fitness))
        return False

# Downsamples a point cloud using voxelization
def voxel_down_sample(pcd, voxel_size):
    """
    Downsamples a point cloud using voxelization.
    Args:
        pcd: Open3D point cloud
        voxel_size: Size of the voxel grid
    Returns:
        pcd_down: Downsampled Open3D point cloud
    """
    try:
        pcd_down = pcd.voxel_down_sample(voxel_size)  # Standard method (Open3D >= 0.8)
    except:
        # Fallback for older Open3D versions (<= 0.7)
        pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
    return pcd_down

# Initializes the global map from a PointCloud2 message
def initialize_global_map(pc_msg):
    """
    Initializes the global point cloud map from a ROS PointCloud2 message.
    Args:
        pc_msg: ROS PointCloud2 message containing the global map
    """
    global global_map
    global_map = o3d.geometry.PointCloud()
    global_map.points = o3d.utility.Vector3dVector(msg_to_array(pc_msg)[:, :3])  # Extract x, y, z
    global_map = voxel_down_sample(global_map, MAP_VOXEL_SIZE)  # Downsample map
    rospy.loginfo('Global map received.')

# Callback to store the current odometry message
def cb_save_cur_odom(odom_msg):
    """
    Callback to store the current odometry message.
    Args:
        odom_msg: ROS Odometry message
    """
    global cur_odom
    cur_odom = odom_msg

# Callback to store and process the current point cloud scan
def cb_save_cur_scan(pc_msg):
    """
    Callback to store and process the current point cloud scan.
    Publishes the scan in the map frame and converts it to an Open3D point cloud.
    Args:
        pc_msg: ROS PointCloud2 message containing the scan
    """
    global cur_scan
    # Adjust frame_id and timestamp (FastLIO provides scan in odom frame)
    pc_msg.header.frame_id = 'camera_init'
    pc_msg.header.stamp = rospy.Time().now()
    pub_pc_in_map.publish(pc_msg)  # Publish scan

    # Reorder fields to fix FastLIO's field ordering issue
    pc_msg.fields = [pc_msg.fields[0], pc_msg.fields[1], pc_msg.fields[2],
                     pc_msg.fields[4], pc_msg.fields[5], pc_msg.fields[6],
                     pc_msg.fields[3], pc_msg.fields[7]]
    pc = msg_to_array(pc_msg)  # Convert to NumPy array

    # Convert to Open3D point cloud
    cur_scan = o3d.geometry.PointCloud()
    cur_scan.points = o3d.utility.Vector3dVector(pc[:, :3])

# Thread function for periodic global localization
def thread_localization():
    """
    Thread function that periodically performs global localization at a specified frequency.
    """
    global T_map_to_odom
    while True:
        rospy.sleep(1 / FREQ_LOCALIZATION)  # Sleep based on localization frequency
        # Use the previous map-to-odom transformation as the initial guess
        global_localization(T_map_to_odom)

# Main function
if __name__ == '__main__':
    # Configuration parameters
    MAP_VOXEL_SIZE = 0.4  # Voxel size for downsampling the global map
    SCAN_VOXEL_SIZE = 0.1  # Voxel size for downsampling the scan
    FREQ_LOCALIZATION = 0.5  # Global localization frequency (Hz)
    LOCALIZATION_TH = 0.95  # Fitness score threshold for accepting localization
    FOV = 6.28  # Field of view in radians (360 degrees for ring LiDAR)
    FOV_FAR = 30  # Maximum distance for FOV filtering (meters)

    # Initialize ROS node
    rospy.init_node('fast_lio_localization')
    rospy.loginfo('Localization Node Inited...')

    # Initialize ROS publishers
    pub_pc_in_map = rospy.Publisher('/cur_scan_in_map', PointCloud2, queue_size=1)  # Current scan in map frame
    pub_submap = rospy.Publisher('/submap', PointCloud2, queue_size=1)  # Submap (FOV points)
    pub_map_to_odom = rospy.Publisher('/map_to_odom', Odometry, queue_size=1)  # Map-to-odom transformation

    # Initialize ROS subscribers
    rospy.Subscriber('/cloud_registered', PointCloud2, cb_save_cur_scan, queue_size=1)  # Point cloud scans
    rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1)  # Odometry data

    # Wait for and initialize the global map
    rospy.logwarn('Waiting for global map......')
    initialize_global_map(rospy.wait_for_message('/map', PointCloud2))

    # Wait for initial pose and perform initial localization
    while not initialized:
        rospy.logwarn('Waiting for initial pose....')
        pose_msg = rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped)  # Wait for initial pose
        initial_pose = pose_to_mat(pose_msg)  # Convert to transformation matrix
        if cur_scan:
            initialized = global_localization(initial_pose)  # Attempt initial localization
        else:
            rospy.logwarn('First scan not received!!!!!')

    # Log successful initialization
    rospy.loginfo('')
    rospy.loginfo('Initialize successfully!!!!!!')
    rospy.loginfo('')

    # Start periodic localization in a separate thread
    _thread.start_new_thread(thread_localization, ())

    # Keep the node running
    rospy.spin()