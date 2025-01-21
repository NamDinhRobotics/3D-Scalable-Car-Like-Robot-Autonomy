#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
import numpy as np
import tf.transformations as tft


class LaserToMapTransformer:
    def __init__(self):
        rospy.init_node('laser_to_map_transformer')

        # Subscriber for Odometry data
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Subscriber for PointCloud2 in laser frame
        self.pointcloud_sub = rospy.Subscriber('/pointcloud', PointCloud2, self.pointcloud_callback)

        # Publisher for transformed PointCloud2 in map frame
        self.pointcloud_pub = rospy.Publisher('/pointcloud_map', PointCloud2, queue_size=10)

        # Latest odometry transform
        self.odom_transform = None

        # Rate for publishing at 20 Hz
        self.rate = rospy.Rate(20)

        # Transformed message buffer
        self.transformed_pointcloud_msg = None

    def odom_callback(self, odom_msg):
        """Callback to process Odometry data."""
        position = odom_msg.pose.pose.position
        orientation = odom_msg.pose.pose.orientation

        # Convert Odometry to transformation matrix
        translation = [position.x, position.y, position.z]
        rotation = [orientation.x, orientation.y, orientation.z, orientation.w]
        self.odom_transform = tft.compose_matrix(translate=translation, angles=tft.euler_from_quaternion(rotation))

    def apply_transform(self, pointcloud_msg, transform_matrix):
        """Apply the transform to the point cloud."""
        points = np.array(list(pc2.read_points(pointcloud_msg, skip_nans=True)))
        points_xyz = np.ones((len(points), 4))  # Homogeneous coordinates
        points_xyz[:, :3] = points[:, :3]  # Copy x, y, z coordinates

        # Transform points
        transformed_points = (transform_matrix @ points_xyz.T).T

        # Create a new PointCloud2 message
        transformed_msg = PointCloud2()
        transformed_msg.header = pointcloud_msg.header
        transformed_msg.header.frame_id = "map"  # Target frame
        transformed_msg = pc2.create_cloud_xyz32(transformed_msg.header, transformed_points[:, :3])

        return transformed_msg

    def pointcloud_callback(self, pointcloud_msg):
        """Callback to transform and publish the point cloud."""
        if self.odom_transform is None:
            rospy.logwarn("Odometry transform not available yet.")
            return

        try:
            # Apply the transformation
            self.transformed_pointcloud_msg = self.apply_transform(pointcloud_msg, self.odom_transform)

        except Exception as e:
            rospy.logwarn(f"Point cloud transformation failed: {e}")

    def run(self):
        """Main loop for publishing at 20 Hz."""
        while not rospy.is_shutdown():
            if self.transformed_pointcloud_msg:
                self.pointcloud_pub.publish(self.transformed_pointcloud_msg)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        transformer = LaserToMapTransformer()
        transformer.run()
    except rospy.ROSInterruptException:
        pass
