#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection

class LaserScanToPointCloud:
    def __init__(self):
        rospy.init_node('laserscan_to_pointcloud')
        self.laser_proj = LaserProjection()
        self.pointcloud_pub = rospy.Publisher('/pointcloud', PointCloud2, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

    def scan_callback(self, scan_msg):
        # Convert LaserScan to PointCloud2
        pointcloud_msg = self.laser_proj.projectLaser(scan_msg)

        # Publish the point cloud
        self.pointcloud_pub.publish(pointcloud_msg)

if __name__ == '__main__':
    try:
        LaserScanToPointCloud()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
