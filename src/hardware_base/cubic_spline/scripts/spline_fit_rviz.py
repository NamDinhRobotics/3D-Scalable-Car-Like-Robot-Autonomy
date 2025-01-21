#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import numpy as np

class CubicSplineRViz:
    def __init__(self):
        # Initialize node
        rospy.init_node("cubic_spline_rviz")

        # Publisher for visualization markers
        self.marker_pub = rospy.Publisher("cubic_spline_marker", Marker, queue_size=10)

        # List to store clicked points
        self.points = []

        # Subscriber for clicked points in RViz
        rospy.Subscriber("/clicked_point", Point, self.point_callback)

    def point_callback(self, msg):
        # Store the clicked point
        self.points.append([msg.x, msg.y])

        # Fit and visualize if more than 2 points are present
        if len(self.points) > 2:
            self.fit_and_visualize()

    def fit_and_visualize(self):
        # Extract x and y from points
        points_np = np.array(self.points)
        x_points = points_np[:, 0]
        y_points = points_np[:, 1]

        # Compute spline fit
        n = len(x_points)
        h = np.diff(x_points)
        A = np.zeros((n, n))
        R = np.zeros(n)

        # Natural spline boundary conditions
        A[0, 0] = 1
        A[-1, -1] = 1

        # Fill the tridiagonal matrix
        for i in range(1, n - 1):
            A[i, i - 1] = h[i - 1]
            A[i, i] = 2 * (h[i - 1] + h[i])
            A[i, i + 1] = h[i]
            R[i] = 6 * ((y_points[i + 1] - y_points[i]) / h[i] - (y_points[i] - y_points[i - 1]) / h[i - 1])

        # Solve for second derivatives
        M = np.linalg.solve(A, R)

        # Generate spline visualization
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.05
        marker.color = ColorRGBA(0, 1, 0, 1)

        # Compute cubic spline points
        x_fine = np.linspace(x_points[0], x_points[-1], 100)
        y_fine = []

        for x in x_fine:
            for i in range(n - 1):
                if x_points[i] <= x <= x_points[i + 1]:
                    a = y_points[i]
                    b = (y_points[i + 1] - y_points[i]) / h[i] - h[i] * (2 * M[i] + M[i + 1]) / 6
                    c = M[i] / 2
                    d = (M[i + 1] - M[i]) / (6 * h[i])
                    dx = x - x_points[i]
                    y_fine.append(a + b * dx + c * dx**2 + d * dx**3)
                    break

        # Add points to marker
        for i in range(len(x_fine)):
            marker.points.append(Point(x_fine[i], y_fine[i], 0))

        # Publish marker
        self.marker_pub.publish(marker)

if __name__ == "__main__":
    try:
        node = CubicSplineRViz()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

