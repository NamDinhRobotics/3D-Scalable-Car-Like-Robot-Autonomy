#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <spline.h> // Include the cubic spline library

class CubicSplineRViz
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Publisher line_marker_pub_;   // Publishes the cubic spline
    ros::Publisher point_marker_pub_; // Publishes individual points
    std::vector<Eigen::Vector2d> points_; // Stores selected points as Eigen::Vector2d

public:
    CubicSplineRViz()
    {
        // Subscribe to /initialpose
        pose_sub_ = nh_.subscribe("/initialpose", 10, &CubicSplineRViz::poseCallback, this);
        line_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("cubic_spline_marker", 10);
        point_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("clicked_points_marker", 10);
    }

    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
    {
        // Extract x and y coordinates from PoseWithCovarianceStamped
        points_.emplace_back(msg->pose.pose.position.x, msg->pose.pose.position.y);

        // Publish the individual points as markers
        publishPointMarkers();

        if (points_.size() > 2)
        {
            fitAndVisualize();
        }
    }

    void publishPointMarkers() const {
        visualization_msgs::Marker point_marker;
        point_marker.header.frame_id = "map";
        point_marker.header.stamp = ros::Time::now();
        point_marker.ns = "clicked_points";
        point_marker.id = 0;
        point_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        point_marker.action = visualization_msgs::Marker::ADD;
        point_marker.scale.x = 0.2; // Marker size
        point_marker.scale.y = 0.2;
        point_marker.scale.z = 0.2;
        point_marker.color.r = 1.0; // Red points
        point_marker.color.g = 0.0;
        point_marker.color.b = 0.0;
        point_marker.color.a = 1.0;

        for (const auto& point : points_)
        {
            geometry_msgs::Point p;
            p.x = point.x(); // x-coordinate
            p.y = point.y(); // y-coordinate
            p.z = 0.0;
            point_marker.points.push_back(p);
        }

        // Publish the point markers
        point_marker_pub_.publish(point_marker);
    }

    void fitAndVisualize()
    {
        const size_t n = points_.size();
        std::vector<double> x(n), y(n);

        // Extract x and y from points
        for (size_t i = 0; i < n; ++i)
        {
            x[i] = points_[i].x();
            y[i] = points_[i].y();
        }

        // Fit cubic splines using the spline library
        tk::spline spline_x, spline_y;
        std::vector<double> t(n); // Parameterization
        for (size_t i = 0; i < n; ++i)
        {
            t[i] = i; // Assign sequential integers as parameter values
        }
        spline_x.set_points(t, x);
        spline_y.set_points(t, y);

        // Generate spline visualization
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "cubic_spline";
        line_marker.id = 1;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.scale.x = 0.05; // Line width
        line_marker.color.r = 0.0; // Green line
        line_marker.color.g = 1.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 1.0;

        // Generate points along the spline for visualization
        constexpr int resolution = 10; // Number of points per segment
        for (int i = 0; i < resolution * (n - 1); ++i)
        {
            const double ti = i * 1.0 / resolution;
            geometry_msgs::Point p;
            p.x = spline_x(ti);
            p.y = spline_y(ti);
            p.z = 0.0;
            line_marker.points.push_back(p);
        }

        // Publish the spline marker
        line_marker_pub_.publish(line_marker);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cubic_spline_rviz");
    CubicSplineRViz node;
    ros::spin();
    return 0;
}
