#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include "spline_rw.h"

class CircleSplineRViz {
private:
    ros::NodeHandle nh_;
    ros::Timer timer_;
    ros::Publisher circle_point_pub_; // Publishes the circle points
    ros::Publisher spline_line_pub_;  // Publishes the cubic spline line
    std::vector<Eigen::Vector2d> points_; // Stores the circle points

    double circle_center_x_;
    double circle_center_y_;
    double circle_radius_;
    int num_points_;

public:
    CircleSplineRViz()
    {
        // Get parameters
        nh_.param("circle_center_x", circle_center_x_, 0.0);
        nh_.param("circle_center_y", circle_center_y_, 0.0);
        nh_.param("circle_radius", circle_radius_, 1.0);
        nh_.param("num_points", num_points_, 50);

        // Publishers
        circle_point_pub_ = nh_.advertise<visualization_msgs::Marker>("circle_points_marker", 10);
        spline_line_pub_ = nh_.advertise<visualization_msgs::Marker>("cubic_spline_marker", 10);

        // Timer to periodically update markers
        timer_ = nh_.createTimer(ros::Duration(0.1), &CircleSplineRViz::timerCallback, this);

        ROS_INFO("CircleSplineRViz initialized.");
    }

    void timerCallback(const ros::TimerEvent&)
    {
        generateCirclePoints();
        publishCirclePoints();
        fitAndVisualizeSpline();
    }

    void generateCirclePoints()
    {
        points_.clear();
        for (int i = 0; i < num_points_; ++i)
        {
            double angle = 2 * M_PI * i / num_points_;
            double x = circle_center_x_ + circle_radius_ * cos(angle);
            double y = circle_center_y_ + circle_radius_ * sin(angle);
            points_.emplace_back(x, y);
        }
    }

    void publishCirclePoints() const
    {
        visualization_msgs::Marker point_marker;
        point_marker.header.frame_id = "map";
        point_marker.header.stamp = ros::Time::now();
        point_marker.ns = "circle_points";
        point_marker.id = 0;
        point_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        point_marker.action = visualization_msgs::Marker::ADD;

        point_marker.scale.x = 0.05; // Marker size
        point_marker.scale.y = 0.05;
        point_marker.scale.z = 0.05;
        point_marker.color.r = 1.0; // Red points
        point_marker.color.g = 0.0;
        point_marker.color.b = 0.0;
        point_marker.color.a = 1.0;

        for (const auto &point : points_)
        {
            geometry_msgs::Point p;
            p.x = point.x();
            p.y = point.y();
            p.z = 0.0;
            point_marker.points.push_back(p);
        }

        // Publish the circle points
        circle_point_pub_.publish(point_marker);
    }

    void fitAndVisualizeSpline()
    {
        const size_t n = points_.size();
        std::vector<double> x(n), y(n);

        for (size_t i = 0; i < n; ++i)
        {
            x[i] = points_[i].x();
            y[i] = points_[i].y();
        }

        std::vector<double> t(n);
        for (size_t i = 0; i < n; ++i) {
            t[i] = i;
        }

        Spline spline_x, spline_y;
        spline_x.fit(t, x);
        spline_y.fit(t, y);

        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "cubic_spline";
        line_marker.id = 1;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;

        line_marker.scale.x = 0.03; // Line width
        line_marker.color.r = 0.0; // Green line
        line_marker.color.g = 1.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 1.0;

        constexpr int resolution = 100;
        for (int i = 0; i < resolution * (n - 1); ++i) {
            double ti = i * 1.0 / resolution;
            geometry_msgs::Point p;
            p.x = spline_x.evaluate(ti);
            p.y = spline_y.evaluate(ti);
            p.z = 0.0;
            line_marker.points.push_back(p);
        }

        spline_line_pub_.publish(line_marker);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle_spline_rviz");
    CircleSplineRViz node;
    ros::spin();
    return 0;
}
