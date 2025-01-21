//
// Created by dinhnambkhn on 21/11/2024.
//
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <cmath>
#include <spline.h> // Include the spline library (https://kluge.in-chemnitz.de/opensource/spline/)

class CircleSplinePublisher {
public:
    CircleSplinePublisher() {
        ros::NodeHandle nh("~");

        // Parameters
        nh.param("circle_center_x", circle_center_x_, 0.0);
        nh.param("circle_center_y", circle_center_y_, 0.0);
        nh.param("circle_radius", circle_radius_, 1.0);
        nh.param("num_points", num_points_, 50);

        // Publishers
        points_pub_ = nh.advertise<visualization_msgs::Marker>("circle_points", 10);
        line_pub_ = nh.advertise<visualization_msgs::Marker>("spline_line", 10);

        // Timer
        timer_ = nh.createTimer(ros::Duration(0.1), &CircleSplinePublisher::publishCallback, this);

        ROS_INFO("CircleSplinePublisher initialized.");
    }

private:
    ros::Publisher points_pub_;
    ros::Publisher line_pub_;
    ros::Timer timer_;

    double circle_center_x_, circle_center_y_, circle_radius_;
    int num_points_;

    void generateCirclePoints(std::vector<double>& x, std::vector<double>& y) const {
        x.clear();
        y.clear();

        for (int i = 0; i < num_points_; ++i) {
            const double angle = 2 * M_PI * i / num_points_;
            x.push_back(circle_center_x_ + circle_radius_ * cos(angle));
            y.push_back(circle_center_y_ + circle_radius_ * sin(angle));
        }
    }

    void publishCallback(const ros::TimerEvent&) const {
        // Generate circle points
        std::vector<double> x, y;
        generateCirclePoints(x, y);

        // Fit a spline to the circle points
        tk::spline spline_x, spline_y;
        std::vector<double> t(num_points_);
        for (int i = 0; i < num_points_; ++i) {
            t[i] = i; // Parameterize with integers
        }
        spline_x.set_points(t, x);
        spline_y.set_points(t, y);

        // Generate points on the spline
        std::vector<double> spline_x_points, spline_y_points;
        for (int i = 0; i < 10 * num_points_; ++i) {
            const double ti = i * 1.0 / 10;
            spline_x_points.push_back(spline_x(ti));
            spline_y_points.push_back(spline_y(ti));
        }

        // Publish circle points as markers
        publishPointsMarker(x, y);

        // Publish spline as a line marker
        publishLineMarker(spline_x_points, spline_y_points);
    }

    void publishPointsMarker(const std::vector<double>& x, const std::vector<double>& y) const {
        visualization_msgs::Marker points_marker;
        points_marker.header.frame_id = "map";
        points_marker.header.stamp = ros::Time::now();
        points_marker.ns = "circle_points";
        points_marker.id = 0;
        points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        points_marker.action = visualization_msgs::Marker::ADD;

        points_marker.scale.x = 0.1; // Point size
        points_marker.scale.y = 0.1;
        points_marker.color.r = 0.0;
        points_marker.color.g = 1.0;
        points_marker.color.b = 0.0;
        points_marker.color.a = 1.0;

        for (size_t i = 0; i < x.size(); ++i) {
            geometry_msgs::Point p;
            p.x = x[i];
            p.y = y[i];
            p.z = 0.0;
            points_marker.points.push_back(p);
        }

        points_pub_.publish(points_marker);
    }

    void publishLineMarker(const std::vector<double>& x, const std::vector<double>& y) const {
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "spline_line";
        line_marker.id = 1;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;

        line_marker.scale.x = 0.02; // Line width
        line_marker.color.r = 1.0;
        line_marker.color.g = 0.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 1.0;

        for (size_t i = 0; i < x.size(); ++i) {
            geometry_msgs::Point p;
            p.x = x[i];
            p.y = y[i];
            p.z = 0.0;
            line_marker.points.push_back(p);
        }

        line_pub_.publish(line_marker);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "circle_spline_publisher");

    CircleSplinePublisher publisher;

    ros::spin();

    return 0;
}
