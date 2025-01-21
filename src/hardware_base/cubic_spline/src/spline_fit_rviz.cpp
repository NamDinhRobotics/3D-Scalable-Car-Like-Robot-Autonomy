#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h> // For quaternion computation
#include <fstream>
#include <iostream>
#include "spline_rw.h"

class CubicSplineRViz {
public:
    CubicSplineRViz() {
        // Use a private NodeHandle for parameters
        ros::NodeHandle nh_private("~");

        // Retrieve the file path parameter
        if (!nh_private.getParam("file_path", file_name_)) {
            ROS_WARN("Parameter 'file_path' not found! Using default value.");
            file_name_ = "/home/dinhnambkhn/UGV_HighSpeed/src/cubic_spline/paths/spline_path.csv";
        }

        ROS_INFO("Using file path: %s", file_name_.c_str());

        //spline_waypoints
        nh_private.param("waypoints_topic", waypoints_topic_, std::string("spline_waypoints"));

        // Subscribe to /move_base_simple/goal
        pose_sub_ = nh_.subscribe("/move_base_simple/goal", 10, &CubicSplineRViz::poseCallback, this);
        line_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("cubic_spline_marker", 10);
        point_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("clicked_points_marker", 10);
        waypoints_pub_ = nh_.advertise<geometry_msgs::PoseArray>(waypoints_topic_, 10);

        // Timer for periodic waypoint publication
        timer_ = nh_.createTimer(ros::Duration(0.1), &CubicSplineRViz::timerCallback, this);
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        // Extract x and y coordinates from PoseStamped
        points_.emplace_back(msg->pose.position.x, msg->pose.position.y);

        // Publish the individual points as markers
        publishPointMarkers();

        if (points_.size() > 2) {
            fitAndVisualize();
        }
    }

    void timerCallback(const ros::TimerEvent&) {
        // Periodically publish the waypoints
        if (!waypoints_.poses.empty()) {
            waypoints_pub_.publish(waypoints_);
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

        for (const auto &point : points_) {
            geometry_msgs::Point p;
            p.x = point.x(); // x-coordinate
            p.y = point.y(); // y-coordinate
            p.z = 0.0;
            point_marker.points.push_back(p);
        }

        // Publish the point markers
        point_marker_pub_.publish(point_marker);
    }

    void fitAndVisualize() {
        const size_t n = points_.size();
        std::vector<double> x(n), y(n);

        // Extract x and y from points
        for (size_t i = 0; i < n; ++i) {
            x[i] = points_[i].x();
            y[i] = points_[i].y();
        }

        // Parameterize points
        std::vector<double> t(n);
        for (auto i = 0; i < n; ++i) {
            t[i] = i;
        }

        // Fit splines
        Spline spline_x, spline_y;
        spline_x.fit(t, x);
        spline_y.fit(t, y);

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

        waypoints_.poses.clear();
        waypoints_.header.frame_id = "map";
        waypoints_.header.stamp = ros::Time::now();

        std::ofstream csv_file(file_name_, std::ios::out);

        // Handle file opening errors
        if (!csv_file) {
            std::cerr << "Error: Could not open file " << file_name_ << " for writing." << std::endl;
            return;
        }

        csv_file << "timestamp,x,y,yaw,q_w,q_x,q_y,q_z\n";

        constexpr int resolution = 10; // Number of points to sample
        for (int i = 0; i <= resolution * (n - 1); ++i) {
            const double ti = static_cast<double>(i) / resolution;
            geometry_msgs::Point p;
            p.x = spline_x.evaluate(ti);
            p.y = spline_y.evaluate(ti);
            p.z = 0.0;
            line_marker.points.push_back(p);

            // Compute tangent-based orientation
            geometry_msgs::Pose pose;
            pose.position.x = p.x;
            pose.position.y = p.y;
            pose.position.z = 0.0;

            double dx = spline_x.derivative(ti);
            double dy = spline_y.derivative(ti);
            double heading = atan2(dy, dx);

            // Convert heading to quaternion
            tf::Quaternion q;
            q.setRPY(0, 0, heading);
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();

            waypoints_.poses.push_back(pose);

            // Write to CSV file
            csv_file << ros::Time::now() << "," << p.x << "," << p.y << "," << heading << ","
                     << pose.orientation.w << "," << pose.orientation.x << ","
                     << pose.orientation.y << "," << pose.orientation.z << "\n";
        }

        csv_file.close();
        ROS_INFO("Path successfully saved to %s", file_name_.c_str());
        std::cout << "Path saved to file: " << file_name_ << std::endl;

        // Publish the spline marker
        line_marker_pub_.publish(line_marker);
    }

private:
    ros::NodeHandle nh_;
    std::string file_name_;
    ros::Subscriber pose_sub_;
    ros::Publisher line_marker_pub_;  // Publishes the cubic spline
    ros::Publisher point_marker_pub_; // Publishes individual points
    ros::Publisher waypoints_pub_;    // Publishes waypoints
    ros::Timer timer_;                // Timer for periodic waypoint publication
    geometry_msgs::PoseArray waypoints_; // Stores waypoints
    std::vector<Eigen::Vector2d> points_; // Stores selected points as Eigen::Vector2d
    std::string waypoints_topic_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "cubic_spline_rviz");
    CubicSplineRViz node;
    ros::spin();
    return 0;
}
