#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h> // For quaternion computation
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include "spline_rw.h" // Include spline library (e.g., tinyspline or tk::spline)

class PathPublisher {
public:
    PathPublisher() {
        // Use a private NodeHandle for parameters
        ros::NodeHandle nh_private("~");

        // Get file path parameter
        if (!nh_private.getParam("file_path", file_name_)) {
            ROS_ERROR("Parameter 'file_path' not found! Node will exit.");
            ros::shutdown();
            return;
        }

        // Get step size parameter
        nh_private.param("step_size", step_, 0.05); // Default to 0.05 meters
        ROS_INFO("Equidistant step size: %.2f", step_);

        ROS_INFO("Loading path from: %s", file_name_.c_str());

        // Load the path from the file
        if (!loadPathFromFile()) {
            ROS_ERROR("Failed to load path from file: %s", file_name_.c_str());
            ros::shutdown();
            return;
        }

        // Smooth and interpolate the path
        if (!smoothAndInterpolatePath()) {
            ROS_ERROR("Failed to interpolate the path.");
            ros::shutdown();
            return;
        }
        //path_name_
        nh_private.param("path_name", path_name_, std::string("loaded_path"));

        // Initialize publisher
        path_pub_ = nh_.advertise<geometry_msgs::PoseArray>(path_name_, 10);

        // Start timer to publish path at 100ms intervals
        timer_ = nh_.createTimer(ros::Duration(0.1), &PathPublisher::timerCallback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher path_pub_;
    ros::Timer timer_;
    std::string file_name_;
    double step_{}; // Equidistant step size
    geometry_msgs::PoseArray path_;
    //topic name: loaded_path
    std::string path_name_;

    bool loadPathFromFile() {
        std::ifstream file(file_name_);
        if (!file.is_open()) {
            ROS_ERROR("Unable to open file: %s", file_name_.c_str());
            return false;
        }

        std::string line;
        bool header_skipped = false;

        while (std::getline(file, line)) {
            // Skip header if present
            if (!header_skipped) {
                header_skipped = true;
                continue;
            }

            std::stringstream ss(line);
            std::string timestamp, x_str, y_str, yaw_str, qw_str, qx_str, qy_str, qz_str;

            // Parse CSV fields
            if (!(std::getline(ss, timestamp, ',') &&
                  std::getline(ss, x_str, ',') &&
                  std::getline(ss, y_str, ',') &&
                  std::getline(ss, yaw_str, ',') &&
                  std::getline(ss, qw_str, ',') &&
                  std::getline(ss, qx_str, ',') &&
                  std::getline(ss, qy_str, ',') &&
                  std::getline(ss, qz_str, ','))) {
                ROS_WARN("Malformed line in file: %s", line.c_str());
                continue;
            }

            // Convert to double
            try {
                double x = std::stod(x_str);
                double y = std::stod(y_str);
                double qw = std::stod(qw_str);
                double qx = std::stod(qx_str);
                double qy = std::stod(qy_str);
                double qz = std::stod(qz_str);

                // Create Pose
                geometry_msgs::Pose pose;
                pose.position.x = x;
                pose.position.y = y;
                pose.position.z = 0.0;
                pose.orientation.w = qw;
                pose.orientation.x = qx;
                pose.orientation.y = qy;
                pose.orientation.z = qz;

                path_.poses.push_back(pose);
            } catch (const std::exception& e) {
                ROS_WARN("Error parsing line: %s", e.what());
                continue;
            }
        }

        file.close();

        if (path_.poses.empty()) {
            ROS_ERROR("No valid data found in file.");
            return false;
        }

        // Set the header for the PoseArray
        path_.header.frame_id = "map";
        ROS_INFO("Loaded %lu poses from file.", path_.poses.size());
        return true;
    }

    bool smoothAndInterpolatePath() {
        if (path_.poses.size() < 2) {
            ROS_ERROR("Insufficient waypoints to interpolate.");
            return false;
        }

        const size_t n = path_.poses.size();
        std::vector<double> x(n), y(n);

        // Extract x and y from poses
        for (size_t i = 0; i < n; ++i) {
            x[i] = path_.poses[i].position.x;
            y[i] = path_.poses[i].position.y;
        }

        // Parameterize points with t
        std::vector<double> t(n);
        for (size_t i = 0; i < n; ++i) {
            t[i] = i;  // Use index as parameter
        }

        // Fit splines to x(t) and y(t)
        Spline spline_x, spline_y;
        spline_x.fit(t, x);
        spline_y.fit(t, y);

        // Generate interpolated points
        std::vector<geometry_msgs::Pose> smoothed_poses;
        constexpr int resolution = 10; // Number of samples between each original point
        for (size_t i = 0; i < n - 1; ++i) { // Iterate between original points
            for (int j = 0; j < resolution; ++j) { // Sample between points
                double ti = t[i] + static_cast<double>(j) / resolution;
                geometry_msgs::Pose pose;
                pose.position.x = spline_x.evaluate(ti);
                pose.position.y = spline_y.evaluate(ti);
                pose.position.z = 0.0; // Keep Z constant

                // Use original orientation (optional: interpolate orientation)
                pose.orientation = path_.poses[i].orientation;

                smoothed_poses.push_back(pose);
            }
        }

        // Add the final pose explicitly
        smoothed_poses.push_back(path_.poses.back());

        // Replace original path with interpolated path
        path_.poses = smoothed_poses;
        ROS_INFO("Interpolated path with %lu equidistant points.", path_.poses.size());
        return true;
    }


    void timerCallback(const ros::TimerEvent&) {
        path_.header.stamp = ros::Time::now();
        path_pub_.publish(path_);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_publisher");
    PathPublisher node;
    ros::spin();
    return 0;
}
