//
// Created by dinhnambkhn on 26/11/2024.
//
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h> // For quaternion computation
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

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

        ROS_INFO("Loading path from: %s", file_name_.c_str());

        // Load the path from the file
        if (!loadPathFromFile()) {
            ROS_ERROR("Failed to load path from file: %s", file_name_.c_str());
            ros::shutdown();
            return;
        }

        // Initialize publisher
        path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("loaded_path", 10);

        // Start timer to publish path at 100ms intervals
        timer_ = nh_.createTimer(ros::Duration(0.1), &PathPublisher::timerCallback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher path_pub_;
    ros::Timer timer_;
    std::string file_name_;
    geometry_msgs::PoseArray path_;

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
