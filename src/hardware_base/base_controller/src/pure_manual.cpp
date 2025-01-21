#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Joy.h> // For joystick input
#include <tf/tf.h>
#include <cmath>
#include "utils.h"

class PurePursuitController {
public:
    PurePursuitController() {
        // Load parameters
        ros::NodeHandle nh_private("~");
        nh_private.param("lookahead_distance", lookahead_distance_, 2.0);
        nh_private.param("max_speed", max_speed_, 4.0);
        nh_private.param("min_speed", min_speed_, 0.22);
        nh_private.param("max_steering_angle", max_steering_angle_, 0.5);
        //wheel_base_
        nh_private.param("wheel_base", wheel_base_, 0.36);
        //odom topic
        nh_private.param("odometry_topic", odometry_topic_, std::string("odometry_livox"));
        //path topic
        nh_private.param("path_topic", path_topic_, std::string("loaded_path"));

        // Subscribers
        odom_sub_ = nh_.subscribe(odometry_topic_, 10, &PurePursuitController::odomCallback, this);
        path_sub_ = nh_.subscribe(path_topic_, 10, &PurePursuitController::pathCallback, this);
        joy_sub_ = nh_.subscribe("joy", 10, &PurePursuitController::joyCallback, this); // Joystick input

        // Publishers
        ackermann_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 20);
        lookahead_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("lookahead_marker", 10);

        waypoints_.reserve(1000);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        current_pose_ = msg->pose.pose;
        current_speed_ = sqrt(msg->twist.twist.linear.x * msg->twist.twist.linear.x +
                              msg->twist.twist.linear.y * msg->twist.twist.linear.y);
        has_odom_ = true;

        // Compute and publish control commands if path is available
        if (has_path_ && has_odom_ && !manual_mode_) {
            computeControl();
        }
    }
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg) {
        //ROS_INFO joy_msg->buttons[button_lb_] and joy_msg->buttons[button_rb_] for bool
        //ROS_INFO("joy_msg->buttons[button_lb_]: %d", joy_msg->buttons[button_lb_]);
        //ROS_INFO("joy_msg->buttons[button_rb_]: %d", joy_msg->buttons[button_rb_]);
        // Switch to manual mode if LB is pressed
        if (joy_msg->buttons[button_lb_] ) {
            manual_mode_ = true;
            //ROS_INFO("Switched to MANUAL mode.");
            ROS_INFO("\033[1;33mSwitched to MANUAL mode.\033[0m");

        }

        // Switch back to auto mode if RB is pressed
        if (joy_msg->buttons[button_rb_]) {
            manual_mode_ = false;
            //ROS_INFO("Switched to AUTO mode.");
            ROS_INFO("\033[1;32mSwitched to AUTO mode.\033[0m");

        }

        // In manual mode, get speed and steering input from joystick axes
        if (manual_mode_) {
            manual_speed_ = max_speed_ * joy_msg->axes[axis_speed_]; // Speed scaling
            manual_steering_ = max_steering_angle_ * joy_msg->axes[axis_steering_]; // Steering scaling

            // Publish manual control commands
            ackermann_msgs::AckermannDriveStamped drive_msg;
            drive_msg.drive.speed = manual_speed_;
            drive_msg.drive.steering_angle = manual_steering_;
            ackermann_pub_.publish(drive_msg);

            ROS_INFO("Manual Control - Speed: %.2f, Steering: %.2f", manual_speed_, manual_steering_);
        }
    }
    void pathCallback(const geometry_msgs::PoseArray::ConstPtr &msg) {
        path_ = *msg;
        has_path_ = !path_.poses.empty();

        // Convert the path to a vector of waypoints
        waypoints_.clear();
        waypoints_.reserve(path_.poses.size());
        //convert path to waypoints with id
        for (int i = 0; i < path_.poses.size(); ++i) {
            Waypoint wp;
            wp.id = i;
            wp.x = path_.poses[i].position.x;
            wp.y = path_.poses[i].position.y;
            wp.z = path_.poses[i].position.z;
            wp.qx = path_.poses[i].orientation.x;
            wp.qy = path_.poses[i].orientation.y;
            wp.qz = path_.poses[i].orientation.z;
            wp.qw = path_.poses[i].orientation.w;
            wp.yaw = tf::getYaw(path_.poses[i].orientation);
            wp.curvature = 0.0;
            wp.ref_speed = max_speed_;
            waypoints_.push_back(wp);
        }

    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber joy_sub_;
    ros::Subscriber path_sub_;
    ros::Publisher ackermann_pub_;
    ros::Publisher lookahead_marker_pub_;

    geometry_msgs::Pose current_pose_;
    double current_speed_{};
    geometry_msgs::PoseArray path_;

    bool has_odom_ = false;
    bool has_path_ = false;
    bool manual_mode_ = false;
    double manual_speed_ = 0.0;
    double manual_steering_ = 0.0;
    int button_lb_ = 4;     // LB button for manual mode
    int button_rb_ = 5;     // RB button for auto mode
    int axis_speed_ = 4;    // Vertical axis for speed
    int axis_steering_ = 3; // Horizontal axis for steering

    double lookahead_distance_{};
    double max_speed_{};
    double min_speed_{0.22}; // Minimum speed to prevent the vehicle from stopping
    double max_steering_angle_{};

    //path following
    std::vector<Waypoint> waypoints_ = {};

    double wheel_base_{0.36}; // Distance between front and rear axles of the vehicle
    std::string odometry_topic_;
    std::string path_topic_;

    void computeControl() {
        // Find the lookahead point
        geometry_msgs::Pose lookahead_point;
        if (!findLookaheadPoint(lookahead_point)) {
            ROS_WARN("No valid lookahead point found.");
            return;
        }

        // Publish the lookahead point marker
        publishLookaheadMarker(lookahead_point);

        // Transform lookahead point to the vehicle's coordinate frame
        double dx = lookahead_point.position.x - current_pose_.position.x;
        double dy = lookahead_point.position.y - current_pose_.position.y;

        tf::Quaternion q(
                current_pose_.orientation.x,
                current_pose_.orientation.y,
                current_pose_.orientation.z,
                current_pose_.orientation.w
        );
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        double transformed_x = dx * cos(-yaw) - dy * sin(-yaw);
        double transformed_y = dx * sin(-yaw) + dy * cos(-yaw);

        // Compute curvature
        if (transformed_x <= 0) {
            ROS_WARN("Lookahead point is behind the vehicle. Skipping control update.");
            return;
        }
        double curvature = 2.0 * transformed_y / (lookahead_distance_ * lookahead_distance_);

        // Compute steering angle , wheel_base_ = 0.36
        double steering_angle = atan(wheel_base_ * curvature);
        steering_angle = std::max(std::min(steering_angle, max_steering_angle_), -max_steering_angle_);

        // Scale velocity based on steering angle
        double normalized_steering = std::abs(steering_angle) / max_steering_angle_; // 0.0 to 1.0
        double speed = max_speed_ *
                       (1.0 - normalized_steering * (max_speed_ - min_speed_ / max_speed_)); // Scale between 0.22 and max_speed_

        // Ensure speed is not lower than the minimum value (e.g., 0.22)
        speed = std::max(speed, min_speed_);

        // Publish control command
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.drive.speed = speed;
        drive_msg.drive.steering_angle = steering_angle;

        ackermann_pub_.publish(drive_msg);
        has_odom_ = false;

        ROS_INFO("Published command: speed=%.2f, steering_angle=%.2f, curvature=%.2f",
                 drive_msg.drive.speed, drive_msg.drive.steering_angle, curvature);
    }

    void publishLookaheadMarker(const geometry_msgs::Pose &lookahead_point) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "pure_pursuit";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose = lookahead_point;
        marker.scale.x = 0.2; // Size of the marker
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 0.0; // Red color
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;

        lookahead_marker_pub_.publish(marker);
    }

    bool findLookaheadPoint(geometry_msgs::Pose &lookahead_point) {
        if (waypoints_.empty()) {
            ROS_WARN("No waypoints available.");
            return false;
        }

        // Find the closest waypoint
        Waypoint closest_wp = findClosestWaypoint();
        int closest_wp_id = closest_wp.id;

        // Validate the closest waypoint ID
        if (closest_wp_id < 0 || closest_wp_id >= waypoints_.size()) {
            ROS_WARN("Invalid closest waypoint ID.");
            return false;
        }

        // Search for a valid lookahead point starting from the closest waypoint ID
        for (int i = closest_wp_id; i < waypoints_.size(); ++i) {
            double dist = sqrt(pow(waypoints_[i].x - current_pose_.position.x, 2) +
                               pow(waypoints_[i].y - current_pose_.position.y, 2));

            if (dist >= lookahead_distance_) {
                // Found a valid lookahead point
                lookahead_point.position.x = waypoints_[i].x;
                lookahead_point.position.y = waypoints_[i].y;
                lookahead_point.position.z = waypoints_[i].z;
                lookahead_point.orientation.x = waypoints_[i].qx;
                lookahead_point.orientation.y = waypoints_[i].qy;
                lookahead_point.orientation.z = waypoints_[i].qz;
                lookahead_point.orientation.w = waypoints_[i].qw;
                return true;
            }
        }

        ROS_WARN("No valid lookahead point found starting from the closest waypoint.");
        return false;
    }


    //find the closest waypoint
    Waypoint findClosestWaypoint() {
        double min_dist = std::numeric_limits<double>::max();
        Waypoint closest_wp;
        for (const auto &wp : waypoints_) {
            double dist = sqrt(pow(wp.x - current_pose_.position.x, 2) + pow(wp.y - current_pose_.position.y, 2));
            if (dist < min_dist) {
                min_dist = dist;
                closest_wp = wp;
            }
        }
        return closest_wp;
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "pure_pursuit_controller");
    PurePursuitController controller;
    ros::spin();
    return 0;
}
