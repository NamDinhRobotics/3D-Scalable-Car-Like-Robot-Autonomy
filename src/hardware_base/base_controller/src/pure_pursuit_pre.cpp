#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <cmath>

class PurePursuitController {
public:
    PurePursuitController() {
        // Load parameters
        ros::NodeHandle nh_private("~");
        nh_private.param("lookahead_distance", lookahead_distance_, 2.0);
        nh_private.param("max_speed", max_speed_, 2.0);
        nh_private.param("min_speed", min_speed_, 0.22);
        nh_private.param("max_steering_angle", max_steering_angle_, 0.5);

        // Subscribers
        odom_sub_ = nh_.subscribe("odom", 10, &PurePursuitController::odomCallback, this);
        path_sub_ = nh_.subscribe("loaded_path", 10, &PurePursuitController::pathCallback, this);

        // Publishers
        ackermann_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 20);
        lookahead_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("lookahead_marker", 10);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        current_pose_ = msg->pose.pose;
        current_speed_ = sqrt(msg->twist.twist.linear.x * msg->twist.twist.linear.x +
                              msg->twist.twist.linear.y * msg->twist.twist.linear.y);
        has_odom_ = true;

        // Compute and publish control commands if path is available
        if (has_path_ && has_odom_) {
            computeControl();
        }
    }

    void pathCallback(const geometry_msgs::PoseArray::ConstPtr &msg) {
        path_ = *msg;
        has_path_ = !path_.poses.empty();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber path_sub_;
    ros::Publisher ackermann_pub_;
    ros::Publisher lookahead_marker_pub_;

    geometry_msgs::Pose current_pose_;
    double current_speed_{};
    geometry_msgs::PoseArray path_;

    bool has_odom_ = false;
    bool has_path_ = false;

    double lookahead_distance_{};
    double max_speed_{};
    double min_speed_{0.22}; // Minimum speed to prevent the vehicle from stopping
    double max_steering_angle_{};

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

        // Compute steering angle
        double steering_angle = atan(curvature);
        steering_angle = std::max(std::min(steering_angle, max_steering_angle_), -max_steering_angle_);

        // Scale velocity based on steering angle
        double normalized_steering = std::abs(steering_angle) / max_steering_angle_; // 0.0 to 1.0
        double speed = max_speed_ *
                       (1.0 - normalized_steering * (1.0 - 0.22 / max_speed_)); // Scale between 0.22 and max_speed_

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
        static geometry_msgs::Pose previous_lookahead_point; // Store the previous lookahead point
        static bool has_previous_point = false;             // Check if a previous point exists

        for (const auto &pose: path_.poses) {
            double dx = pose.position.x - current_pose_.position.x;
            double dy = pose.position.y - current_pose_.position.y;

            tf::Quaternion q(
                    current_pose_.orientation.x,
                    current_pose_.orientation.y,
                    current_pose_.orientation.z,
                    current_pose_.orientation.w
            );
            double roll, pitch, yaw;
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

            // Transform to vehicle frame
            double transformed_x = dx * cos(-yaw) - dy * sin(-yaw);

            // Check if the point is ahead and satisfies the lookahead distance
            double distance = sqrt(dx * dx + dy * dy);

            // Adjust lookahead distance based on current speed
            double ratio = current_speed_ / max_speed_;
            double lookahead_distance = lookahead_distance_ * (1.0 + 0.5 * ratio);

            if (transformed_x > 0 && distance >= lookahead_distance && distance <= 3 * lookahead_distance) {
                if (has_previous_point) {
                    // Smoothly interpolate between the previous point and the current candidate
                    double alpha = 0.3; // Smoothing factor (0.0: keep previous point, 1.0: accept new point)
                    lookahead_point.position.x =
                            alpha * pose.position.x + (1 - alpha) * previous_lookahead_point.position.x;
                    lookahead_point.position.y =
                            alpha * pose.position.y + (1 - alpha) * previous_lookahead_point.position.y;
                    lookahead_point.position.z = pose.position.z; // No need to smooth z (assume 2D)

                    lookahead_point.orientation = pose.orientation; // Copy orientation directly
                } else {
                    // First valid lookahead point, no smoothing needed
                    lookahead_point = pose;
                    has_previous_point = true;
                }

                previous_lookahead_point = lookahead_point; // Update the previous lookahead point
                return true;
            }
        }

        has_previous_point = false; // Reset if no valid point is found
        return false; // No valid lookahead point found
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "pure_pursuit_controller");
    PurePursuitController controller;
    ros::spin();
    return 0;
}
