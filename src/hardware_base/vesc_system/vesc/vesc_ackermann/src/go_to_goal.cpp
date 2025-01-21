//
// Created by dinhnambkhn on 21/11/2024.
//
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <cmath>

class NavGoalFollower {
public:
    NavGoalFollower() {
        // Initialize parameters
        nh_.param("goal_tolerance", goal_tolerance_, 0.1); // Goal tolerance (m)
        nh_.param("max_speed", max_speed_, 0.5);      // Max linear speed (m/s)
        nh_.param("min_speed", min_speed_, 0.22);     // Minimum speed (m/s)
        nh_.param("max_steering_angle", max_steering_angle_, 0.5); // Max steering angle (radians)

        // Subscribers
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 20, &NavGoalFollower::goalCallback, this);
        odom_sub_ = nh_.subscribe("/odometry_livox", 20, &NavGoalFollower::odomCallback, this);

        // Publisher
        ackermann_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 20);

        goal_received_ = false;
        odom_received_ = false;

        ROS_INFO("NavGoalFollower (Ackermann with Speed Reduction) node initialized.");
    }

    void spin() {
        ros::Rate rate(20); // 10 Hz

        while (ros::ok()) {
            if (goal_received_ && odom_received_) {

                bool reverse = false;
                // Convert goal to vehicle's coordinate frame
                double goal_x_local, goal_y_local;
                transformToVehicleFrame(goal_x_, goal_y_, goal_x_local, goal_y_local);

                // Compute distance to the goal in the local frame
                const double distance = sqrt(goal_x_local * goal_x_local + goal_y_local * goal_y_local);

                // Determine forward or backward motion
                double speed = max_speed_;
                if (goal_x_local < 0) {
                    // If the goal is behind, move backward
                    speed = -max_speed_;
                    reverse = true;
                }

                // Reduce speed as the vehicle approaches the goal
                if (distance < 1.5) {  // Within 1 meter, reduce speed proportionally
                    speed = copysign(std::max(min_speed_, distance / 1.0 * max_speed_), speed);
                }

                // Compute steering angle
                double angle_to_goal = atan2(goal_y_local, goal_x_local);
                // If moving backward, steer in the opposite direction
                if (reverse) {
                    angle_to_goal = copysign(M_PI - fabs(angle_to_goal), angle_to_goal);
                }

                // Control logic
                if (distance > goal_tolerance_) {
                    ackermann_msgs::AckermannDriveStamped ackermann_cmd;

                    // Set speed , static_cast<float> is used to convert double to float
                    ackermann_cmd.drive.speed = static_cast<float>(speed);

                    // Proportional control for steering angle
                    ackermann_cmd.drive.steering_angle = std::max(std::min(angle_to_goal, max_steering_angle_), -max_steering_angle_);

                    // Publish Ackermann command
                    ackermann_pub_.publish(ackermann_cmd);
                } else {
                    ROS_INFO("Goal reached.");
                    goal_received_ = false;

                    // Stop the robot
                    ackermann_msgs::AckermannDriveStamped stop_cmd;
                    stop_cmd.drive.speed = 0.0;
                    stop_cmd.drive.steering_angle = 0.0;
                    ackermann_pub_.publish(stop_cmd);
                }
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber goal_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher ackermann_pub_;

    double goal_x_{}, goal_y_{};
    double current_x_{}, current_y_{}, current_yaw_{};
    double goal_tolerance_{};
    double max_speed_{}, min_speed_{}, max_steering_angle_{};
    bool goal_received_, odom_received_;

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;
        goal_received_ = true;
        ROS_INFO("Received new goal: x=%.2f, y=%.2f", goal_x_, goal_y_);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        // Extract yaw from quaternion
        const tf::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w
        );
        const tf::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);

        odom_received_ = true;
    }

    void transformToVehicleFrame(const double goal_x_global, const double goal_y_global, double &goal_x_local, double &goal_y_local) const {
        // Transform the goal coordinates from global to local vehicle frame
        const double dx = goal_x_global - current_x_;
        const double dy = goal_y_global - current_y_;

        // Rotation matrix for transforming to local frame
        goal_x_local = cos(-current_yaw_) * dx - sin(-current_yaw_) * dy;
        goal_y_local = sin(-current_yaw_) * dx + cos(-current_yaw_) * dy;

        ROS_INFO("Goal in local frame: x=%.2f, y=%.2f", goal_x_local, goal_y_local);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ackermann_nav_goal_with_speed_reduction");

    NavGoalFollower follower;
    follower.spin();

    return 0;
}
