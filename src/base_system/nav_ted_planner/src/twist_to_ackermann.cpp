//
// Created by dinhnambkhn on 08/04/2025.
//
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <string>
#include <cmath>

class TwistToAckermann {
public:
    TwistToAckermann() {
        // Initialize ROS node
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        // Get parameters with defaults
        private_nh.param("wheelbase", wheelbase_, 0.36);
        private_nh.param("twist_cmd_topic", twist_cmd_topic_, std::string("/cmd_vel"));
        private_nh.param("ackermann_cmd_topic", ackermann_cmd_topic_, std::string("/ackermann_cmd"));
        private_nh.param("frame_id", frame_id_, std::string("odom"));

        // Set up publisher and subscriber
        pub_ = nh.advertise<ackermann_msgs::AckermannDriveStamped>(ackermann_cmd_topic_, 1);
        sub_ = nh.subscribe(twist_cmd_topic_, 1, &TwistToAckermann::cmdCallback, this);

        // Log startup info
        ROS_INFO("Starting conversion: wheelbase=%.2f, input=%s, output=%s",
                 wheelbase_, twist_cmd_topic_.c_str(), ackermann_cmd_topic_.c_str());
    }

private:
    double convertTransRotVelToSteeringAngle(double v, double omega) {
        if (omega == 0.0 || v == 0.0) {
            return 0.0;
        }
        double radius = v / omega;
        double steering = std::atan(wheelbase_ / radius);
        ROS_DEBUG("v=%.2f, omega=%.2f, radius=%.2f, steering=%.2f", v, omega, radius, steering);
        return steering;
    }

    void cmdCallback(const geometry_msgs::Twist::ConstPtr& data) {
        double v = data->linear.x;
        double omega = data->angular.z;
        double steering = convertTransRotVelToSteeringAngle(v, omega);

        // Create and populate AckermannDriveStamped message
        ackermann_msgs::AckermannDriveStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = frame_id_;
        msg.drive.steering_angle = steering;
        msg.drive.speed = v;

        // Publish the message
        pub_.publish(msg);
        ROS_INFO("Published: speed=%.2f, steering=%.2f", v, steering);
    }

    ros::Publisher pub_;
    ros::Subscriber sub_;
    double wheelbase_;
    std::string twist_cmd_topic_;
    std::string ackermann_cmd_topic_;
    std::string frame_id_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "twist_to_ackermann");
    TwistToAckermann converter;
    ros::spin();
    return 0;
}