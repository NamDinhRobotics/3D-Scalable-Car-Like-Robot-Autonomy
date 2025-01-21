//
// Created by dinhnambkhn on 17/12/2024.
//
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

// Global publisher for Ackermann commands
ros::Publisher ackermann_pub_;

// Joystick axes and button mappings
int axis_linear = 4;   // Joystick axis for linear speed (e.g., vertical axis)
int axis_steering = 3; // Joystick axis for steering (e.g., horizontal axis)
double max_speed = 3.0;       // Max linear speed [m/s]
double max_steering_angle = 0.5; // Max steering angle [radians]

// Joystick handler callback function
void joystickHandler(const sensor_msgs::Joy::ConstPtr& joy)
{
    // Create AckermannDriveStamped message
    ackermann_msgs::AckermannDriveStamped ackermann_msg;

    //ROS INFO joy->axes[axis_linear] and joy->axes[axis_steering]
    ROS_INFO("joy->axes[axis_linear]: %f", joy->axes[axis_linear]);
    ROS_INFO("joy->axes[axis_steering]: %f", joy->axes[axis_steering]);

    // Read axes for speed and steering
    double speed = max_speed * joy->axes[axis_linear]; // Scale speed
    double steering_angle = max_steering_angle * joy->axes[axis_steering]; // Scale steering

    // Populate AckermannDriveStamped message
    ackermann_msg.header.stamp = ros::Time::now();
    ackermann_msg.drive.speed = speed;
    ackermann_msg.drive.steering_angle = steering_angle;

    // Publish the Ackermann message
    ackermann_pub_.publish(ackermann_msg);

    // Log output
    ROS_INFO("Publishing Ackermann Command: Speed=%.2f, Steering Angle=%.2f",
             speed, steering_angle);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "manual_control_node");
    ros::NodeHandle nh;

    // Initialize subscriber to joystick input
    ros::Subscriber subJoystick =
            nh.subscribe<sensor_msgs::Joy>("/joy", 5, joystickHandler);

    // Initialize publisher for Ackermann commands
    ackermann_pub_ = nh.advertise<ackermann_msgs::AckermannDriveStamped>(
            "/ackermann_cmd", 20);

    ROS_INFO("Manual Control Node Initialized. Waiting for joystick input...");

    ros::spin(); // Keep the node running
    return 0;
}
