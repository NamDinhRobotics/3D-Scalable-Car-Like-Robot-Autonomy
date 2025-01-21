//
// Created by dinhnambkhn on 13/11/2024.
//
#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <chrono>
#include <thread>

int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_control_test_node");
    ros::NodeHandle nh;

    // Publisher to the ackermann_cmd topic
    ros::Publisher ackermann_cmd_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann_cmd", 10);

    // Define parameters for forward and backward motion
    constexpr double speed = 0.25;  // Set to a moderate speed for controlled movement
    constexpr double travel_time = 2.5;  // Approximate time to travel 0.5 meters at this speed
    constexpr double stop_duration = 3.0;  // Stop duration in seconds

    ackermann_msgs::AckermannDriveStamped drive_msg;

    // Step 1: Move forward
    drive_msg.header.stamp = ros::Time::now();
    drive_msg.drive.speed = speed;  // Forward speed
    drive_msg.drive.steering_angle = 0.0;  // Straight steering
    ros::Time start_time = ros::Time::now();

    ROS_INFO("Moving forward for 0.5 meters...");
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < travel_time) {
        ackermann_cmd_pub.publish(drive_msg);
        ros::Duration(0.1).sleep();  // Small sleep to control publish rate
    }

    // Step 2: Stop for 3 seconds
    drive_msg.drive.speed = 0.0;  // Set speed to zero to stop
    ROS_INFO("Stopping for 3 seconds...");
    start_time = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < stop_duration) {
        ackermann_cmd_pub.publish(drive_msg);
        ros::Duration(0.1).sleep();  // Maintain stop command
    }

    // Step 3: Move backward for 0.5 meters
    drive_msg.drive.speed = -speed;  // Backward speed
    ROS_INFO("Moving backward for 0.5 meters...");
    start_time = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < travel_time) {
        ackermann_cmd_pub.publish(drive_msg);
        ros::Duration(0.1).sleep();  // Small sleep to control publish rate
    }

    // Step 4: Stop after backward motion
    drive_msg.drive.speed = 0.0;
    ROS_INFO("Final stop.");
    ackermann_cmd_pub.publish(drive_msg);

    return 0;
}
