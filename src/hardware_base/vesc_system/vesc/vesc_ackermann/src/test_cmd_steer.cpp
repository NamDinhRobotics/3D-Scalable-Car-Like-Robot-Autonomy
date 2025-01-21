#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <chrono>

int main(int argc, char** argv) {
    ros::init(argc, argv, "ackermann_cmd_test_node");
    ros::NodeHandle nh;

    // Publisher to the ackermann_cmd topic
    const ros::Publisher ackermann_cmd_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann_cmd", 10);

    // Set the loop rate for publishing messages
    ros::Rate loop_rate(2); // 2 Hz for a steady test signal

    // Initial speed and steering angle values
    float speed = 0.25;
    float steering_angle = 0.5236; // 30 degrees in radians

    // Timer to track the 10-second intervals
    auto start_time = std::chrono::steady_clock::now();

    ROS_INFO("Starting to publish test AckermannDriveStamped messages with alternating speed and steering angle...");

    while (ros::ok()) {
        ackermann_msgs::AckermannDriveStamped test_msg;
        test_msg.header.stamp = ros::Time::now(); // Update the timestamp
        test_msg.drive.speed = speed;
        test_msg.drive.steering_angle = steering_angle;

        // Publish the test message
        ackermann_cmd_pub.publish(test_msg);
        ROS_INFO("Published test AckermannDriveStamped message: speed = %f, steering_angle = %f (radians)", test_msg.drive.speed, test_msg.drive.steering_angle);

        // Check if 10 seconds have passed to switch directions
        auto now = std::chrono::steady_clock::now();
        const auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
        if (elapsed_seconds >= 2) {
            // Reverse the speed and steering angle
            speed = -speed;
            steering_angle = -steering_angle;

            // Reset the timer
            start_time = std::chrono::steady_clock::now();
            ROS_INFO("Switching direction: speed = %f, steering_angle = %f (radians)", speed, steering_angle);
        }

        // Sleep to maintain the loop rate
        loop_rate.sleep();
    }

    return 0;
}
