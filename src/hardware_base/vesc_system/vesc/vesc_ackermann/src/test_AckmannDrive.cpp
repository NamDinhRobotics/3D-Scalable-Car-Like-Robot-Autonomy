#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <chrono>

int main(int argc, char** argv) {
    ros::init(argc, argv, "ackermann_cmd_test_node");
    ros::NodeHandle nh;

    // Publisher to the ackermann_cmd topic
    ros::Publisher ackermann_cmd_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann_cmd", 10);

    // Set the loop rate for publishing messages
    ros::Rate loop_rate(20); // 2 Hz for smooth transitions

    // Initialize variables for speed and steering adjustment
    float speed = 0.0;
    float steering_angle = 0.0;
    bool increasing_speed = true;
    bool increasing_steering = true;
    bool holding = false;
    auto hold_start_time = std::chrono::steady_clock::now();

    ROS_INFO("Starting to publish test AckermannDriveStamped messages with varying speeds and steering angles...");

    while (ros::ok()) {
        ackermann_msgs::AckermannDriveStamped test_msg;
        test_msg.header.stamp = ros::Time::now(); // Update the timestamp
        test_msg.drive.speed = speed;
        test_msg.drive.steering_angle = steering_angle;

        // Publish the test message
        ackermann_cmd_pub.publish(test_msg);
        ROS_INFO("Published test AckermannDriveStamped message: speed = %f, steering_angle = %f", test_msg.drive.speed, test_msg.drive.steering_angle);

        // Check if we need to start or continue holding the speed and steering angle
        if (holding) {
            auto now = std::chrono::steady_clock::now();
            const auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(now - hold_start_time).count();

            // Continue holding until 30 seconds have passed
            if (elapsed_seconds >= 2) {
                holding = false;
                increasing_speed = !increasing_speed;  // Switch speed direction after hold
                increasing_steering = !increasing_steering;  // Switch steering direction after hold
            }
        } else {
            // Adjust speed if not holding
            if (increasing_speed) {
                speed += 0.01;
                if (speed >= 1.0) {  // Reached upper speed limit, start holding
                    holding = true;
                    hold_start_time = std::chrono::steady_clock::now();
                }
            } else {
                speed -= 0.01;
                if (speed <= -1.0) {  // Reached lower speed limit, start holding
                    holding = true;
                    hold_start_time = std::chrono::steady_clock::now();
                }
            }

            // Adjust steering angle in a similar pattern
            if (increasing_steering) {
                steering_angle += 0.01;  // Increase steering angle by 0.05 radians
                if (steering_angle >= 0.5) {  // Reached upper steering limit
                    increasing_steering = false;
                }
            } else {
                steering_angle -= 0.01;  // Decrease steering angle by 0.05 radians
                if (steering_angle <= -0.5) {  // Reached lower steering limit
                    increasing_steering = true;
                }
            }
        }

        // Sleep to maintain the loop rate
        loop_rate.sleep();
    }

    return 0;
}
