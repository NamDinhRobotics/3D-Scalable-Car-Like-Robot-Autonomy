#include <cmath>
#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>

using namespace std;

constexpr double PI = 3.1415926;

double sensorOffsetX = 0;
double sensorOffsetY = 0;
int pubSkipNum = 1;
int pubSkipCount = 0;
bool twoWayDrive = true;
double lookAheadDis = 0.5;
double yawRateGain = 7.5;
double stopYawRateGain = 7.5;
double maxYawRate = 45.0;
double maxSpeed = 1.0;
double maxAccel = 1.0;
double switchTimeThre = 1.0;
double dirDiffThre = 0.1;
double stopDisThre = 0.2;
double slowDwnDisThre = 1.0;
bool useInclRateToSlow = false;
double inclRateThre = 120.0;
double slowRate1 = 0.25;
double slowRate2 = 0.5;
double slowTime1 = 2.0;
double slowTime2 = 2.0;
bool useInclToStop = false;
double inclThre = 45.0;
double stopTime = 5.0;
bool noRotAtStop = false;
bool noRotAtGoal = true;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyYaw = 0;
int safetyStop = 0;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleXRec = 0;
float vehicleYRec = 0;
float vehicleZRec = 0;
float vehicleRollRec = 0;
float vehiclePitchRec = 0;
float vehicleYawRec = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

double odomTime = 0;
double joyTime = 0;
double slowInitTime = 0;
double stopInitTime = false;
int pathPointID = 0;
bool pathInit = false;
bool navFwd = true;
double switchTime = 0;

nav_msgs::Path path;

//odom_topic
string odom_topic;
//laser_topic
string lidar_topic;


double wheelbase = 0.36; // Wheelbase of the vehicle
ros::Publisher lookahead_marker_pub_;

void odomHandler(const nav_msgs::Odometry::ConstPtr &odomIn) {
    odomTime = odomIn->header.stamp.toSec();

    double roll, pitch, yaw;
    const geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

    vehicleRoll = static_cast<float>(roll);
    vehiclePitch = static_cast<float>(pitch);
    vehicleYaw = static_cast<float>(yaw);
    vehicleX = static_cast<float>(odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY);
    vehicleY = static_cast<float>(odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY);
    vehicleZ = static_cast<float>(odomIn->pose.pose.position.z);

    //If the absolute tilt exceeds the threshold, the vehicle is instructed to stop.
    //This is critical for safety in situations such as steep inclines or rough terrains.
    if ((fabs(roll) > inclThre * PI / 180.0 || fabs(pitch) > inclThre * PI / 180.0) && useInclToStop) {
        stopInitTime = odomIn->header.stamp.toSec();
    }
    //Monitors the vehicle's roll rate (angular velocity around the x-axis) and pitch rate (angular velocity around the y-axis).
    //If either exceeds the threshold(inclRateThre), it records the time(slowInitTime) when the condition was detected, indicating that the vehicle should slow down.
    if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 || fabs(odomIn->twist.twist.angular.y) >
         inclRateThre * PI / 180.0) && useInclRateToSlow) {
        slowInitTime = odomIn->header.stamp.toSec();
    }
}

void pathHandler(const nav_msgs::Path::ConstPtr &pathIn) {
    const int pathSize = static_cast<int>(pathIn->poses.size());
    path.poses.resize(pathSize);
    for (int i = 0; i < pathSize; i++) {
        path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
        path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
        path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
    }

    vehicleXRec = vehicleX;
    vehicleYRec = vehicleY;
    vehicleZRec = vehicleZ;
    vehicleRollRec = vehicleRoll;
    vehiclePitchRec = vehiclePitch;
    vehicleYawRec = vehicleYaw;

    pathPointID = 0;
    pathInit = true;
}

void joystickHandler(const sensor_msgs::Joy::ConstPtr &joy) {
    joyTime = ros::Time::now().toSec();

    joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
    joySpeed = joySpeedRaw;
    if (joySpeed > 1.0) joySpeed = 1.0;
    if (joy->axes[4] == 0) joySpeed = 0;
    joyYaw = joy->axes[3];
    if (joySpeed == 0 && noRotAtStop) joyYaw = 0;

    if (joy->axes[4] < 0 && !twoWayDrive) {
        joySpeed = 0;
        joyYaw = 0;
    }

    if (joy->axes[2] > -0.1) {
        autonomyMode = false;
    } else {
        autonomyMode = true;
    }
}

void speedHandler(const std_msgs::Float32::ConstPtr &speed) {
    const double speedTime = ros::Time::now().toSec();

    if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
        joySpeed = static_cast<float>(speed->data / maxSpeed);

        if (joySpeed < 0) joySpeed = 0;
        else if (joySpeed > 1.0) joySpeed = 1.0;
    }
}

void stopHandler(const std_msgs::Int8::ConstPtr &stop) {
    safetyStop = static_cast<unsigned char>(stop->data);
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
    marker.color.r = 1.0; // Red color
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;

    lookahead_marker_pub_.publish(marker);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "pathFollower");
    ros::NodeHandle nh;
    auto nhPrivate = ros::NodeHandle("~");

    nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
    nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
    nhPrivate.getParam("pubSkipNum", pubSkipNum);
    nhPrivate.getParam("twoWayDrive", twoWayDrive);
    nhPrivate.getParam("lookAheadDis", lookAheadDis);
    nhPrivate.getParam("yawRateGain", yawRateGain);
    nhPrivate.getParam("stopYawRateGain", stopYawRateGain);
    nhPrivate.getParam("maxYawRate", maxYawRate);
    nhPrivate.getParam("maxSpeed", maxSpeed);
    nhPrivate.getParam("maxAccel", maxAccel);
    nhPrivate.getParam("switchTimeThre", switchTimeThre);
    nhPrivate.getParam("dirDiffThre", dirDiffThre);
    nhPrivate.getParam("stopDisThre", stopDisThre);
    nhPrivate.getParam("slowDwnDisThre", slowDwnDisThre);
    nhPrivate.getParam("useInclRateToSlow", useInclRateToSlow);
    nhPrivate.getParam("inclRateThre", inclRateThre);
    nhPrivate.getParam("slowRate1", slowRate1);
    nhPrivate.getParam("slowRate2", slowRate2);
    nhPrivate.getParam("slowTime1", slowTime1);
    nhPrivate.getParam("slowTime2", slowTime2);
    nhPrivate.getParam("useInclToStop", useInclToStop);
    nhPrivate.getParam("inclThre", inclThre);
    nhPrivate.getParam("stopTime", stopTime);
    nhPrivate.getParam("noRotAtStop", noRotAtStop);
    nhPrivate.getParam("noRotAtGoal", noRotAtGoal);
    nhPrivate.getParam("autonomyMode", autonomyMode);
    nhPrivate.getParam("autonomySpeed", autonomySpeed);
    nhPrivate.getParam("joyToSpeedDelay", joyToSpeedDelay);

    //getPram odom topic with default value "/state_estimation"
    nhPrivate.param("odom_topic", odom_topic, std::string("/state_estimation"));
    nhPrivate.param("lidar_topic", lidar_topic, std::string("/registered_scan"));

    ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry>(odom_topic, 5, odomHandler);

    ros::Subscriber subPath = nh.subscribe<nav_msgs::Path>("/path", 5, pathHandler);

    ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy>("/joy", 5, joystickHandler);

    ros::Subscriber subSpeed = nh.subscribe<std_msgs::Float32>("/speed", 5, speedHandler);

    ros::Subscriber subStop = nh.subscribe<std_msgs::Int8>("/stop", 5, stopHandler);

    ros::Publisher pubSpeed = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 5);
    geometry_msgs::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = "vehicle";

    ros::Publisher ackermann_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 10);

    lookahead_marker_pub_ = nh.advertise<visualization_msgs::Marker>("lookahead_marker", 10);

    if (autonomyMode) {
        //static cast
        joySpeed = static_cast<float>(autonomySpeed / maxSpeed);

        if (joySpeed < 0) joySpeed = 0;
        else if (joySpeed > 1.0) joySpeed = 1.0;
    }

    ros::Rate rate(100);
    bool status = ros::ok();
    while (status) {
        ros::spinOnce();

        if (pathInit) {
            float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec)
                                + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
            float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec)
                                + cos(vehicleYawRec) * (vehicleY - vehicleYRec);

            auto pathSize = path.poses.size();
            auto endDisX = static_cast<float>(path.poses[pathSize - 1].pose.position.x - vehicleXRel);
            auto endDisY = static_cast<float>(path.poses[pathSize - 1].pose.position.y - vehicleYRel);
            auto endDis = sqrt(endDisX * endDisX + endDisY * endDisY);

            float disX, disY, dis;
            while (pathPointID < pathSize - 1) {
                disX = static_cast<float>(path.poses[pathPointID].pose.position.x - vehicleXRel);
                disY = static_cast<float>(path.poses[pathPointID].pose.position.y - vehicleYRel);
                dis = sqrt(disX * disX + disY * disY);
                if (dis < lookAheadDis) {
                    pathPointID++;
                } else {
                    break;
                }
            }

            // pub look ahead
            geometry_msgs::Pose lookahead_point;
            //convert point in vehicle frame to map frame
            lookahead_point.position.x = vehicleXRec + path.poses[pathPointID].pose.position.x * cos(vehicleYawRec)
                                         - path.poses[pathPointID].pose.position.y * sin(vehicleYawRec);
            lookahead_point.position.y = vehicleYRec + path.poses[pathPointID].pose.position.x * sin(vehicleYawRec)
                                         + path.poses[pathPointID].pose.position.y * cos(vehicleYawRec);
            //lookahead_point.position.x = vehicleXRec + path.poses[pathPointID].pose.position.x * cos(vehicleYawRec);
            //lookahead_point.position.y = vehicleYRec + path.poses[pathPointID].pose.position.x * sin(vehicleYawRec);

            publishLookaheadMarker(lookahead_point);

            disX = static_cast<float>(path.poses[pathPointID].pose.position.x - vehicleXRel);
            disY = static_cast<float>(path.poses[pathPointID].pose.position.y - vehicleYRel);
            dis = sqrt(disX * disX + disY * disY);
            float pathDir = atan2(disY, disX);

            float dirDiff = vehicleYaw - vehicleYawRec - pathDir;
            if (dirDiff > PI) dirDiff -= 2 * PI;
            else if (dirDiff < -PI) dirDiff += 2 * PI;
            if (dirDiff > PI) dirDiff -= 2 * PI;
            else if (dirDiff < -PI) dirDiff += 2 * PI;

            if (twoWayDrive) {
                double time = ros::Time::now().toSec();
                if (fabs(dirDiff) > PI / 2 && navFwd && time - switchTime > switchTimeThre) {
                    navFwd = false;
                    switchTime = time;
                } else if (fabs(dirDiff) < PI / 2 && !navFwd && time - switchTime > switchTimeThre) {
                    navFwd = true;
                    switchTime = time;
                }
            }

            auto joySpeed2 = static_cast<float>(maxSpeed * joySpeed);
            if (!navFwd) {
                dirDiff += PI;
                if (dirDiff > PI) dirDiff -= 2 * PI;
                joySpeed2 *= -1;
            }

            if (fabs(vehicleSpeed) < 2.0 * maxAccel / 100.0)
                vehicleYawRate = -static_cast<float>(stopYawRateGain * dirDiff);
            else vehicleYawRate = -static_cast<float>(yawRateGain * dirDiff);

            if (vehicleYawRate > maxYawRate * PI / 180.0) vehicleYawRate = static_cast<float>(maxYawRate * PI / 180.0);
            else if (vehicleYawRate < -maxYawRate * PI / 180.0)
                vehicleYawRate = -static_cast<float>(maxYawRate * PI / 180.0);

            if (joySpeed2 == 0 && !autonomyMode) {
                vehicleYawRate = static_cast<float>(maxYawRate * joyYaw * PI / 180.0);
            } else if (pathSize <= 1 || (dis < stopDisThre && noRotAtGoal)) {
                vehicleYawRate = 0;
            }

            if (pathSize <= 1) {
                joySpeed2 = 0;
            } else if (endDis / slowDwnDisThre < joySpeed) {
                joySpeed2 *= static_cast<float>(endDis / slowDwnDisThre);
            }

            float joySpeed3 = joySpeed2;
            if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0) joySpeed3 *= static_cast<float>(slowRate1);
            else if (odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0)
                joySpeed3 *= static_cast<float>(slowRate2);

            if (fabs(dirDiff) < dirDiffThre && dis > stopDisThre) {
                if (vehicleSpeed < joySpeed3) vehicleSpeed += static_cast<float>(maxAccel / 100.0);
                else if (vehicleSpeed > joySpeed3) vehicleSpeed -= static_cast<float>(maxAccel / 100.0);
            } else {
                if (vehicleSpeed > 0) vehicleSpeed -= static_cast<float>(maxAccel / 100.0);
                else if (vehicleSpeed < 0) vehicleSpeed += static_cast<float>(maxAccel / 100.0);
            }

            if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {
                vehicleSpeed = 0;
                vehicleYawRate = 0;
            }

            if (safetyStop >= 1) vehicleSpeed = 0;
            if (safetyStop >= 2) vehicleYawRate = 0;

            pubSkipCount--;
            if (pubSkipCount < 0) {
                cmd_vel.header.stamp = ros::Time().fromSec(odomTime);
                if (fabs(vehicleSpeed) <= maxAccel / 100.0) cmd_vel.twist.linear.x = 0;
                else cmd_vel.twist.linear.x = vehicleSpeed;
                cmd_vel.twist.angular.z = vehicleYawRate;
                pubSpeed.publish(cmd_vel);

                ackermann_msgs::AckermannDriveStamped ackermann_msg;
                ackermann_msg.header = cmd_vel.header; // Copy the header

                double linear_velocity = cmd_vel.twist.linear.x;
                double angular_velocity = cmd_vel.twist.angular.z;

                // Map linear velocity to speed
                ackermann_msg.drive.speed = 0.5; //linear_velocity;

                // Compute steering angle
                if (fabs(linear_velocity) > 1e-6) {
                    ackermann_msg.drive.steering_angle = atan(
                        static_cast<float>(angular_velocity * wheelbase / linear_velocity));
                } else {
                    ackermann_msg.drive.steering_angle = 0.0; // No steering angle if the vehicle is stationary
                }
                // Publish the Ackermann command
                ackermann_pub.publish(ackermann_msg);

                pubSkipCount = pubSkipNum;
            }
        }

        status = ros::ok();
        rate.sleep();
    }

    return 0;
}
