#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // Still needed for other tf2 utilities
#include <Eigen/Geometry> // For Eigen types
#include <math.h>

class PointCloudToLaserScan {
public:
    PointCloudToLaserScan() : tf_listener_(tf_buffer_) {
        sub_cloud_ = nh_.subscribe("point_cloud_topic", 1, &PointCloudToLaserScan::cloudCallback, this);
        sub_odom_ = nh_.subscribe("odom_topic", 1, &PointCloudToLaserScan::odomCallback, this);
        pub_ = nh_.advertise<sensor_msgs::LaserScan>("laser_scan_topic", 1);
        pub_transformed_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("transformed_cloud_topic", 1);

        nh_.param("base_frame", base_frame_, std::string("base_link"));
        nh_.param("min_height", min_height_, -0.1);
        nh_.param("max_height", max_height_, 0.1);
        nh_.param("angle_min", angle_min_, -M_PI);
        nh_.param("angle_max", angle_max_, M_PI);
        nh_.param("angle_increment", angle_increment_, 0.01);
        nh_.param("range_min", range_min_, 0.0);
        nh_.param("range_max", range_max_, 30.0);
    }

private:
    void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg) {
        latest_odom_ = *odom_msg;
        odom_received_ = true;
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        if (!odom_received_) {
            ROS_WARN("Odometry not received yet, skipping point cloud processing.");
            return;
        }

        // Convert ROS PointCloud2 to PCL
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);

        // Transform point cloud to base_frame
        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        try {
            geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
                    base_frame_, cloud_msg->header.frame_id, cloud_msg->header.stamp, ros::Duration(1.0));

            // Manually convert TransformStamped to Eigen::Affine3f
            const auto& t = transform.transform.translation;
            const auto& q = transform.transform.rotation;

            Eigen::Translation3f translation(t.x, t.y, t.z);
            Eigen::Quaternionf rotation(q.w, q.x, q.y, q.z);
            Eigen::Affine3f eigen_transform = translation * rotation;

            // Apply transformation to point cloud
            pcl::transformPointCloud(cloud, transformed_cloud, eigen_transform);
        } catch (tf2::TransformException& ex) {
            ROS_WARN("Transform failed: %s", ex.what());
            return;
        }

        //publish transformed point cloud for debugging
        sensor_msgs::PointCloud2 transformed_cloud_msg;
        pcl::toROSMsg(transformed_cloud, transformed_cloud_msg);
        transformed_cloud_msg.header = cloud_msg->header;
        transformed_cloud_msg.header.frame_id = base_frame_;
        transformed_cloud_msg.header.stamp = ros::Time::now();
        pub_transformed_cloud_.publish(transformed_cloud_msg);

        // Initialize LaserScan message
        sensor_msgs::LaserScan scan;
        scan.header = cloud_msg->header;
        scan.header.frame_id = base_frame_;
        scan.angle_min = angle_min_;
        scan.angle_max = angle_max_;
        scan.angle_increment = angle_increment_;
        scan.range_min = range_min_;
        scan.range_max = range_max_;
        scan.ranges.resize((angle_max_ - angle_min_) / angle_increment_, std::numeric_limits<float>::infinity());

        // Project transformed 3D points to 2D plane
        for (const auto& point : transformed_cloud.points) {
            if (point.z < min_height_ || point.z > max_height_) {
                continue;
            }

            float range = sqrt(point.x * point.x + point.y * point.y);
            if (range < range_min_ || range > range_max_) {
                continue;
            }

            float angle = atan2(point.y, point.x);
            if (angle < angle_min_ || angle > angle_max_) {
                continue;
            }

            int index = (angle - angle_min_) / angle_increment_;
            if (index >= 0 && index < scan.ranges.size()) {
                scan.ranges[index] = std::min(scan.ranges[index], range);
            }
        }
        odom_received_ = false; // Reset odom_received_ after processing
        pub_.publish(scan);
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_cloud_, sub_odom_;
    ros::Publisher pub_, pub_transformed_cloud_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    nav_msgs::Odometry latest_odom_;
    bool odom_received_ = false;
    std::string base_frame_;
    double min_height_, max_height_;
    double angle_min_, angle_max_, angle_increment_;
    double range_min_, range_max_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_to_laserscan_node");
    PointCloudToLaserScan converter;
    ros::spin();
    return 0;
}