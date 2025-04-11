#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudToOccupancyGrid {
public:
    PointCloudToOccupancyGrid() {
        // Subscribe to point cloud topic
        sub_cloud_ = nh_.subscribe("point_cloud", 1, &PointCloudToOccupancyGrid::cloudCallback, this);
        // Publish occupancy grid
        pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("custom_occupancy_map", 1);

        // Load parameters
        nh_.param("min_height", min_height_, -0.2);
        nh_.param("max_height", max_height_, 0.2);
        nh_.param("resolution", resolution_, 0.05);
        nh_.param("grid_width", grid_width_, 200);
        nh_.param("grid_height", grid_height_, 200);

        // Initialize the persistent occupancy grid
        grid_.header.frame_id = "map";
        grid_.info.resolution = resolution_;
        grid_.info.width = grid_width_;
        grid_.info.height = grid_height_;
        grid_.info.origin.position.x = -grid_width_ * resolution_ / 2.0;  // Center at (0,0)
        grid_.info.origin.position.y = -grid_height_ * resolution_ / 2.0;
        grid_.info.origin.position.z = 0.0;
        grid_.info.origin.orientation.w = 1.0;
        grid_.data.resize(grid_width_ * grid_height_, -1);  // -1 = unknown

        ROS_INFO("Node initialized. Subscribed to 'point_cloud', publishing to 'custom_occupancy_map'.");
        ROS_INFO("Grid size: %d x %d, resolution: %.2f, origin: (%.2f, %.2f)",
                 grid_width_, grid_height_, resolution_,
                 grid_.info.origin.position.x, grid_.info.origin.position.y);
    }

private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {

        // Update the grid header timestamp
        grid_.header.stamp = cloud_msg->header.stamp;

        // Convert ROS PointCloud2 to PCL
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);

        int points_processed = 0;
        int points_in_grid = 0;

        // Project 3D points to 2D grid and update occupancy
        for (const auto& point : cloud.points) {
            points_processed++;
            if (point.z < min_height_ || point.z > max_height_) {
                continue;  // Filter points outside height range
            }

            // Convert to grid coordinates
            int x_idx = static_cast<int>((point.x - grid_.info.origin.position.x) / resolution_);
            int y_idx = static_cast<int>((point.y - grid_.info.origin.position.y) / resolution_);

            // Check if point is within grid bounds
            if (x_idx >= 0 && x_idx < grid_width_ && y_idx >= 0 && y_idx < grid_height_) {
                int index = y_idx * grid_width_ + x_idx;
                if (grid_.data[index] == -1 || grid_.data[index] == 100) {
                    grid_.data[index] = 100;  // Mark as occupied
                    points_in_grid++;
                }
            }
        }


        // Publish the updated occupancy grid
        pub_.publish(grid_);
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_cloud_;
    ros::Publisher pub_;
    nav_msgs::OccupancyGrid grid_;  // Persistent grid
    double min_height_, max_height_;
    double resolution_;
    int grid_width_, grid_height_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_to_occupancy_grid_node");
    ROS_INFO("Starting pcl_to_occupancy_grid_node...");
    PointCloudToOccupancyGrid converter;
    ros::spin();
    return 0;
}
