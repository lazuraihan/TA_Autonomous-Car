#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

// The LaserScanToPointCloud class subscribes to the /scan topic and publishes to the /pointcloud topic.
class LaserScanToPointCloud {
public:
    LaserScanToPointCloud() {
        // Initialize ROS node
        nh_ = ros::NodeHandle();

        // Subscribe to the LaserScan topic
        scan_sub_ = nh_.subscribe("/scan", 1, &LaserScanToPointCloud::scanCallback, this);

        // Advertise the PointCloud topic
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pointcloud", 1);
    }

// The scanCallback function is called whenever a new LaserScan message is received.
// It uses the projectLaser method of the laser_geometry::LaserProjection class to convert the LaserScan message to a PointCloud message. //
private:
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
        // Convert LaserScan to PointCloud
        laser_geometry::LaserProjection projector;
        sensor_msgs::PointCloud2 cloud;
        projector.projectLaser(*scan_msg, cloud);

        // Publish the PointCloud
        cloud_pub_.publish(cloud);
    }

    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher cloud_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "laserscan_to_pointcloud");
    LaserScanToPointCloud converter;
    ros::spin();
    return 0;
}
