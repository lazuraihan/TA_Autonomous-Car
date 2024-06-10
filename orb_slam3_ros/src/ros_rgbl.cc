/**
* 
* Adapted from TUMFTM_ORB-SLAM3_RGBL: Examples/RGB-L/rgbl_kitti.cc
*
*/

#include "common.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber() : new_image(false), new_pointcloud(false) {}

    void GrabRGBL(const sensor_msgs::ImageConstPtr& msgRGBL);
    void LoadPointcloudFromROSMsg(const sensor_msgs::PointCloud2ConstPtr& msg);

    cv::Mat image;
    cv::Mat point_cloud;
    ros::Time image_timestamp;
    bool new_image;
    bool new_pointcloud;

    ORB_SLAM3::System* pSLAM;
    void Process();
};

// Program utama 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBL");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

    std::string node_name = ros::this_node::getName();
    ros::NodeHandle node_handler;
    image_transport::ImageTransport image_transport(node_handler);

    std::string voc_file, settings_file;
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");       
        ros::shutdown();
        return 1;
    }

    node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "map");
    node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");

    bool enable_pangolin;
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, true);

    ORB_SLAM3::System* pSLAM = new ORB_SLAM3::System(voc_file, settings_file, ORB_SLAM3::System::RGBL, enable_pangolin);

    ImageGrabber igb;
    igb.pSLAM = pSLAM;

    ros::Subscriber sub_img = node_handler.subscribe("/camera/color/image_raw", 100, &ImageGrabber::GrabRGBL, &igb);
    ros::Subscriber sub_pointcloud = node_handler.subscribe("/pointcloud", 100, &ImageGrabber::LoadPointcloudFromROSMsg, &igb);

    ros::spin();

    pSLAM->Shutdown();
    ros::shutdown();

    delete pSLAM;
}

void ImageGrabber::GrabRGBL(const sensor_msgs::ImageConstPtr& msgRGBL)
{
    cv_bridge::CvImageConstPtr cv_ptrRGBL;
    try
    {
        cv_ptrRGBL = cv_bridge::toCvShare(msgRGBL);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Store the image and timestamp
    image = cv_ptrRGBL->image.clone();
    image_timestamp = cv_ptrRGBL->header.stamp;
    new_image = true;

    // Process if both image and point cloud are received
    if (new_image && new_pointcloud)
    {
        Process();
    }
}

void ImageGrabber::LoadPointcloudFromROSMsg(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    int num = msg->width * msg->height;
    point_cloud = cv::Mat::zeros(cv::Size(num, 4), CV_32F);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    bool has_intensity = false;
    for (const auto& field : msg->fields) {
        if (field.name == "intensity") {
            has_intensity = true;
            break;
        }
    }

    sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");

    for (int i = 0; i < num; ++i, ++iter_x, ++iter_y, ++iter_z) {
        point_cloud.at<float>(0, i) = *iter_x;
        point_cloud.at<float>(1, i) = *iter_y;
        point_cloud.at<float>(2, i) = *iter_z;
        point_cloud.at<float>(3, i) = has_intensity ? *iter_intensity : 1.0f;
        if (has_intensity) {
            ++iter_intensity;
        }
    }

    new_pointcloud = true;

    if (new_image && new_pointcloud)
    {
        Process();
    }
}


void ImageGrabber::Process()
{
    // Ensure both data are processed only once
    new_image = false;
    new_pointcloud = false;

    // // Pass the image and pointcloud to the SLAM system
    Sophus::SE3f Tcw = pSLAM->TrackRGBL(image, point_cloud, image_timestamp.toSec());

    // Additional processing if needed...
}

