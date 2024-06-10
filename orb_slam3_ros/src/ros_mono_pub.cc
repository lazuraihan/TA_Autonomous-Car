#include <unistd.h>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <time.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include<opencv2/core/core.hpp>
// #include "opencv2/imgcodecs\legacy\constants_c.h"

//#include"../../../include/System.h"
#include "System.h"

#include "MapPoint.h"
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <Converter.h>

//#include "common.h"

//! parameters

bool read_from_topic = false, read_from_camera = false;
std::string image_topic = "/camera/color/image_raw";
int all_pts_pub_gap = 0;

vector<string> vstrImageFilenames;
vector<double> vTimestamps;
cv::VideoCapture cap_obj;

bool pub_all_pts = false;
int pub_count = 0;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
	vector<double> &vTimestamps);
inline bool isInteger(const std::string & s);
void publish(ORB_SLAM3::System &SLAM, ros::Publisher &pub_pts_and_pose,
	ros::Publisher &pub_all_kf_and_pts, int frame_id);

class ImageGrabber{
public:
	ImageGrabber(ORB_SLAM3::System &_SLAM, ros::Publisher &_pub_pts_and_pose,
		ros::Publisher &_pub_all_kf_and_pts) :
		SLAM(_SLAM), pub_pts_and_pose(_pub_pts_and_pose),
		pub_all_kf_and_pts(_pub_all_kf_and_pts), frame_id(0){}

	void GrabImage(const sensor_msgs::ImageConstPtr& msg);

	ORB_SLAM3::System &SLAM;
	ros::Publisher &pub_pts_and_pose;
	ros::Publisher &pub_all_kf_and_pts;
	int frame_id;
};
bool parseParams(int argc, char **argv);

using namespace std;

int main(int argc, char **argv){
	ros::init(argc, argv, "Monopub");
	ros::start();
	if (!parseParams(argc, argv)) {
		return EXIT_FAILURE;
	}
	int n_images = vstrImageFilenames.size();

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);
	ros::NodeHandle nodeHandler;																										
	//ros::Publisher pub_cloud = nodeHandler.advertise<sensor_msgs::PointCloud2>("cloud_in", 1000);
	ros::Publisher pub_pts_and_pose = nodeHandler.advertise<geometry_msgs::PoseArray>("pts_and_pose", 1000);
	ros::Publisher pub_all_kf_and_pts = nodeHandler.advertise<geometry_msgs::PoseArray>("all_kf_and_pts", 1000);
	if (read_from_topic) {
		ImageGrabber igb(SLAM, pub_pts_and_pose, pub_all_kf_and_pts);
		ros::Subscriber sub = nodeHandler.subscribe(image_topic, 1, &ImageGrabber::GrabImage, &igb);
		ros::spin();
	}
	else{
		ros::Rate loop_rate(5);
		cv::Mat im;
		double tframe = 0;
#ifdef COMPILEDWITHC11
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
		std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
		for (int frame_id = 0; read_from_camera || frame_id < n_images; ++frame_id){
			if (read_from_camera) {
				cap_obj.read(im);
#ifdef COMPILEDWITHC11
				std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
				std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
				tframe = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
				//printf("fps: %f\n", 1.0 / tframe);
			}
			else {
				// Read image from file
				im = cv::imread(vstrImageFilenames[frame_id], cv::IMREAD_UNCHANGED);
				tframe = vTimestamps[frame_id];
			}
			if (im.empty()){
				cerr << endl << "Failed to load image at: " << vstrImageFilenames[frame_id] << endl;
				return 1;
			}
			// Pass the image to the SLAM system
			//cv::Mat curr_pose = SLAM.TrackMonocular(im, tframe);
			Sophus::SE3f curr_pose = SLAM.TrackMonocular(im, tframe);

			publish(SLAM, pub_pts_and_pose, pub_all_kf_and_pts, frame_id);

			//cv::imshow("Press escape to exit", im);
			//if (cv::waitKey(1) == 27) {
			//	break;
			//}
			ros::spinOnce();
			loop_rate.sleep();
			if (!ros::ok()){ break; }
		}
	}
	//ros::spin();

	mkdir("results", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	SLAM.getMap()->GetCurrentMap()->Save("results//map_pts_out.obj");
	SLAM.getMap()->GetCurrentMap()->SaveWithTimestamps("results//map_pts_and_keyframes.txt");
	// Save camera trajectory
	SLAM.SaveKeyFrameTrajectoryTUM("results//key_frame_trajectory.txt");


	// Stop all threads
	SLAM.Shutdown();
	//geometry_msgs::PoseArray pt_array;
	//pt_array.header.seq = 0;
	//pub_pts_and_pose.publish(pt_array);
	ros::shutdown();
	return 0;
}

void publish(ORB_SLAM3::System &SLAM, ros::Publisher &pub_pts_and_pose,
	ros::Publisher &pub_all_kf_and_pts, int frame_id) {
	ROS_INFO("Publish function called");
	if (all_pts_pub_gap>0 && pub_count >= all_pts_pub_gap) {
		pub_all_pts = true;
		pub_count = 0;
	}
	if (pub_all_pts || SLAM.getLoopClosing()->loop_detected || SLAM.getTracker()->loop_detected) {
		pub_all_pts = SLAM.getTracker()->loop_detected = SLAM.getLoopClosing()->loop_detected = false;
		geometry_msgs::PoseArray kf_pt_array;
		vector<ORB_SLAM3::KeyFrame*> key_frames = SLAM.getMap()->GetCurrentMap()->GetAllKeyFrames();

		 if (key_frames.empty()) {
            ROS_WARN("No keyframes available");
        }
		//! placeholder for number of keyframes
		kf_pt_array.poses.push_back(geometry_msgs::Pose());
		sort(key_frames.begin(), key_frames.end(), ORB_SLAM3::KeyFrame::lId);
		unsigned int n_kf = 0;
		for (auto key_frame : key_frames) {
			// pKF->SetPose(pKF->GetPose()*Two);

			if (key_frame->isBad())
				continue;

			Eigen::Matrix3f R = key_frame->GetRotation();
			Eigen::Matrix3f R_transpose = R.transpose();
			vector<float> q = ORB_SLAM3::Converter::toQuaternionE(R_transpose);
			Eigen::Vector3f twc = key_frame->GetCameraCenter();
			geometry_msgs::Pose kf_pose;

			kf_pose.position.x = twc.x();
			kf_pose.position.y = twc.y();
			kf_pose.position.z = twc.z();
			kf_pose.orientation.x = q[0];
			kf_pose.orientation.y = q[1];
			kf_pose.orientation.z = q[2];
			kf_pose.orientation.w = q[3];
			kf_pt_array.poses.push_back(kf_pose);

			unsigned int n_pts_id = kf_pt_array.poses.size();
			//! placeholder for number of points
			kf_pt_array.poses.push_back(geometry_msgs::Pose());
			std::set<ORB_SLAM3::MapPoint*> map_points = key_frame->GetMapPoints();
			unsigned int n_pts = 0;
			for (auto map_pt : map_points) {
				if (!map_pt || map_pt->isBad()) {
					//printf("Point %d is bad\n", pt_id);
					continue;
				}
				Eigen::Vector3f pt_pose = map_pt->GetWorldPos();
				if (!pt_pose.allFinite()) {
    				continue;

					}
				geometry_msgs::Pose curr_pt;
				//printf("wp size: %d, %d\n", wp.rows, wp.cols);
				//pcl_cloud->push_back(pcl::PointXYZ(wp.at<float>(0), wp.at<float>(1), wp.at<float>(2)));
				curr_pt.position.x = pt_pose.x();
				curr_pt.position.y = pt_pose.y();
				curr_pt.position.z = pt_pose.z();
				kf_pt_array.poses.push_back(curr_pt);
				++n_pts;
			}
			geometry_msgs::Pose n_pts_msg;
			n_pts_msg.position.x = n_pts_msg.position.y = n_pts_msg.position.z = n_pts;
			kf_pt_array.poses[n_pts_id] = n_pts_msg;
			++n_kf;
		}
		geometry_msgs::Pose n_kf_msg;
		n_kf_msg.position.x = n_kf_msg.position.y = n_kf_msg.position.z = n_kf;
		kf_pt_array.poses[0] = n_kf_msg;
		kf_pt_array.header.frame_id = "1";
		kf_pt_array.header.seq = frame_id + 1;
		printf("Publishing data for %u keyframes\n", n_kf);
		pub_all_kf_and_pts.publish(kf_pt_array);
	}
	else if (SLAM.getTracker()->mCurrentFrame.is_keyframe) {
		++pub_count;
		SLAM.getTracker()->mCurrentFrame.is_keyframe = false;
		ORB_SLAM3::KeyFrame* pKF = SLAM.getTracker()->mCurrentFrame.mpReferenceKF;

		cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);
		Sophus::SE3<float> Trw_r = ORB_SLAM3::Converter::ToSophusSE3f(Trw);
		//Eigen::Matrix<float,4,4> eigenTrw = ORB_SLAM3::Converter::cvMatToEigen(Trw);
		//Sophus::SE3f Trw_r(Eigen::Matrix<float,4,4>(eigenTrw));

		// Eigen::Matrix<float,4,4> Trw_r = eigenTrw.block<3, 3>(0, 0);
		// Sophus::SE3f Trw_R = Sophus::SE3f::rotatioin_matrix(Trw_r);

		// If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
		//while (pKF->isBad())
		//{
		//	Trw = Trw*pKF->mTcp;
		//	pKF = pKF->GetParent();
		//}

		vector<ORB_SLAM3::KeyFrame*> vpKFs = SLAM.getMap()->GetCurrentMap()->GetAllKeyFrames();
		sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM3::KeyFrame::lId);

		// Transform all keyframes so that the first keyframe is at the origin.
		// After a loop closure the first keyframe might not be at the origin.
		Sophus::SE3f Two = vpKFs[0]->GetPoseInverse(); // 4x4 matrix

		Trw_r = Trw_r*pKF->GetPose()*Two; // 4x4 matrix

		//Eigen::Matrix4f eigenMat = Trw.matrix();
    	//cv::Mat Trw_mat(4, 4, CV_32F);
    	//	for (int i = 0; i < 4; ++i)
        //		for (int j = 0; j < 4; ++j)
        //    		Trw_mat.at<float>(i, j) = eigenMat(i, j);

		Sophus::SE3f lit = SLAM.getTracker()->mlRelativeFramePoses.back(); // 4x4 
		Sophus::SE3f Tcw = lit*Trw_r;										// 4x4
		Eigen::Matrix3f Rwc = Tcw.rotationMatrix().transpose();				// 3x3
		Eigen::Vector3f twc = -Rwc * Tcw.translation();

		vector<float> q = ORB_SLAM3::Converter::toQuaternionE(Rwc);
		//geometry_msgs::Pose camera_pose;
		//std::vector<ORB_SLAM3::MapPoint*> map_points = SLAM.getMap()->GetCurrentMap()->GetAllMapPoints();
		std::vector<ORB_SLAM3::MapPoint*> map_points = SLAM.GetTrackedMapPoints();
		int n_map_pts = map_points.size();

		//printf("n_map_pts: %d\n", n_map_pts);

		//pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

		geometry_msgs::PoseArray pt_array;
		//pt_array.poses.resize(n_map_pts + 1);

		geometry_msgs::Pose camera_pose;

		camera_pose.position.x = twc.x();
		camera_pose.position.y = twc.y();
		camera_pose.position.z = twc.z();

		camera_pose.orientation.x = q[0];
		camera_pose.orientation.y = q[1];
		camera_pose.orientation.z = q[2];
		camera_pose.orientation.w = q[3];

		pt_array.poses.push_back(camera_pose);

		//printf("Done getting camera pose\n");

		for (int pt_id = 1; pt_id <= n_map_pts; ++pt_id){

			if (!map_points[pt_id - 1] || map_points[pt_id - 1]->isBad()) {
				//printf("Point %d is bad\n", pt_id);
				continue;
			}
			Eigen::Vector3f wp = map_points[pt_id - 1]->GetWorldPos();

			if (!wp.allFinite()) {
				//printf("World position for point %d is empty\n", pt_id);
				continue;
			}
			geometry_msgs::Pose curr_pt;
			//printf("wp size: %d, %d\n", wp.rows, wp.cols);
			//pcl_cloud->push_back(pcl::PointXYZ(wp.at<float>(0), wp.at<float>(1), wp.at<float>(2)));
			curr_pt.position.x = wp.x();
			curr_pt.position.y = wp.y();
			curr_pt.position.z = wp.z();
			pt_array.poses.push_back(curr_pt);
			//printf("Done getting map point %d\n", pt_id);
		}
		//sensor_msgs::PointCloud2 ros_cloud;
		//pcl::toROSMsg(*pcl_cloud, ros_cloud);
		//ros_cloud.header.frame_id = "1";
		//ros_cloud.header.seq = ni;

		//printf("valid map pts: %lu\n", pt_array.poses.size()-1);

		//printf("ros_cloud size: %d x %d\n", ros_cloud.height, ros_cloud.width);
		//pub_cloud.publish(ros_cloud);
		pt_array.header.frame_id = "1";
		pt_array.header.seq = frame_id + 1;
		ROS_INFO("Publishing pts_and_pose with %lu points", pt_array.poses.size() - 1);
		pub_pts_and_pose.publish(pt_array);
		//pub_kf.publish(camera_pose);
	}
}

inline bool isInteger(const std::string & s){
	if (s.empty() || ((!isdigit(s[0])) && (s[0] != '-') && (s[0] != '+'))) return false;

	char * p;
	strtol(s.c_str(), &p, 10);

	return (*p == 0);
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps){
	ifstream fTimes;
	string strPathTimeFile = strPathToSequence + "/times.txt";
	fTimes.open(strPathTimeFile.c_str());
	while (!fTimes.eof()){
		string s;
		getline(fTimes, s);
		if (!s.empty()){
			stringstream ss;
			ss << s;
			double t;
			ss >> t;
			vTimestamps.push_back(t);
		}
	}

	string strPrefixLeft = strPathToSequence + "/image_0/";

	const int nTimes = vTimestamps.size();
	vstrImageFilenames.resize(nTimes);

	for (int i = 0; i < nTimes; i++)
	{
		stringstream ss;
		ss << setfill('0') << setw(6) << i;
		vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
	}
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg){
	// Copy the ros image message to cv::Mat.
	cv_bridge::CvImageConstPtr cv_ptr;
	try{
		cv_ptr = cv_bridge::toCvShare(msg);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	SLAM.TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
	publish(SLAM, pub_pts_and_pose, pub_all_kf_and_pts, frame_id);
	++frame_id;
}

bool parseParams(int argc, char **argv) {
	if (argc < 4){
		cerr << endl << "Usage: rosrun ORB_SLAM3 Monopub path_to_vocabulary path_to_settings path_to_sequence/camera_id/-1 <image_topic>" << endl;
		return 1;
	}
	if (isInteger(std::string(argv[3]))) {
		int camera_id = atoi(argv[3]);
		if (camera_id >= 0){
			read_from_camera = true;
			printf("Reading images from camera with id %d\n", camera_id);
			cap_obj.open(camera_id);
			if (!(cap_obj.isOpened())) {
				printf("Camera stream could not be initialized successfully\n");
				ros::shutdown();
				return 0;
			}
			int img_height = cap_obj.get(cv::CAP_PROP_FRAME_HEIGHT);
			int img_width = cap_obj.get(cv::CAP_PROP_FRAME_WIDTH);
			printf("Images are of size: %d x %d\n", img_width, img_height);
		}
		else {
			read_from_topic = true;
			if (argc > 4){
				image_topic = std::string(argv[4]);
			}
			printf("Reading images from topic %s\n", image_topic.c_str());
		}
	}
	else {
		LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);
	}
	if (argc >= 5) {
		all_pts_pub_gap = atoi(argv[4]);
	}
	printf("all_pts_pub_gap: %d\n", all_pts_pub_gap);
	return 1;
}