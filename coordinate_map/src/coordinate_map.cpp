/**********************
 **********************/

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/pcl_macros.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace cv;
using namespace std;
class ImageConverter
{
private:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_depth; // depth subscriber

	ros::Subscriber camera_info_sub_;	// camera_info subscriber
	ros::Subscriber semantic_info_sub_; // semantic info subscriber to receive segmentation
	ros::Publisher pcl_pub;				// semantic pointcloud publisher

	sensor_msgs::CameraInfo camera_info;

	// remember to change this 
	//Simulation
	// int imageWidth = 1920;   
	// int imageHeight = 1080;

	//fieldtest rosbag
	int imageWidth = 640;   
	int imageHeight = 480;

	// Mat depthImage = Mat::zeros(imageWidth, imageHeight, CV_32FC1); // simulation
	Mat depthImage = Mat::zeros(imageWidth, imageHeight, CV_16UC1); // rosbag
	Mat semantic_info = Mat::zeros(imageWidth, imageHeight, CV_8UC1);

public:
	ImageConverter() : it_(nh_)
	{
		// topic sub for simulation:
		// image_sub_depth = it_.subscribe("/camera/depth/image_raw",
		// 								1, &ImageConverter::imageDepthCb, this);
		// camera_info_sub_ =
		// 	nh_.subscribe("/camera/rgb/camera_info", 1,
		// 				  &ImageConverter::cameraInfoCb, this);
		
		// topic sub for fieldtest:
		image_sub_depth = it_.subscribe("/d435/aligned_depth_to_color/image_raw",
										1, &ImageConverter::imageDepthCb, this);
		
		camera_info_sub_ =
			nh_.subscribe("/d435/aligned_depth_to_color/camera_info", 1,
						  &ImageConverter::cameraInfoCb, this);


		semantic_info_sub_ =
			nh_.subscribe("/Semantic_info", 1, &ImageConverter::semanticInfoCb, this);

		// topic pub:
		pcl_pub = nh_.advertise<sensor_msgs::PointCloud2>("semantic_pcl", 10);
	}

	~ImageConverter()
	{
		cv::destroyWindow("colorImage");
	}

	void cameraInfoCb(const sensor_msgs::CameraInfo &msg)
	{
		camera_info = msg;
	}

	void semanticInfoCb(const sensor_msgs::ImageConstPtr &msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr =
				cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
			semantic_info = cv_ptr->image;
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		semanticUpdate();
	}

	void imageDepthCb(const sensor_msgs::ImageConstPtr &msg)
	{
		cv_bridge::CvImagePtr cv_ptr;

		try
		{
			// cv_ptr =
			// 	cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);//simulation
			cv_ptr =
				cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);//rosbag
			depthImage = cv_ptr->image;
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
	}

	void semanticUpdate() // update semantic info
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		sensor_msgs::PointCloud2 output;
		std::cout << depthImage.size()<< "---\n";
		
		for (int u = 0; u < imageWidth; u++) 
			for (int v = 0; v < imageHeight; v++)
			{
				// float tmp_z = depthImage.at<float>(v, u); // simulation
				float tmp_z = 0.001 * depthImage.at<u_int16_t>(v, u); // rosbag
				if(tmp_z<=0 || tmp_z>=8) continue;
				float tmp_x = (u - camera_info.K.at(2)) / camera_info.K.at(0) * tmp_z;
				float tmp_y = (v - camera_info.K.at(5)) / camera_info.K.at(4) * tmp_z; 
				
				// constant add as static tf in simulation
				// float real_x = tmp_z+0.07;
				// float real_y = -tmp_x-0.037;
				// float real_z = -tmp_y+0.107;

				// constant add as static tf in field test
				float real_x = tmp_z;
				float real_y = -tmp_x;
				float real_z = -tmp_y+0.2;
				
				pcl::PointXYZRGB p;
				p.x = real_x;
				p.y = real_y;
				p.z = real_z;
				// semantic label is saved in pointcloud blue channel
				int info = semantic_info.at<u_int8_t>(v, u);
				p.r = 0;
				p.g = 0;
				p.b = info;
				// cout<<real_x<<"  "<<real_y<<"  "<<real_z<<endl;
				cloud->points.push_back(p);
			}
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudx = cloud;
		pcl::toROSMsg(*cloudx, output);
		// output.header.frame_id = "base_footprint";   //simulation
		output.header.frame_id = "t265_link";  //field test rosbag
		output.header.stamp = camera_info.header.stamp;
		pcl_pub.publish(output);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "coordinate_map");
	ImageConverter imageconverter;
	
	ros::spin();
	return (0);
}
