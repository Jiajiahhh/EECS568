#include "vo_features.h"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <sensor_msgs/Imu.h>
using namespace cv;
using namespace std;
#define MIN_NUM_FEAT 50

// IMP: Change the file directories (4 places) according to where your dataset is saved before running!

class VisualOdometry
{
public:
    VisualOdometry(ros::NodeHandle &nh)
    {
        R_f = Mat::eye(3, 3, CV_64F); // initialize
        t_f = Mat::zeros(3, 1, CV_64F);
        overall_dis_x = 0;
        overall_dis_y = 0;
        std::ofstream ofs("output.txt", std::ofstream::out | std::ofstream::trunc);
        std::ofstream ofs_gt("true.txt", std::ofstream::out | std::ofstream::trunc);
        image_sub_ = nh.subscribe("/d435/color/image_raw", 1, &VisualOdometry::imageCallback, this);
        depth_sub_ = nh.subscribe("/d435/depth/image_rect_raw", 1, &VisualOdometry::depthCallback, this);
        pose_sub_ = nh.subscribe("/t265/odom/sample", 1000, &VisualOdometry::stringCallback, this);
        imu_sub_ = nh.subscribe("/imu", 1000, &VisualOdometry::imuCallback, this);
    }
    Mat loadCalibration(const string &filePath);
    double calculateYawAngle(const Mat &R);
    double previous_main(cv::Mat frame1, cv::Mat frame2, double time);
    void stringCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
    {
        position_x = msg->pose.pose.position.x;
        position_y = msg->pose.pose.position.y;
        
    }
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        ori_x=msg->orientation.x;
        ori_y=msg->orientation.y;
        ori_z= msg->orientation.z;
        ori_w= msg->orientation.w;
        av_x=msg->angular_velocity.x;
        av_y= msg->angular_velocity.y;
        av_z= msg->angular_velocity.z;
        la_x= msg->linear_acceleration.x;
        la_y= msg->linear_acceleration.y;
        la_z= msg->linear_acceleration.z;
        // cout<<ori_x<<" "<<ori_y<<" "<<ori_z<<endl;
    }
    void depthCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr_dep;
        try
        {
            auto cv_ptr_dep = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_64FC1);
            curr_depth_frame = cv_ptr_dep->image;
            
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }
    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        
        try
        {
            curr_frame_time_ = msg->header.stamp.toSec();
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat current_frame = cv_ptr->image;
            cv::Mat dep = curr_depth_frame;
            // 判断当前帧是否为空
            // cout<<current_frame.empty()<<endl;
            if (!current_frame.empty() && !dep.empty())
            {
                // cv::imshow("Image", dep);
                waitKey(1);
                
                // cv::imshow("Image", dep);
                // 如果是第一帧或者特征点数目低于阈值，则进行特征点检测
                if (last_frame_.empty() || points1.size() < MIN_NUM_FEAT)
                {
                    // cout<<2<<endl;
                    cvtColor(current_frame, img_1, COLOR_BGR2GRAY);
                    featureDetection(img_1, points1); // 检测特征点
                    cout<<points1.size()<<endl;
                    last_frame_time_ = curr_frame_time_;
                }
                else
                {
                    // 特征点追踪
                    cvtColor(current_frame, img_1, COLOR_BGR2GRAY);
                    vector<uchar> status;
                    if (!points2.empty())
                    {
                        // std::cout << "Number of points1: " << points1.size() << std::endl;
                        featureTracking(img_2, img_1, points2, points1, status); // 确保points2到points1的追踪是有意义的
                    }
                    double time_diff = curr_frame_time_ - last_frame_time_;
                    Mat R;
                    std::ofstream out_file("output.txt", std::ios::out | std::ios::app);
                    std::ofstream out_file_true("true.txt", std::ios::out | std::ios::app);
                    double cx = 960.5;
                    double cy = 540.5;
                    double fx = 1206.8897719532354;
                    double fy = 1206.8897719532354;
                    double scale_sum = 0;
                    for (auto &point : points2)
                    {
                        if (point.x <= 0 || point.x > 1960 || point.y <= 0 || point.y > 1080)
                        {
                            points2.push_back(point);
                        }
                    }
                    for (auto &point : points1)
                    {
                        if (point.x <= 0 || point.x > 1960 || point.y <= 0 || point.y > 1080)
                        {
                            points1.push_back(point);
                        }
                    }
                    
                    // for (size_t i = 0; i < points2.size(); i++)
                    // {
                    //     cout << points1[i].x << "  " << points1[i].y << endl;
                    //     cout << points2[i].x << "  " << points2[i].y << endl;
                    //     cout << endl;
                    // }
                    std::vector<cv::Point2f> validPoints2;
                    std::vector<cv::Point3f> validPoints3;
                    std::vector<cv::Point3f> curr_validPoints3;
                    std::vector<double> diff_list;

                    // if (points2.size() <= 5)
                    //     return;
                    int zero_counter = 0;

                    for (size_t i = 0; i < points2.size(); i++)
                    {
                        // cout << 3 << endl;
                        // cout << points2[i].y << "   " << points2[i].x << endl;

                        // cout << curr_depth_frame.at<double>(points2[i].y, points2[i].x) << endl;
                        double curr_d = 0;
                        double prev_d = 0;

                        if (curr_depth_frame.at<double>(points2[i].y, points2[i].x) != 0 && curr_depth_frame.at<double>(points1[i].y, points1[i].x) != 0)
                        {
                            curr_d = curr_depth_frame.at<double>(points2[i].y, points2[i].x);
                            prev_d = pre_depth_frame.at<double>(points1[i].y, points1[i].x);
                        }
                        // cout << curr_d << "   " << prev_d << endl;
                        if ((curr_d >= 0.5 && curr_d <= 10) && (prev_d >= 0.5 && prev_d <= 10))
                        {
                            // transfer 2d to 3d
                            cv::Point3f curr_pt3D;
                            cv::Point3f prev_pt3D;
                            // cout<<curr_d<< " "<<prev_d<<endl;
                            curr_pt3D.x = (points2[i].x - cx) * curr_d / fx;
                            curr_pt3D.y = (points2[i].y - cy) * curr_d / fy;
                            prev_pt3D.x = (points1[i].x - cx) * prev_d / fx;
                            prev_pt3D.y = (points1[i].y - cy) * prev_d / fy;
                            if (std::abs(curr_pt3D.x) > 0.1 && std::abs(curr_pt3D.y) > 0.1 && std::abs(prev_pt3D.x) > 0.1 && std::abs(prev_pt3D.y) > 0.1 && std::abs(points2[i].x) > 0.1 && std::abs(points2[i].y) > 0.1 && std::abs(points1[i].x) > 0.1 && std::abs(points1[i].y) > 0.1)
                            {
                                curr_pt3D.z = curr_d;
                                prev_pt3D.z = prev_d;
                                validPoints2.push_back(points2[i]);
                                validPoints3.push_back(prev_pt3D);
                                curr_validPoints3.push_back(curr_pt3D);
                                double dx = curr_pt3D.x - prev_pt3D.x;
                                double dy = curr_pt3D.y - prev_pt3D.y;
                                double dz = curr_pt3D.z - prev_pt3D.z;
                                double diff = sqrt(dx * dx + dy * dy + dz * dz);
                                scale_sum += diff;
                                diff_list.push_back(diff);
                                // cout<<curr_pt3D<<endl;
                                // cout<<prev_pt3D<<endl;
                            }
                            else
                            {
                                zero_counter++;
                            }
                        }
                        else
                        {
                            zero_counter++;
                        }
                    }
                    cv::Mat point2Mat(validPoints2.size(), 2, CV_64F);
                    cv::Mat point3Mat(validPoints3.size(), 3, CV_64F);
                    for (size_t i = 0; i < validPoints2.size(); i++)
                    {
                        point2Mat.at<double>(i, 0) = validPoints2[i].x;
                        point2Mat.at<double>(i, 1) = validPoints2[i].y;
                    }
                    for (size_t i = 0; i < validPoints3.size(); ++i)
                    {
                        point3Mat.at<double>(i, 0) = validPoints3[i].x;
                        point3Mat.at<double>(i, 1) = validPoints3[i].y;
                        point3Mat.at<double>(i, 2) = validPoints3[i].z;
                    }
                    cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
                    cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
                    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1206.8897719532354, 0, 960.5,
                                            0, 1206.8897719532354, 540.5,
                                            0, 0, 1);
                    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);
                    // cout << point2Mat.rows << " " << point2Mat.rows << endl;
                    if (point3Mat.rows >= 10 && point2Mat.rows >= 10)
                    {
                        // cout << 6 << endl;
                        cv::solvePnP(point3Mat, point2Mat, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_EPNP);
                        // cout << tvec << endl;
                    }
                    // else
                    // {
                    //     points2.clear();
                    //     points2 = points1;
                    //     return;
                    // }

                    scale_sum /= (points2.size() - zero_counter);
                    cv::Rodrigues(rvec, R);
                    if (abs(tvec.at<double>(0)) > 0.08 || abs(tvec.at<double>(1)) > 0.08 || abs(tvec.at<double>(2)) > 0.08)
                    {
                        // cout << 1 << endl;
                        tvec.at<double>(0) = 0;
                        tvec.at<double>(1) = 0;
                        tvec.at<double>(2) = 0;
                    }
                    if (abs(tvec.at<double>(0)) < 0.001)
                        tvec.at<double>(0) = 0;
                    if (abs(tvec.at<double>(1)) < 0.001)
                        tvec.at<double>(1) = 0;
                    if (abs(tvec.at<double>(2)) < 0.001)
                        tvec.at<double>(2) = 0;
                    cout << tvec << endl;
                    R_f = R_f * R;
                    t_f = t_f + (tvec);
                    // t_f = t_f + (R_f*tvec);
                    // cout<<R_f<<endl;
                    double yaw = calculateYawAngle(R_f);
                    overall_dis_x = t_f.at<double>(2);
                    overall_dis_y = -t_f.at<double>(1);
                    overall_dis_z = t_f.at<double>(0);
                    out_file << "x= " << overall_dis_x << " y= " << -overall_dis_y << " yaw= " << yaw << " dt= " << time_diff;
                    // cout << validPoints3.size() << endl; 
                    // cout<<yaw<<endl;
                    if (validPoints3.size() > 10)
                    {
                        for (int i = 0; i < 10; i++)
                        {
                            out_file << "[" << validPoints3[i].x << "," << validPoints3[i].y << "|" << curr_validPoints3[i].x << "," << curr_validPoints3[i].y << "]";
                        }
                        out_file << endl;
                        out_file_true << position_x << " " << position_y<<" ";
                        // cout<<ori_x<<" "<<ori_y<<" "<<ori_z<<" "<<ori_w<<" "<<av_x<<" "<<av_y<<" "<<av_z<<" "<<la_x<<" "<<la_y<<" "<<la_z<<endl;
                        out_file_true<<ori_x<<" "<<ori_y<<" "<<ori_z<<" "<<ori_w<<" "<<av_x<<" "<<av_y<<" "<<av_z<<" "<<la_x<<" "<<la_y<<" "<<la_z<<endl;
                        cout << "x= " << overall_dis_x << " y= " << overall_dis_y << " yaw= " << yaw << " dt= " << time_diff << std::endl<< endl;
                        // // cout<<R_f<<endl;
                    }
                    waitKey(1);
                    points2.clear();
                    points2 = points1; 
                }
                last_frame_ = current_frame.clone();
                pre_depth_frame = curr_depth_frame.clone();
                last_frame_time_ = curr_frame_time_;
                img_2 = img_1.clone(); 
            }
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

private:
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Subscriber depth_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber imu_sub_;
    double curr_frame_time_;
    double time_diff;
    double last_frame_time_;
    cv::Mat R_f, t_f;
    cv::Mat img_1, img_2;
    double overall_dis_x;
    double overall_dis_y;
    double overall_dis_z;
    vector<Point2f> points1, points2;
    vector<Point3f> points3d_1, points3d_2;
    cv::Mat pre_depth_frame;
    cv::Mat curr_depth_frame;
    cv::Mat current_frame;
    cv::Mat last_frame_;
    double position_x;
    double position_y;
    double ori_x;double ori_y;double ori_z;double ori_w;
    double av_x;double av_y;double av_z;
    double la_x;double la_y;double la_z;
};
// camera calibration package
double VisualOdometry::calculateYawAngle(const Mat &R)
{
    // calculate radius
    double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0));
    bool singular = sy < 1e-6; // singularity

    double yaw;
    if (!singular)
    {
        yaw = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        yaw = atan2(-R.at<double>(0,1), R.at<double>(1,1));
    }

    // degree
    return yaw * (180.0 / CV_PI);
}

int main(int argc, char **argv)
{
    // initial ros node
    ros::init(argc, argv, "visodo_ros_v2");

    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    VisualOdometry vo(nh);
    ros::Publisher position_pub = nh.advertise<std_msgs::String>("position_publisher", 1000);
    ros::Rate loop_rate(10);
    int count = 0;
    ros::spin();
}