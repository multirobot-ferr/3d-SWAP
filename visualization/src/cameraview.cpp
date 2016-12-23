/*
 * Node for vision cameras UAVs
 * Author: Luis Manuel Ramirez de la Cova
 * Date: 25-05-2016
 * Modificate: 25-05-2016
 * Organization: University of Seville, GRVC
 * Description: Algorithm that can generate a file .world
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;

void imageCallback1(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::imshow("UAV1", cv_bridge::toCvShare(msg, "bgr8")->image);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void imageCallback2(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::imshow("UAV2", cv_bridge::toCvShare(msg, "bgr8")->image);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void imageCallback3(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::imshow("UAV3", cv_bridge::toCvShare(msg, "bgr8")->image);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    cv::namedWindow("UAV1");
    cv::startWindowThread();
    cv::namedWindow("UAV2");
    cv::startWindowThread();
    cv::namedWindow("UAV3");
    cv::startWindowThread();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub1 = it.subscribe("/mbzirc_1/camera_0/image_raw", 1, imageCallback1);
    image_transport::Subscriber sub2 = it.subscribe("/mbzirc_2/camera_0/image_raw", 1, imageCallback2);
    image_transport::Subscriber sub3 = it.subscribe("/mbzirc_3/camera_0/image_raw", 1, imageCallback3);

    ros::Rate rate(10.0);

    while(nh.ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
    ros::shutdown();
    cv::destroyAllWindows();
}
