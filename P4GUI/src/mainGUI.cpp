#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>


using namespace cv;
using namespace std;

//void callbackImageFront(const sensor_msgs::Image::ConstPtr& msg){
void callbackImageFront(const sensor_msgs::ImageConstPtr& msg){
    ROS_INFO("Image recieved");
     cv_bridge::CvImagePtr img;
     img = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_8UC3);
     cv::Mat imgMat = img->image;
     cv::imshow("my display",imgMat);
     waitKey(1);
}

int main(int argc, char **argv) {

  ros::init(argc, argv,"Cool");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/ardrone/front/image_raw", 5, callbackImageFront);

  ros::Rate loop_rate(10);

    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

  return 0;
}
