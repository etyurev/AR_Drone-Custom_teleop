#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>

#include <string>
#include <sstream>


using namespace cv;
using namespace std;

//Global variables


void callbackImageDepth(const sensor_msgs::ImageConstPtr& msg){



     cv_bridge::CvImagePtr img;
     img = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
     cv::Mat imgDepth, imgDepthVisualize;
     imgDepth = img->image;
     cv::imshow("32 float", img->image);

     //Remove noise
     medianBlur( imgDepth, imgDepth, 3);

     //Normalize the depth image
     for (int x = 0; x < imgDepth.cols; x++) {
		     for (int y = 0; y < imgDepth.rows; y++){


            imgDepth.at<float>(y,x) = imgDepth.at<float>(y,x)/7.0;

         }
       }

     //Create the visualization
     imgDepthVisualize = imgDepth;
     imgDepthVisualize.convertTo(imgDepthVisualize, CV_8UC1, 255.0);

     //Swap black and white, so objects near appear white, and vice versa
     for (int x = 0; x < imgDepthVisualize.cols; x++) {
		     for (int y = 0; y < imgDepthVisualize.rows; y++){

            if (imgDepthVisualize.at<uchar>(y,x) > 0){
              imgDepthVisualize.at<uchar>(y,x) = 255 - imgDepthVisualize.at<uchar>(y,x);
              }
         }
       }

     resize(imgDepthVisualize, imgDepthVisualize, cv::Size(), 5, 5);

     //Show the Depth image visualized
     cv::imshow("Depth image",imgDepthVisualize );


}


int main(int argc, char **argv) {


  ros::init(argc, argv,"Cool");

  //All the relevant ros subscribers declared here
  ros::NodeHandle nh;
  ros::Subscriber sub_ImageDepth = nh.subscribe("/royale_camera_driver/depth_image", 1, callbackImageDepth);

  ros::Rate loop_rate(10);

    while (ros::ok()){

      waitKey(1);

        ros::spinOnce();
        loop_rate.sleep();
    }

  return 0;
}
