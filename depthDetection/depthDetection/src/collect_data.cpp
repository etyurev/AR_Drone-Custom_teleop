#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>

#include <string>
#include <sstream>
#include <cmath>

#include "depthImage.h"

using namespace cv;
using namespace std;

Mat imgDepthMM;
bool pictureCaptured = false;

 void callbackImageDepth(const sensor_msgs::ImageConstPtr& msg){

      cv_bridge::CvImagePtr img;
      img = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);

      Mat imgDepth = img->image;
      cv::imshow("32 float", img->image);

      imgDepthMM = imgDepth.clone();


      //Convert to mm
      for (int x = 0; x < imgDepthMM.cols; x++) {
          for (int y = 0; y < imgDepthMM.rows; y++){
             imgDepthMM.at<float>(y,x) = imgDepthMM.at<float>(y,x)*1000;
          }
        }

      pictureCaptured = true;
 }

 int main(int argc, char **argv) {

   //File path for captured frames
   string path = "../data/raw/";
   string fileName = "dataRaw";
   string fileFormat = ".png";

   depthImage depth_img;

   ros::init(argc, argv,"Cool");

   //All the relevant ros subscribers declared here
   ros::NodeHandle nh;
   ros::Subscriber sub_ImageDepth = nh.subscribe("/royale_camera_driver/depth_image", 1, callbackImageDepth);

   int counter = 0;

   ros::Rate loop_rate(1);

     while (ros::ok()){

       //Bit shift captured image into 3 channels so it can be stored with
       //32 bit information

       Mat imgStorage;
       depth_img.convert_for_storage(imgDepthMM, imgStorage);

     if(pictureCaptured == true){

       //For visualization purposes

       Mat imgVis = Mat(imgDepthMM.rows, imgDepthMM.cols, CV_8UC3);
       imgVis = imgDepthMM.clone();

       depth_img.norm_visualize(imgVis, imgVis);
       depth_img.invert_black_white(imgVis);

       cvtColor(imgVis, imgVis, COLOR_GRAY2RGB);

       resize(imgVis, imgVis, cv::Size(), 3, 3);
       imshow("boxes", imgVis);

       //Save the captured depth image

       std::ostringstream ss;
       ss << path << fileName << counter << fileFormat;
       imwrite(ss.str(), imgStorage);
       counter++;

     }

       waitKey(1);

         ros::spinOnce();
         loop_rate.sleep();
     }

   return 0;
 }
