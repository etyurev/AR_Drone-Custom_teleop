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

using namespace cv;
using namespace std;

Mat imgDepthMM;
bool pictureCaptured = false;

void norm_visualize(Mat& imgIn, Mat& imgOut){
  //Input: CV_32FC1 in mm.
  //Output: CV_8UC1 in pixels between 0-255
  //Divides every value with 7 m. Therefore 255 equals 7 meters

  for(int i = 0; i < imgIn.cols; i++){
     for(int j = 0; j < imgIn.rows; j++){
         imgOut.at<float>(j,i) = imgIn.at<float>(j,i)/7000;
     }
   }
   imgOut.convertTo(imgOut, CV_8UC1, 255);
 }

 void invert_black_white(Mat& imgIn){
   for (int x = 0; x < imgIn.cols; x++) {
       for (int y = 0; y < imgIn.rows; y++){

          if (imgIn.at<uchar>(y,x) > 0){
            imgIn.at<uchar>(y,x) = 255 - imgIn.at<uchar>(y,x);
            }
       }
     }
 }

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


   ros::init(argc, argv,"Cool");

   //All the relevant ros subscribers declared here
   ros::NodeHandle nh;
   ros::Subscriber sub_ImageDepth = nh.subscribe("/royale_camera_driver/depth_image", 1, callbackImageDepth);

   int counter = 0;

   ros::Rate loop_rate(1);

     while (ros::ok()){

       string path = "../data/raw/";
       string fileName = "dataRaw";
       string fileFormat = ".png";

       Mat storeMM = cv::Mat(imgDepthMM.rows, imgDepthMM.cols, CV_8UC3);
      for(int i = 0; i < storeMM.cols; i++){
          for(int j = 0; j < storeMM.rows; j++){
              float orig = imgDepthMM.at<float>(j,i);

              uint32_t orig_int = int(orig);

              storeMM.at<Vec3b>(j,i)[0] = (uint8_t)((orig_int&0xFF0000) >> 16);
              storeMM.at<Vec3b>(j,i)[1] = (uint8_t)((orig_int&0x00FF00) >> 8);
              storeMM.at<Vec3b>(j,i)[2] = (uint8_t)((orig_int&0x0000FF) >> 0);

          }
      }


     //Create the visualization
     //imshow("converted back", storeMM);

     if(pictureCaptured == true){

       Mat imgVis = Mat(imgDepthMM.rows, imgDepthMM.cols, CV_8UC3);
       imgVis = imgDepthMM.clone();

       norm_visualize(imgVis, imgVis);
       invert_black_white(imgVis);

       std::ostringstream ss;
       ss << path << fileName << counter << fileFormat;
       imwrite(ss.str(), storeMM);
       counter++;

       cvtColor(imgVis, imgVis, COLOR_GRAY2RGB);

       resize(imgVis, imgVis, cv::Size(), 3, 3);
       imshow("boxes", imgVis);

     }

       waitKey(1);

         ros::spinOnce();
         loop_rate.sleep();
     }

   return 0;
 }
