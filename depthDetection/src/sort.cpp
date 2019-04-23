#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>

#include <string>
#include <sstream>
#include <vector>


using namespace cv;
using namespace std;

void restore_mm(Mat& imgIn, Mat& imgOut){
  //Input: CV_8UC3 stored with bits in different vectors
  //Output: CV_32FC1 each pixel equals distance in mm

  for(int i = 0; i < imgIn.cols; i++){
     for(int j = 0; j < imgIn.rows; j++){

         uint64_t orig_int = 0;
         orig_int = imgIn.at<Vec3b>(j,i)[0] << 16;
         orig_int = orig_int+imgIn.at<Vec3b>(j,i)[1] << 8;
         orig_int = orig_int+imgIn.at<Vec3b>(j,i)[2] << 0;

         imgOut.at<float>(j,i) = orig_int;
     }
   }
 }

void norm_visualize(Mat& imgIn, Mat& imgOut){
  //Input: CV_32FC1 in mm.
  //Output: CV_8UC1 in pixels between 0-255
  //Divides every value with 7 m. Therefore 255 equals 7 meters

  for(int i = 0; i < imgIn.cols; i++){
     for(int j = 0; j < imgIn.rows; j++){
         imgOut.at<float>(j,i) = imgIn.at<float>(j,i)/7000;
     }
   }
   imgOut.convertTo(imgOut, CV_8UC1, 255.0);
 }





int main(int argc, char **argv) {


  ros::init(argc, argv,"Cool");

  //All the relevant ros subscribers declared here
  ros::NodeHandle nh;

  //Mat storeMM = imread("../data/raw/dataRaw1.png");

  Mat imgMM, imgVisualize;
  int counter = 0;
  vector<String> filenames;

  string pathPos = "../data/positive/";
  string pathNeg = "../data/negative/";
  string fileName = "data";
  string fileFormat = ".png";

  // Get all png in the folder
  cv::glob("../data/rawData/*.png", filenames);

  for (size_t i=0; i<filenames.size(); i++)
  {
      Mat imgRaw = imread(filenames[i]);
      imgMM = Mat(imgRaw.rows, imgRaw.cols, CV_32FC1);
      imgVisualize = Mat(imgMM.rows, imgMM.cols, CV_32FC1);

      restore_mm(imgRaw, imgMM);
      norm_visualize(imgMM, imgVisualize);

      label:

      imshow("visualized", imgVisualize);
      int key = waitKey(0);
      //a = 97
      //b = 98
      //c = 99

      if(key == 97){
        ROS_INFO("Positive");

        std::ostringstream ss;
        ss << pathPos << fileName << counter << fileFormat;
        imwrite(ss.str(), imgRaw);

      }
      else if (key == 98){
        ROS_INFO("Negative");

        std::ostringstream ss;
        ss << pathNeg << fileName << counter << fileFormat;
        imwrite(ss.str(), imgRaw);
      }
      else if (key == 99){
        return 0;
      }
      else{
        ROS_INFO("Please press a for positive image, or b for negative image, or c to terminate.");
        goto label;
      }
      counter++;

  }

  return 0;
}
