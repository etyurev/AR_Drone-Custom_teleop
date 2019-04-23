#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

#include "sensor_msgs/Image.h"

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
         ROS_INFO("%i hh", orig_int);
         float val = orig_int;
         imgOut.at<float>(j,i) = val;
     }
   }
 }

int main(){

  vector<String> filenames_positive, filenames_negative;
  //Compute features for all positive samples
  cv::glob("../data/raw/*.png", filenames_positive);
  cv::glob("../data/raw/*.png", filenames_negative);

  Mat imgSetupRaw = imread("../data/raw/dataRaw1.png");
  Mat imgSetupMM = Mat(imgSetupRaw.rows, imgSetupRaw.cols, CV_32FC1);



  int totalSamples = filenames_negative.size() + filenames_positive.size();

  Mat labels = Mat(totalSamples, 1, CV_32FC1);
  Mat features = Mat(totalSamples, 1, CV_32FC1);

  for (size_t i=0; i<filenames_positive.size(); i++)
  {
    Mat imgRaw = imread(filenames_positive[i]);
    Mat imgMM = Mat(imgRaw.rows, imgRaw.cols, CV_32FC1);
    restore_mm(imgRaw, imgMM);

    //Attach positive labels
    labels.at<float>(i,1) = 1;
    //Extract features

  }

  for (size_t i=0; i<filenames_negative.size(); i++)
  {
    Mat imgRaw = imread(filenames_positive[i]);
    Mat imgMM = Mat(imgRaw.rows, imgRaw.cols, CV_32FC1);
    restore_mm(imgRaw, imgMM);

    //Attach negative labels
    int label_pos = filenames_positive.size() + i;
    labels.at<float>(label_pos,1) = 1;
    //Extract features

  }


  return 0;
}
