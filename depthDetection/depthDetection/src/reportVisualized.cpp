#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/ml.hpp>


#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>

#include <string>
#include <sstream>
#include <cmath>

#include "preProcess.h"


using namespace cv;
using namespace cv::ml;
using namespace std;

//Global variables
cv::Mat imgDepth, imgDepthVisualize, imgDepthMM, imgVis, imgVis2;
preProcess preProcessObject;
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

 void copy_to_CV8UC3(Mat& imgIn, Mat& imgOut){
   //Input: CV_8UC1
   //Output: Copies the grey vector to CV_8UC3 format

   for(int i = 0; i < imgIn.cols; i++){
      for(int j = 0; j < imgIn.rows; j++){
          imgOut.at<Vec3b>(j,i)[0] = imgIn.at<uchar>(j,i);
          imgOut.at<Vec3b>(j,i)[1] = imgIn.at<uchar>(j,i);
          imgOut.at<Vec3b>(j,i)[2] = imgIn.at<uchar>(j,i);
      }
    }
  }


void draw_head_rectangles(Mat& imgIn, vector<int> head_points_x, vector<int> head_points_y, vector<float> gamma_values){
  //Input rectangles to be visualized
  //Draws all given rectangles on the given image

  for(size_t i=0; i<gamma_values.size(); i++){

    rectangle(imgIn,Point((head_points_x[i]-gamma_values[i]),head_points_y[i]),Point((head_points_x[i]+gamma_values[i]), (head_points_y[i]+(2*gamma_values [i]))),(255,255,255),1);

  }
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


int main(int argc, char **argv) {

  Mat imgSetupRaw = imread("../data/raw/dataRaw27.png");
  Mat imgDepthMM = Mat(imgSetupRaw.rows, imgSetupRaw.cols, CV_32FC1);
  restore_mm(imgSetupRaw, imgDepthMM);

  vector<int> points_x, points_y, depth_value_vector;
  vector<float> gamma_val;


  Mat imgVis = Mat(imgDepthMM.rows, imgDepthMM.cols, CV_8UC3);
  imgVis = imgDepthMM.clone();

  norm_visualize(imgVis, imgVis);
  invert_black_white(imgVis);
  cvtColor(imgVis, imgVis, COLOR_GRAY2RGB);
  resize(imgVis, imgVis, cv::Size(), 3, 3);

  imshow("dd", imgVis);
  waitKey(0);
/*
  preProcessObject.fill_img(imgDepthMM, imgDepthMM, 1);

  imgVis = imgDepthMM.clone();

  norm_visualize(imgVis, imgVis);
  invert_black_white(imgVis);
  cvtColor(imgVis, imgVis, COLOR_GRAY2RGB);
  resize(imgVis, imgVis, cv::Size(), 3, 3);

  imshow("dd", imgVis);
  waitKey(0);

  medianBlur( imgDepthMM, imgDepthMM, 3);

  imgVis = imgDepthMM.clone();

  norm_visualize(imgVis, imgVis);
  invert_black_white(imgVis);
  cvtColor(imgVis, imgVis, COLOR_GRAY2RGB);
  resize(imgVis, imgVis, cv::Size(), 3, 3);

  imshow("dd", imgVis);
  waitKey(0);
*/
  preProcessObject.find_heads_and_process(imgDepthMM, points_x, points_y, gamma_val, depth_value_vector, 80, 350);

  imgVis = imgDepthMM.clone();
  norm_visualize(imgVis, imgVis);
  invert_black_white(imgVis);
  cvtColor(imgVis, imgVis, COLOR_GRAY2RGB);
  draw_head_rectangles(imgVis, points_x, points_y, gamma_val);
  //resize(imgVis, imgVis, cv::Size(), 3, 3);

  imshow("dd", imgVis);
  waitKey(0);

  imgVis = imgDepthMM.clone();
  norm_visualize(imgVis, imgVis);
  invert_black_white(imgVis);
  cvtColor(imgVis, imgVis, COLOR_GRAY2RGB);

  for(int i = 0; i < points_x.size(); i++){
    Mat imgROIVis, imgROIData;
    preProcessObject.crop_roi(imgVis, imgROIVis, points_x[i], points_y[i], gamma_val[i]);
    imshow("dd", imgROIVis);
    waitKey(0);

  }



  return 0;
}
