#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/ml.hpp>

#include "sensor_msgs/Image.h"

#include <string>
#include <sstream>
#include <vector>
#include <chrono>


using namespace cv;
using namespace cv::ml;
using namespace std;
using namespace std::chrono;


void restore_mm(Mat& imgIn, Mat& imgOut){
  //Input: CV_8UC3 stored with bits in different vectors
  //Output: CV_32FC1 each pixel equals distance in mm

  for(int i = 0; i < imgIn.cols; i++){
     for(int j = 0; j < imgIn.rows; j++){

         uint64_t orig_int = 0;
         orig_int = imgIn.at<Vec3b>(j,i)[0] << 16;
         orig_int = orig_int+imgIn.at<Vec3b>(j,i)[1] << 8;
         orig_int = orig_int+imgIn.at<Vec3b>(j,i)[2] << 0;
         float val = orig_int;
         imgOut.at<float>(j,i) = val;
     }
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

int main(){

  vector<String> filenames_positive, filenames_negative;
  //Compute features for all positive samples
  cv::glob("../data/positive/*.png", filenames_positive);
  cv::glob("../data/negative/*.png", filenames_negative);

  Mat imgSetupRaw = imread("../data/positive/data6.png");
  Mat imgSetupMM = Mat(imgSetupRaw.rows, imgSetupRaw.cols, CV_32FC1);
  restore_mm(imgSetupRaw, imgSetupMM);


  Mat imgVis = imgSetupMM.clone();

  norm_visualize(imgVis, imgVis);
  invert_black_white(imgVis);
  //imshow("dd", imgVis);
  //waitKey(0);

  //HOGDescriptor hog;
  HOGDescriptor hog( Size(60,60), Size(20,20), Size(10,10), Size(10,10), 9);
  vector<float> ders;
  vector<Point> locs;

  hog.compute(imgVis, ders, Size(10,10), Size(5, 5), locs);


  int totalSamples = filenames_negative.size() + filenames_positive.size();

  Mat labels = Mat(totalSamples, 1, CV_32SC1);
  //Mat labels = Mat(totalSamples, 1, CV_8UC1);
  Mat features = Mat(totalSamples, ders.size(), CV_32FC1);


  for (size_t i=0; i<filenames_positive.size(); i++)
  {
    Mat imgRaw = imread(filenames_positive[i]);
    Mat imgMM = Mat(imgRaw.rows, imgRaw.cols, CV_32FC1);
    restore_mm(imgRaw, imgMM);

    Mat imgVis = imgMM.clone();

    norm_visualize(imgVis, imgVis);
    invert_black_white(imgVis);

    //Attach positive labels
    labels.at<int>(i,0) = 1;
    //Extract features

    HOGDescriptor hog( Size(60,60), Size(20,20), Size(10,10), Size(10,10), 9);
    vector<float> hogFeatures;
    vector<Point> locations;

    hog.compute(imgVis, hogFeatures, Size(10,10), Size(5, 5), locations);

    for(int j=0;j<ders.size();j++){
      features.at<float>(i,j)=hogFeatures.at(j);
      }

  }

  for (size_t i=0; i<filenames_negative.size(); i++)
  {
    Mat imgRaw = imread(filenames_negative[i]);
    Mat imgMM = Mat(imgRaw.rows, imgRaw.cols, CV_32FC1);
    restore_mm(imgRaw, imgMM);

    Mat imgVis = imgMM.clone();

    norm_visualize(imgVis, imgVis);
    invert_black_white(imgVis);

    //Attach negative labels
    int label_pos = filenames_positive.size() + i;
    labels.at<int>(label_pos,1) = -1;
    //Extract features

    HOGDescriptor hog( Size(60,60), Size(20,20), Size(10,10), Size(10,10), 9);
    vector<float> hogFeatures;
    vector<Point> locations;

    hog.compute(imgVis, hogFeatures, Size(10,10), Size(5, 5), locations);

    for(int j=0;j<ders.size();j++){
      features.at<float>(i+filenames_positive.size(),j)=hogFeatures.at(j);
      }

  }

  Ptr<SVM> svm = SVM::create();
    svm->setType(SVM::C_SVC);
    svm->setKernel(SVM::LINEAR);
    svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, 100, 1e-6));
    svm->train(features, ROW_SAMPLE, labels);

    vector<String> filenames_test;
    //Compute features for all positive samples
    cv::glob("../data/rawData/*.png", filenames_test);


    for (size_t i=0; i<filenames_test.size(); i++)
    {
      ROS_INFO("%i, test", i);
      high_resolution_clock::time_point t1 = high_resolution_clock::now();
      Mat imgRaw = imread(filenames_test[i]);
      Mat imgMM = Mat(imgRaw.rows, imgRaw.cols, CV_32FC1);
      restore_mm(imgRaw, imgMM);

      Mat imgVis = imgMM.clone();

      norm_visualize(imgVis, imgVis);
      invert_black_white(imgVis);

      //Extract features

      HOGDescriptor hog( Size(60,60), Size(20,20), Size(10,10), Size(10,10), 9);
      vector<float> hogFeatures;
      vector<Point> locations;

      hog.compute(imgVis, hogFeatures, Size(10,10), Size(5, 5), locations);
        Mat Hogfeat = Mat(1, hogFeatures.size(), CV_32FC1);

      for(int j=0;j<hogFeatures.size();j++){
        Hogfeat.at<float>(0,j)=hogFeatures.at(j);
        }
        float response = svm->predict(Hogfeat);
        ROS_INFO("response, %f", response);
        if(response == 1){
          imshow("human", imgVis);
          waitKey(0);
        }

        high_resolution_clock::time_point t2 = high_resolution_clock::now();

   auto duration = duration_cast<milliseconds>( t2 - t1 ).count();

  // ROS_INFO("%i",duration);

    }

    Mat imgSetupRaw1 = imread("../data/positive/data12.png");
    Mat imgSetupMM1 = Mat(imgSetupRaw1.rows, imgSetupRaw1.cols, CV_32FC1);
    restore_mm(imgSetupRaw1, imgSetupMM1);


    Mat imgVis1 = imgSetupMM1.clone();

    norm_visualize(imgVis1, imgVis1);
    invert_black_white(imgVis1);


    HOGDescriptor hog1( Size(60,60), Size(20,20), Size(10,10), Size(10,10), 9);
    vector<float> ders1;
    vector<Point> locs1;

    hog1.compute(imgVis1, ders1, Size(10,10), Size(5, 5), locs);

    Mat Hogfeat = Mat(1, ders1.size(), CV_32FC1);

    for(int j=0;j<ders1.size();j++){
      Hogfeat.at<float>(0,j)=ders1.at(j);
      }
    float response = svm->predict(Hogfeat);
    ROS_INFO("responsejjj, %f", response);

  return 0;
}
