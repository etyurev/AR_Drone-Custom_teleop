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
    ROS_INFO("%i, hello", i);
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

    for(int j=0;j<hogFeatures.size();j++){
      features.at<float>(i,j)=hogFeatures.at(j);
      }

  }

  for (size_t i=0; i<filenames_negative.size(); i++)
  {
    ROS_INFO("%i, hello", i);
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

    for(int j=0;j<hogFeatures.size();j++){
      features.at<float>(i+filenames_positive.size(),j)=hogFeatures.at(j);
      }

  }

  Ptr<SVM> svm = SVM::create();
    svm->setType(SVM::C_SVC);
    svm->setKernel(SVM::LINEAR);
    svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, 100, 1e-6));
    svm->train(features, ROW_SAMPLE, labels);


    Mat imgSetupRaw1 = imread("../data/negative/data1.png");
    Mat imgSetupMM1 = Mat(imgSetupRaw1.rows, imgSetupRaw1.cols, CV_32FC1);
    restore_mm(imgSetupRaw1, imgSetupMM1);


    Mat imgVis1 = imgSetupMM.clone();

    norm_visualize(imgVis1, imgVis1);
    invert_black_white(imgVis1);

    HOGDescriptor hog1( Size(60,60), Size(20,20), Size(10,10), Size(10,10), 9);
    vector<float> ders1;
    vector<Point> locs1;

    hog1.compute(imgVis1, ders1, Size(10,10), Size(5, 5), locs);

    Mat Hogfeat = Mat(1, ders1.size(), CV_32FC1);

    for(int i=0;i<ders1.size();i++){
      Hogfeat.at<float>(0,i)=ders1.at(i);
      }
    float response = svm->predict(Hogfeat);
    ROS_INFO("response, %f", response);

  ros::init(argc, argv,"Cool");

  //All the relevant ros subscribers declared here
  ros::NodeHandle nh;
  ros::Subscriber sub_ImageDepth = nh.subscribe("/royale_camera_driver/depth_image", 1, callbackImageDepth);

  ros::Rate loop_rate(10);

    while (ros::ok()){

    if(pictureCaptured == true){

      vector<int> points_x, points_y, depth_value_vector;
      vector<float> gamma_val;

      preProcessObject.fill_img(imgDepthMM, imgDepthMM, 1);
      medianBlur( imgDepthMM, imgDepthMM, 3);

      preProcessObject.find_heads_and_process(imgDepthMM, points_x, points_y, gamma_val, depth_value_vector, 80, 350);

      imgVis = Mat(imgDepthMM.rows, imgDepthMM.cols, CV_8UC3);
      imgVis = imgDepthMM.clone();

      norm_visualize(imgVis, imgVis);
      invert_black_white(imgVis);
      vector<int> heads_for_deletion;

      for(int i = 0; i < points_x.size(); i++){
        Mat imgROIVis, imgROIData;
        preProcessObject.crop_roi(imgVis, imgROIVis, points_x[i], points_y[i], gamma_val[i]);
        //preProcessObject.crop_roi(storeMM, imgROIData, points_x[i], points_y[i], gamma_val[i]);

        HOGDescriptor hog1( Size(60,60), Size(20,20), Size(10,10), Size(10,10), 9);
        vector<float> hogFeatures1;
        vector<Point> locations;

        hog1.compute(imgROIVis, hogFeatures1, Size(10,10), Size(5, 5), locations);

        Mat Hogfeat = Mat(1, hogFeatures1.size(), CV_32FC1);

        for(int j=0;j<hogFeatures1.size();j++){
          Hogfeat.at<float>(0,j)=hogFeatures1.at(j);
          }

        float response = svm->predict(Hogfeat);
        if(response == -1){
          heads_for_deletion.push_back(i);
          ROS_INFO("%i", i);
        }
          //ROS_INFO("response, %f", response);
          //if(response == 1){
          //  imshow("bam", imgROIVis);
          //  waitKey(0);
          //}
          //float confidence = 1.0 / (1.0 + exp(-response));
          //ROS_INFO("confidence, %f", score);

        //imwrite("../data/raw/dataRaw1.png", imgROI);

      }

      if(heads_for_deletion.size() > 0){
        for(int i = heads_for_deletion.size()-1; i<heads_for_deletion.size(); i--){
          if(i < points_x.size()){
            ROS_INFO("i: %i", i);
            ROS_INFO("size: %i", points_x.size());
            points_x.erase(points_x.begin()+heads_for_deletion[i]);
            points_y.erase(points_y.begin()+heads_for_deletion[i]);
            gamma_val.erase(gamma_val.begin()+heads_for_deletion[i]);
          //  i--;
          //depth_value_vector.erase(depth_value_vector.begin()+heads_for_deletion[i]);
          //i--;

        }
        }
      }
      heads_for_deletion.clear();

      cvtColor(imgVis, imgVis, COLOR_GRAY2RGB);



      draw_head_rectangles(imgVis, points_x, points_y, gamma_val);

      resize(imgVis, imgVis, cv::Size(), 3, 3);
      imshow("visualized", imgVis);

    }

      waitKey(1);

        ros::spinOnce();
        loop_rate.sleep();
    }

  return 0;
}
