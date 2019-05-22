#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/ml.hpp>


#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>

#include <cv_bridge/cv_bridge.h>

#include <string>
#include <sstream>
#include <cmath>
#include <chrono>

#include "preProcess.h"
#include "depthImage.h"


using namespace cv;
using namespace cv::ml;
using namespace std;
using namespace std::chrono;

//Global variables
cv::Mat imgDepth, imgDepthVisualize, imgDepthMM, imgVis, imgVis2;
preProcess preProcessObject;
bool pictureCaptured = false;

float getDistance(Mat imgIn){
  ROS_INFO("made it");
  float sum = 0;
  for (int i = 0; i < imgIn.rows; i++){
    sum = sum + imgIn.at<float>(i,30);
  }
  return sum/60;
}


void draw_head_rectangles(Mat& imgIn, vector<int> head_points_x, vector<int> head_points_y, vector<float> gamma_values){
  //Input rectangles to be visualized
  //Draws all given rectangles on the given image

  for(size_t i=0; i<gamma_values.size(); i++){

    rectangle(imgIn,Point((head_points_x[i]-gamma_values[i]),head_points_y[i]),Point((head_points_x[i]+gamma_values[i]), (head_points_y[i]+(2*gamma_values [i]))),(255,255,255),1);

  }
}


 void callbackImageDepth(const sensor_msgs::ImageConstPtr& msg){

      cv_bridge::CvImagePtr img;
      img = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);

      Mat imgDepth = img->image;

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

  depthImage depth_img;

  ROS_INFO("Loading pre trained classifier.");
  Ptr<SVM> svm = Algorithm::load<SVM>("classifier_no_preprocess.svm");

  ros::init(argc, argv,"PredictHumans");

  //All the relevant ros subscribers and publishers declared here
  ros::NodeHandle nh;
  ros::Subscriber sub_ImageDepth = nh.subscribe("/royale_camera_driver/depth_image", 1, callbackImageDepth);
  ros::Publisher collision_pub = nh.advertise<std_msgs::Bool>("Collision", 1);

  VideoCapture cap("test_segmentation_classification.mp4");

  ros::Rate loop_rate(10);

    int enumerate=0;
    bool collision = false;
    std_msgs::Bool msg;

    float fpsSum = 0;
    float frameNumber = 0;

    while (ros::ok()){

    if(pictureCaptured == true){

      frameNumber++;

      high_resolution_clock::time_point t1 = high_resolution_clock::now();

      collision = false;


      vector<int> points_x, points_y, depth_value_vector;
      vector<float> gamma_val;

      //Apply preprocessing
      //preProcessObject.fill_img(imgDepthMM, imgDepthMM, 1);
      //medianBlur( imgDepthMM, imgDepthMM, 3);

      //Segment for heads
      preProcessObject.find_heads_and_process(imgDepthMM, points_x, points_y, gamma_val, depth_value_vector, 80, 350);

      //Create the grey image for visualization and hog extraction
      imgVis = Mat(imgDepthMM.rows, imgDepthMM.cols, CV_8UC3);
      imgVis = imgDepthMM.clone();

      depth_img.norm_visualize(imgVis, imgVis);
      depth_img.invert_black_white(imgVis);

      Mat imgSegmented = imgVis.clone();
      cvtColor(imgSegmented, imgSegmented, COLOR_GRAY2RGB);
      draw_head_rectangles(imgSegmented, points_x, points_y, gamma_val);
      resize(imgSegmented, imgSegmented, cv::Size(), 3, 3);
      //imshow("Segmented", imgSegmented);

      vector<int> heads_for_deletion;

      for(int i = 0; i < points_x.size(); i++){
        Mat imgROIVis; //imgROIDepth;

        //Create grey images scaled to 60x60 pix of the segmented region-
        //of-interest
        preProcessObject.crop_roi(imgVis, imgROIVis, points_x[i], points_y[i], gamma_val[i]);
        //preProcessObject.crop_roi(imgDepthMM, imgROIDepth, points_x[i], points_y[i], gamma_val[i]);

        //Extract hog features for region-of-interest-
        HOGDescriptor hog1( Size(60,60), Size(20,20), Size(10,10), Size(10,10), 9);
        vector<float> hogFeatures1;
        vector<Point> locations;

        hog1.compute(imgROIVis, hogFeatures1, Size(10,10), Size(0, 0), locations);

        Mat Hogfeat = Mat(1, hogFeatures1.size(), CV_32FC1);

        for(int j=0;j<hogFeatures1.size();j++){
          Hogfeat.at<float>(0,j)=hogFeatures1.at(j);
          }

        //Predict if human or not
        float response = svm->predict(Hogfeat);
        if(response == -1){
           heads_for_deletion.push_back(i);
         }
         else if(response == 1){
           if(imgDepthMM.at<float>(points_x[i],points_y[i])<2000 ){
           //if(getDistance(imgROIDepth(imgROIDepth))<2000 ){
             collision = true;
             //ROS_INFO("x,y: %f", imgDepthMM.at<float>(points_x[i],points_y[i]));
           }
           else{
             heads_for_deletion.push_back(i);
           }
         }

      }

      if(heads_for_deletion.size() > 0){
        for(int i = heads_for_deletion.size()-1; i<heads_for_deletion.size(); i--){
          if(i < points_x.size()){
            points_x.erase(points_x.begin()+heads_for_deletion[i]);
            points_y.erase(points_y.begin()+heads_for_deletion[i]);
            gamma_val.erase(gamma_val.begin()+heads_for_deletion[i]);

        }
        }
      }
      heads_for_deletion.clear();


      //Visualize detected humans
      cvtColor(imgVis, imgVis, COLOR_GRAY2RGB);
      draw_head_rectangles(imgVis, points_x, points_y, gamma_val);

      //Calculate FPS
      high_resolution_clock::time_point t2 = high_resolution_clock::now();
      auto duration = duration_cast<microseconds>( t2 - t1 ).count();
      float duration_float = duration;
      duration_float = duration_float/ 1000000;
      float fps = 1 / duration_float;
      //ROS_INFO("%f", fps);
      fpsSum = fpsSum+fps;
      float avgFps = fpsSum/frameNumber;

      std::ostringstream ss, ss1;
      ss << "FPS: " << fps;
      ss1 << "Average FPS: " << avgFps;
      string strFps = ss.str();
      string strAvg = ss1.str();

      resize(imgVis, imgVis, cv::Size(), 3, 3);
      //imshow("visualized", imgVis);

      Mat imgVideo = Mat(imgVis.rows, imgVis.cols+imgSegmented.cols, CV_8UC3);
      imgSegmented.copyTo(imgVideo(cv::Rect(0, 0, imgSegmented.cols, imgSegmented.rows)));
      imgVis.copyTo(imgVideo(cv::Rect(imgSegmented.cols-1, 0, imgVis.cols, imgVis.rows)));
      cv::putText(imgVideo, strFps, Point2f(10, 20), FONT_HERSHEY_DUPLEX, 0.7, Scalar(0, 0, 255), 2, 8, false);
      cv::putText(imgVideo, strAvg, Point2f(10, 40), FONT_HERSHEY_DUPLEX, 0.7, Scalar(0, 0, 255), 2, 8, false);
      imshow("Video", imgVideo);

      string path = "../data/test/";
      string fileName = "frame";
      string fileFormat = ".png";

      std::ostringstream savePath;
      savePath << path << fileName << frameNumber << fileFormat;
      imwrite(savePath.str(), imgVideo);

    }

      waitKey(1);

        ros::spinOnce();
        loop_rate.sleep();
    }

  return 0;
}
