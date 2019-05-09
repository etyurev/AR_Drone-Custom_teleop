#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <string>
#include <sstream>
#include <cmath>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>


using namespace cv;
using namespace std;

//globals
Mat imgDepth;
bool pictureCaptured = false;
bool collision = false;
bool col = false;

void obstacleAvoidance(bool pictureCaptured, Mat img){

  int collideFrame = 0;
  while(pictureCaptured){
    for(int x = 0; x < img.cols; x++){
      for(int y = 0; y < img.rows; y++){
        if(img.at<float>(y,x)>0 && img.at<float>(y,x)<0.8){
          col = true;
        }
        else{
          col = false;
        }
      }
    }
    pictureCaptured = false;
  }
}

bool fill_img(Mat& imgIn, Mat& imgOut, int win_size){

  imgOut = imgIn.clone();
  for(int y = 0; y < imgIn.rows; y++){
    for(int x = 0; x < imgIn.cols; x++){
      //https://www.utdallas.edu/~cxc123730/SPIE2016.pdf
      float val = imgIn.at<float>(y,x);
      vector<float> pixel_values;
      //cout<<"Max value: "<<max<<endl;
      if(val == 0){
        if((x+win_size <= imgIn.cols) && (x- win_size >= 0)){
          if((y+win_size <= imgIn.rows) && (y - win_size >= 0)){
            for(int yy = y-win_size; yy < y+1+win_size; yy++){
              for(int xx = x-win_size; xx < x+1+win_size; xx++){
                pixel_values.push_back(imgIn.at<float>(yy,xx));
              }
            }
            float max = *max_element(pixel_values.begin(), pixel_values.end());
            if(max > 0){
              imgOut.at<float>(y,x) = max;
            }
            else{
              imgOut.at<float>(y,x) = 0;
            }
          }
        }
      }
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

void callbackImageDepth(const sensor_msgs::ImageConstPtr& msg){

  cv_bridge::CvImagePtr img;
  img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  imgDepth = img->image;
  //cv::imshow("32 float", img->image);
  fill_img(imgDepth, imgDepth, 1);
  medianBlur(imgDepth,imgDepth, 3);
  Mat imgVis = imgDepth.clone();
  for(int x = 0; x < imgVis.cols; x++){
    for(int y = 0; y < imgVis.rows; y++){
      imgVis.at<float>(y,x) = imgVis.at<float>(y,x)/7.0;
    }
  }

  imgVis.convertTo(imgVis, CV_8UC1, 255.0);
  resize(imgVis, imgVis, cv::Size(), 5, 5);
  invert_black_white(imgVis);
  cv::imshow("Depth image", imgVis);
  pictureCaptured = true;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "obstacle_avoidance_node");
  ros::NodeHandle nh;
  ros::Subscriber sub_imageDepth = nh.subscribe("/royale_camera_driver/depth_image", 1, callbackImageDepth);
  ros::Publisher collision_pub = nh.advertise<std_msgs::Bool>("Collision", 1);
  ros::Rate loop_rate(10);
  int enumerate = 0;
  while(ros::ok()){

    std_msgs::Bool msg;
    msg.data = collision;
    collision_pub.publish(msg);

    waitKey(1);
    obstacleAvoidance(pictureCaptured, imgDepth);
    if(col == true){
      enumerate++;
    }else{
      enumerate = 0;
    }
    if(enumerate >=4){
      collision = true;
      enumerate = 0;
      col = false;
    }
    else{
      collision = false;
    }
    ROS_INFO("[%i]", enumerate);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
