#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Range.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"

#include <cv_bridge/cv_bridge.h>

#include <string>
#include <sstream>


using namespace cv;
using namespace std;

//Global variables
cv::Mat imgFront, imgBottom;
std::string cmdXStr, cmdYStr, cmdZStr, cmdRotStr;
std::string sonarString;
std::string takeOffLandStr;
Mat imgTongue(350,200, CV_8UC3, Scalar(0,0,0));
Mat imgTongueDraw;

void callbackTakeOff(const std_msgs::Empty::ConstPtr& msg){

      std::ostringstream ss;
      ss << "Flying";
     takeOffLandStr = ss.str();
}

void callbackLand(const std_msgs::Empty::ConstPtr& msg){

      std::ostringstream ss;
      ss << "Grounded";
     takeOffLandStr = ss.str();
}

void callbackSonarHeight(const sensor_msgs::Range::ConstPtr& msg){

      std::ostringstream ss;
      ss << "Sonar height: " << msg->range;
     sonarString = ss.str();
}

void callbackCMDVel(const geometry_msgs::Twist::ConstPtr& msg){

  std::ostringstream ssX, ssY, ssZ, ssRotZ;
  ssX << "Velocity Forward/Backwards: " << msg->linear.x;
  //ssY << "Y Vel: " << msg->linear.y;
  ssZ << "Velocity Elevation: " << msg->linear.z;
  ssRotZ << "Velocity Rotation: " << msg->angular.z;

  cmdXStr =ssX.str();
  //cmdYStr =ssY.str();
  cmdZStr =ssZ.str();
  cmdRotStr = ssRotZ.str();
}

void callbackImageFront(const sensor_msgs::ImageConstPtr& msg){

     cv_bridge::CvImagePtr img;
     img = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::RGB8);
     imgFront = img->image;

     resize(imgFront, imgFront, cv::Size(), 2, 2);

}
void callbackImageBottom(const sensor_msgs::ImageConstPtr& msg){

     cv_bridge::CvImagePtr img;
     img = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::RGB8);
     imgBottom = img->image;

     resize(imgBottom, imgBottom, cv::Size(), 0.5, 0.5);

}
void callbackTongue(const geometry_msgs::Pose2D::ConstPtr& msg){

      imgTongueDraw = imgTongue.clone();
     float x = msg->x;
     float y = msg->y;

     float scaleX = 0.7815; //200/256
     float scaleY = 1.8135; //350/193

     if(x > 128){
       x = 383 - x;
     }
     else if(x < 128){
       x = 128 - x;
     }

     if((30 < y) && (y < 128)){
       y = 126 -y;
     }
     else if ((128 <y)&&( y < 226)){
       y = 323-y;
     }

     x = x * scaleX;
     y = y * scaleY;

     ROS_INFO("%f", x);
     ROS_INFO("%f", y);


     rectangle(imgTongueDraw, Point(x,y), Point(x+2,y+2), Scalar(0,0,255), 4);


}

void createTongueImg(){
  //Mat imgTongue(193,256, CV_8UC3, Scalar(0,0,0));

  //cvtColor(imgTongue,imgTongue, COLOR_BGR2RGB);

  int colorR = 255;
  int colorG = 255;
  int colorB = 255;

  //Add lines

  //Top part

  line(imgTongue,Point(0,imgTongue.rows*0.2487),Point(imgTongue.cols,imgTongue.rows*0.2487),Scalar(colorR,colorG,colorB),1);
  line(imgTongue,Point(imgTongue.cols*0.5,0),Point(imgTongue.cols*0.5,imgTongue.rows*0.4974),Scalar(colorR,colorG,colorB),1);

  //Bottom part
  line(imgTongue,Point(0,imgTongue.rows*0.4974),Point(imgTongue.cols,imgTongue.rows*0.4974),Scalar(colorR,colorG,colorB),1);
  line(imgTongue,Point(0,imgTongue.rows),Point(imgTongue.cols,imgTongue.rows*0.4974),Scalar(colorR,colorG,colorB),1);
  line(imgTongue,Point(0,imgTongue.rows*0.4974),Point(imgTongue.cols,imgTongue.rows),Scalar(colorR,colorG,colorB),1);

  //Add text

  std::string strUp = "UP";
  std::string strDown = "DOWN";
  std::string strTakeOff = "Start";
  std::string strLand = "Land";

  std::string strForward = "F";
  std::string strRotateRight = "R";
  std::string strRotateLeft = "L";
  std::string strStop = "S";

  cv::putText(imgTongue, strUp, Point2f(imgTongue.cols*0.175, imgTongue.rows*0.125*3), FONT_HERSHEY_DUPLEX, 0.7, Scalar(colorR, colorG, colorB), 2, 8, false);
  cv::putText(imgTongue, strDown, Point2f(imgTongue.cols*0.6125, imgTongue.rows*0.125*3), FONT_HERSHEY_DUPLEX, 0.7, Scalar(colorR, colorG, colorB), 2, 8, false);
  cv::putText(imgTongue, strTakeOff, Point2f(imgTongue.cols*0.075, imgTongue.rows*0.125), FONT_HERSHEY_DUPLEX, 0.7, Scalar(colorR, colorG, colorB), 2, 8, false);
  cv::putText(imgTongue, strLand, Point2f(imgTongue.cols*0.6125, imgTongue.rows*0.125), FONT_HERSHEY_DUPLEX, 0.7, Scalar(colorR, colorG, colorB), 2, 8, false);

  cv::putText(imgTongue, strForward, Point2f(imgTongue.cols*0.47, imgTongue.rows*0.60), FONT_HERSHEY_DUPLEX, 0.7, Scalar(colorR, colorG, colorB), 2, 8, false);
  cv::putText(imgTongue, strRotateRight, Point2f(imgTongue.cols*0.80, imgTongue.rows*0.75), FONT_HERSHEY_DUPLEX, 0.7, Scalar(colorR, colorG, colorB), 2, 8, false);
  cv::putText(imgTongue, strRotateLeft, Point2f(imgTongue.cols*0.12, imgTongue.rows*0.75), FONT_HERSHEY_DUPLEX, 0.7, Scalar(colorR, colorG, colorB), 2, 8, false);
  cv::putText(imgTongue, strStop, Point2f(imgTongue.cols*0.47, imgTongue.rows*0.9), FONT_HERSHEY_DUPLEX, 0.7, Scalar(colorR, colorG, colorB), 2, 8, false);

  //imshow("tongue", imgTongue);
  //waitKey(0);
}

int main(int argc, char **argv) {

  imgFront = imread("/home/albert/Pictures/flowers.jpg");
  imgBottom = imread("/home/albert/Pictures/flowers.jpg");
  resize(imgBottom, imgBottom, cv::Size(), 0.1, 0.1);

  createTongueImg();
  imgTongueDraw = imgTongue.clone();




  ros::init(argc, argv,"Cool");

  std::ostringstream ssX, ssY, ssZ, ssRotZ, ss;
  ssX << "Velocity Forward/Backwards: " << 0;
  ssZ << "Velocity Elevation: " << 0;
  ssRotZ << "Velocity Rotation: " << 0;

  cmdXStr =ssX.str();
  cmdZStr =ssZ.str();
  cmdRotStr = ssRotZ.str();

  ss << "Grounded";
  takeOffLandStr = ss.str();

  ros::NodeHandle nh;
  ros::Subscriber sub_ImageFront = nh.subscribe("/ardrone/front/image_raw", 5, callbackImageFront);
  ros::Subscriber sub_ImageBottom = nh.subscribe("/ardrone/bottom/image_raw", 5, callbackImageBottom);
  ros::Subscriber sub_SonarHeight = nh.subscribe("/sonar_height", 5, callbackSonarHeight);
  ros::Subscriber sub_CMDVel = nh.subscribe("/cmd_vel", 5, callbackCMDVel);
  ros::Subscriber sub_TakeOff = nh.subscribe("/ardrone/takeoff", 5, callbackTakeOff);
  ros::Subscriber sub_Land = nh.subscribe("/ardrone/land", 5, callbackLand);
  ros::Subscriber sub_tongueCoordinates = nh.subscribe("/AU_position", 5, callbackTongue);

  ros::Rate loop_rate(10);

    while (ros::ok()){

      cv::putText(imgFront, sonarString, Point2f(imgFront.cols*0.05, imgFront.rows*0.95), FONT_HERSHEY_DUPLEX, 0.7, Scalar(255, 255, 255), 2, 8, false);
      cv::putText(imgFront, cmdZStr, Point2f(imgFront.cols*0.05, imgFront.rows*0.92), FONT_HERSHEY_DUPLEX, 0.7, Scalar(255, 255, 255), 2, 8, false);
      cv::putText(imgFront, cmdXStr, Point2f(imgFront.cols*0.05, imgFront.rows*0.89), FONT_HERSHEY_DUPLEX, 0.7, Scalar(255, 255, 255), 2, 8, false);
      cv::putText(imgFront, cmdRotStr, Point2f(imgFront.cols*0.05, imgFront.rows*0.86), FONT_HERSHEY_DUPLEX, 0.7, Scalar(255, 255, 255), 2, 8, false);
      cv::putText(imgFront, takeOffLandStr, Point2f(imgFront.cols*0.05, imgFront.rows*0.83), FONT_HERSHEY_DUPLEX, 0.7, Scalar(255, 255, 255), 2, 8, false);
      //cv::putText(imgMat, cmdYStr, Point2f(imgMat.cols*0.05, imgMat.rows*0.85), FONT_HERSHEY_DUPLEX, 0.7, Scalar(255, 255, 255), 2, 8, false);



      imgBottom.copyTo(imgFront(cv::Rect(imgFront.cols*0.72, imgFront.rows*0.72, imgBottom.cols, imgBottom.rows)));
      rectangle(imgFront,Point((imgFront.cols*0.72)-1,(imgFront.rows*0.72)-1),Point((imgFront.cols*0.72+imgBottom.cols)+1,(imgFront.rows*0.72+imgBottom.rows)+1),(0,255,0),2);

      // min_x, min_y should be valid in A and [width height] = size(B)
      cv::Rect roi = cv::Rect(0, 0, imgTongue.cols, imgTongue.rows);

      // Blend the ROI of A with B into the ROI of out_image
      float alpha = 0.8;
      cv::addWeighted(imgFront(roi),alpha,imgTongueDraw,1-alpha,0.0,imgFront(roi));
      cv::imshow("Ar Drone",imgFront);
      //cv::imshow("Ar Drone",imgTongueDraw);

      waitKey(1);

        ros::spinOnce();
        loop_rate.sleep();
    }

  return 0;
}
