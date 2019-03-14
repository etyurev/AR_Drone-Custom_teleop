#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Range.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"

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

int main(int argc, char **argv) {

  imgFront = imread("/home/albert/Pictures/flowers.jpg");
  imgBottom = imread("/home/albert/Pictures/flowers.jpg");
  resize(imgBottom, imgBottom, cv::Size(), 0.1, 0.1);
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

  ros::Rate loop_rate(30);

    while (ros::ok()){

      cv::putText(imgFront, sonarString, Point2f(imgFront.cols*0.05, imgFront.rows*0.95), FONT_HERSHEY_DUPLEX, 0.7, Scalar(255, 255, 255), 2, 8, false);
      cv::putText(imgFront, cmdZStr, Point2f(imgFront.cols*0.05, imgFront.rows*0.92), FONT_HERSHEY_DUPLEX, 0.7, Scalar(255, 255, 255), 2, 8, false);
      cv::putText(imgFront, cmdXStr, Point2f(imgFront.cols*0.05, imgFront.rows*0.89), FONT_HERSHEY_DUPLEX, 0.7, Scalar(255, 255, 255), 2, 8, false);
      cv::putText(imgFront, cmdRotStr, Point2f(imgFront.cols*0.05, imgFront.rows*0.86), FONT_HERSHEY_DUPLEX, 0.7, Scalar(255, 255, 255), 2, 8, false);
      cv::putText(imgFront, takeOffLandStr, Point2f(imgFront.cols*0.05, imgFront.rows*0.83), FONT_HERSHEY_DUPLEX, 0.7, Scalar(255, 255, 255), 2, 8, false);
      //cv::putText(imgMat, cmdYStr, Point2f(imgMat.cols*0.05, imgMat.rows*0.85), FONT_HERSHEY_DUPLEX, 0.7, Scalar(255, 255, 255), 2, 8, false);



      imgBottom.copyTo(imgFront(cv::Rect(imgFront.cols*0.72, imgFront.rows*0.72, imgBottom.cols, imgBottom.rows)));
      rectangle(imgFront,Point((imgFront.cols*0.72)-1,(imgFront.rows*0.72)-1),Point((imgFront.cols*0.72+imgBottom.cols)+1,(imgFront.rows*0.72+imgBottom.rows)+1),(0,255,0),2);
      cv::imshow("Ar Drone",imgFront);
      waitKey(1);

        ros::spinOnce();
        loop_rate.sleep();
    }

  return 0;
}
