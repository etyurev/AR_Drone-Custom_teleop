#include <ros/ros.h>
nclude <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Range.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "ardrone_autonomy/Navdata.h"
//#include "pr2_msgs/AccessPoint.h"
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
std::string batteryPercentString;
float batteryPercentValue;
std::string altidString;
float altitude;
std::string wifiString;
float wifi;

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
     img = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
     imgFront = img->image;

     resize(imgFront, imgFront, cv::Size(), 2, 2);

}
void callbackImageBottom(const sensor_msgs::ImageConstPtr& msg){

     cv_bridge::CvImagePtr img;
     img = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::RGB8);
     imgBottom = img->image;

     resize(imgBottom, imgBottom, cv::Size(), 0.5, 0.5);


}

void callbackBattery(const ardrone_autonomy::Navdata::ConstPtr& msg){

      std::ostringstream s1,s2;
      s1  << msg->batteryPercent << "%";
      batteryPercentValue=msg->batteryPercent;
     batteryPercentString = s1.str();

    s2<< msg->altd/10;// << "cm";
     altitude=msg->altd;
    altidString = s2.str();
}

void updateBatteryImage()
{
  cv::Mat imgBattery, mask;
  vector<Mat> rgbLayer;
  imgBattery = imread("/home/yevgen/ardrone_simulator/src/p4GUI/5percent.png",-1);
  resize(imgBattery, imgBattery, cv::Size(), 0.1, 0.1);
  split(imgBattery,rgbLayer);
  if(imgBattery.channels() == 4)
    {
        split(imgBattery,rgbLayer);         // seperate channels
        Mat cs[3] = { rgbLayer[0],rgbLayer[1],rgbLayer[2] };
        merge(cs,3,imgBattery);  // glue together again
        mask = rgbLayer[3];       // png's alpha channel used as mask
    }
  imgBattery.copyTo(imgFront(cv::Rect(imgFront.cols*0.95, imgFront.rows*0.019, imgBattery.cols, imgBattery.rows)),mask);
}
/*
void callbackWifi()
{
  std::ostringstream wifi;
  wifi << "Wifi strength: " << msg->signal;
  wifiString = wifi.str();
}
*/

int main(int argc, char **argv) {

  imgFront = imread("/home/yevgen/Desktop/ros.png");
  resize(imgFront, imgFront, cv::Size(), 10, 10);
  imgBottom = imread("/home/yevgen/Desktop/ros.png");
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
  //ros::Subscriber sub_SonarHeight = nh.subscribe("/sonar_height", 5, callbackSonarHeight);
  ros::Subscriber sub_CMDVel = nh.subscribe("/cmd_vel", 5, callbackCMDVel);
  ros::Subscriber sub_TakeOff = nh.subscribe("/ardrone/takeoff", 5, callbackTakeOff);
  ros::Subscriber sub_Land = nh.subscribe("/ardrone/land", 5, callbackLand);
  ros::Subscriber sub_Bat = nh.subscribe("/ardrone/navdata", 5, callbackBattery);
  //ros::Subscriber sub_wifi = nh.subscribe("",5,callbackWifi);



  ros::Rate loop_rate(30);

    while (ros::ok()){
      float alt=0.7-altitude*0.00025;
      float batOrig=0.95+0.00033*batteryPercentValue;

      cv::putText(imgFront, sonarString, Point2f(imgFront.cols*0.05, imgFront.rows*0.95), FONT_HERSHEY_DUPLEX, 0.7, Scalar(255, 255, 255), 2, 8, false);
if (takeOffLandStr=="Grounded")
{
  cv::putText(imgFront, "GROUNDED", Point2f(imgFront.cols*0.01, imgFront.rows*0.78), FONT_HERSHEY_DUPLEX, 0.7, Scalar(255, 255, 255), 2, 8, false);
}
else
{
  cv::putText(imgFront, "FLYING", Point2f(imgFront.cols*0.01, imgFront.rows*0.13), FONT_HERSHEY_DUPLEX, 0.7, Scalar(255, 255, 255), 2, 8, false);
}
cv::putText(imgFront, batteryPercentString, Point2f(imgFront.cols*0.91, imgFront.rows*0.047), FONT_HERSHEY_DUPLEX, 0.7, Scalar(255, 255, 255), 2, 8, false);

      updateBatteryImage();
      rectangle(imgFront, Point(imgFront.cols*0.01, imgFront.rows*0.20), Point(imgFront.cols*0.06, imgFront.rows*0.70),(0,0,255),-1); //Altitude scala
      rectangle(imgFront, Point(imgFront.cols*0.02,imgFront.rows*alt), Point(imgFront.cols*0.05, imgFront.rows*0.70),(0,255,0),-1);
      cv::putText(imgFront, altidString, Point2f(imgFront.cols*0.03, imgFront.rows*(alt-0.02)), FONT_HERSHEY_DUPLEX, 0.7, Scalar(255, 255, 255), 2, 8, false);
      rectangle(imgFront, Point(imgFront.cols*0.955,imgFront.rows*0.025), Point(imgFront.cols*batOrig, imgFront.rows*0.058),(0,0,255),-1);

      cv::imshow("Ar Drone",imgFront);
      waitKey(1);

        ros::spinOnce();
        loop_rate.sleep();
    }

  return 0;
}
