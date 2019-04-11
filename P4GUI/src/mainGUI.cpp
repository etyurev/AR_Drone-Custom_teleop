#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Range.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "ardrone_autonomy/Navdata.h"

#include <cv_bridge/cv_bridge.h>

#include <string>
#include <sstream>


using namespace cv;
using namespace std;

//Global variables
cv::Mat imgFront, imgBottom;
std::string sonarString;
std::string takeOffLandStr;
std::string xVelStr;
float xVel, zVel;
Mat imgTongue(350,200, CV_8UC3, Scalar(0,0,0));
Mat imgTongueDraw;
bool auPosReceived;
float cmdLinX, cmdLinZ, cmdAngZ;
int fadeAlpha = 1;
HOGDescriptor hog;
std::string batteryPercentString;
float batteryPercentValue;
std::string altidString;
float altitude;
std::string wifiString;
float wifi;

static void detectAndDraw(const HOGDescriptor &hog, Mat &img)
{
    vector<Rect> found, found_filtered;
    double t = (double) getTickCount();
    // Run the detector with default parameters. to get a higher hit-rate
    // (and more false alarms, respectively), decrease the hitThreshold and
    // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
    hog.detectMultiScale(img, found, 0, Size(8,8), Size(32,32), 1.05, 2);
    t = (double) getTickCount() - t;
    cout << "detection time = " << (t*1000./cv::getTickFrequency()) << " ms" << endl;

    for(size_t i = 0; i < found.size(); i++ )
    {
        Rect r = found[i];

        size_t j;
        // Do not add small detections inside a bigger detection.
        for ( j = 0; j < found.size(); j++ )
            if ( j != i && (r & found[j]) == r )
                break;

        if ( j == found.size() )
            found_filtered.push_back(r);
    }

    for (size_t i = 0; i < found_filtered.size(); i++)
    {
        Rect r = found_filtered[i];

        // The HOG detector returns slightly larger rectangles than the real objects,
        // so we slightly shrink the rectangles to get a nicer output.
        r.x += cvRound(r.width*0.1);
        r.width = cvRound(r.width*0.8);
        r.y += cvRound(r.height*0.07);
        r.height = cvRound(r.height*0.8);
        rectangle(img, r.tl(), r.br(), cv::Scalar(0,255,0), 3);
    }
}

void callbackNavdata(const ardrone_autonomy::Navdata::ConstPtr& msg){

    //Subscriber to all the navigation data from the drone.

    xVel = msg->vx*0.001;
    if (xVel < 0.05){
      xVel = 0.0;
    }

    zVel = msg->vz*0.001;
    if (zVel < 0.05){
      zVel = 0.0;
    }
      std::ostringstream ss, s1, s2;
      ss.precision(2);
      ss << xVel;

     xVelStr = ss.str();

     s1  << msg->batteryPercent << "%";
     batteryPercentValue=msg->batteryPercent;
    batteryPercentString = s1.str();

    s2<< msg->altd/10;// << "cm";
    altitude=msg->altd;
    altidString = s2.str();
}

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

      //Subscriber to the sonar, telling the elevation over objects underneath
      //the drone.

      std::ostringstream ss;
      ss << "Sonar height: " << msg->range;
     sonarString = ss.str();
}

void callbackCMDVel(const geometry_msgs::Twist::ConstPtr& msg){

  //Subscribes to the command inputs to the drone, both linear and angular
  //Updates the global variables

  cmdLinX = msg->linear.x;
  cmdAngZ = msg->angular.z;
  cmdLinZ = msg->linear.z;
}

void callbackImageFront(const sensor_msgs::ImageConstPtr& msg){

    //Subscriber to the front image of the drone, updates the global sub_Image
    //variable
     cv_bridge::CvImagePtr img;
     img = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
     imgFront = img->image;

     detectAndDraw(hog,imgFront);
     resize(imgFront, imgFront, cv::Size(), 2, 2);

     //Resize the top image
     //resize(imgFront, imgFront, cv::Size(), 2, 2);

}
void callbackImageBottom(const sensor_msgs::ImageConstPtr& msg){

    //Subscriber to the bottom image of the drone, updates the global imgBottom
    //variable
     cv_bridge::CvImagePtr img;
     img = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
     imgBottom = img->image;

     //Resize the bottom image
     resize(imgBottom, imgBottom, cv::Size(), 0.5, 0.5);

}
void callbackTongue(const geometry_msgs::Pose2D::ConstPtr& msg){

    //Subscriber to AU_position, aka the coodinates from the ITCI

     float x = msg->x;
     float y = msg->y;

     float scaleX = 0.7815; //200/256
     float scaleY = 1.8135; //350/193

     if ((x == 128) && (y == 128)){
       auPosReceived = false;
     }
     else{
       auPosReceived = true;
     }

     // if a command from the ITCI has been received update the ITCI image
     if(auPosReceived == true){

       // Convert AU_position coordinates into openCV x,y coordinates

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

       //Scale the coordinates according to the imgTongue
       x = x * scaleX;
       y = y * scaleY;

       //Clone the orignal ITCI image and draw a rectangle on the clone
       imgTongueDraw = imgTongue.clone();
       rectangle(imgTongueDraw, Point(x,y), Point(x+2,y+2), Scalar(0,0,255), 4);
   }

}

void createTongueImg(){

  //Change the color scheme here

  int colorR = 255;
  int colorG = 255;
  int colorB = 255;

  //Add lines

  ///Top part

  line(imgTongue,Point(0,imgTongue.rows*0.2487),Point(imgTongue.cols,imgTongue.rows*0.2487),Scalar(colorR,colorG,colorB),1);
  line(imgTongue,Point(imgTongue.cols*0.5,0),Point(imgTongue.cols*0.5,imgTongue.rows*0.4974),Scalar(colorR,colorG,colorB),1);

  ///Bottom part
  line(imgTongue,Point(0,imgTongue.rows*0.4974),Point(imgTongue.cols,imgTongue.rows*0.4974),Scalar(colorR,colorG,colorB),1);
  line(imgTongue,Point(0,imgTongue.rows),Point(imgTongue.cols,imgTongue.rows*0.4974),Scalar(colorR,colorG,colorB),1);
  line(imgTongue,Point(0,imgTongue.rows*0.4974),Point(imgTongue.cols,imgTongue.rows),Scalar(colorR,colorG,colorB),1);

  //Add text

  /// Declare strings
  std::string strUp = "UP";
  std::string strDown = "DOWN";
  std::string strTakeOff = "Start";
  std::string strLand = "Land";

  std::string strForward = "F";
  std::string strRotateRight = "R";
  std::string strRotateLeft = "L";
  std::string strStop = "S";

  /// Put text on the imgTongue
  cv::putText(imgTongue, strDown, Point2f(imgTongue.cols*0.175, imgTongue.rows*0.125*3), FONT_HERSHEY_DUPLEX, 0.7, Scalar(colorR, colorG, colorB), 2, 8, false);
  cv::putText(imgTongue, strUp, Point2f(imgTongue.cols*0.6125, imgTongue.rows*0.125*3), FONT_HERSHEY_DUPLEX, 0.7, Scalar(colorR, colorG, colorB), 2, 8, false);
  cv::putText(imgTongue, strTakeOff, Point2f(imgTongue.cols*0.075, imgTongue.rows*0.125), FONT_HERSHEY_DUPLEX, 0.7, Scalar(colorR, colorG, colorB), 2, 8, false);
  cv::putText(imgTongue, strLand, Point2f(imgTongue.cols*0.6125, imgTongue.rows*0.125), FONT_HERSHEY_DUPLEX, 0.7, Scalar(colorR, colorG, colorB), 2, 8, false);

  cv::putText(imgTongue, strForward, Point2f(imgTongue.cols*0.47, imgTongue.rows*0.60), FONT_HERSHEY_DUPLEX, 0.7, Scalar(colorR, colorG, colorB), 2, 8, false);
  cv::putText(imgTongue, strRotateRight, Point2f(imgTongue.cols*0.80, imgTongue.rows*0.75), FONT_HERSHEY_DUPLEX, 0.7, Scalar(colorR, colorG, colorB), 2, 8, false);
  cv::putText(imgTongue, strRotateLeft, Point2f(imgTongue.cols*0.12, imgTongue.rows*0.75), FONT_HERSHEY_DUPLEX, 0.7, Scalar(colorR, colorG, colorB), 2, 8, false);
  cv::putText(imgTongue, strStop, Point2f(imgTongue.cols*0.47, imgTongue.rows*0.9), FONT_HERSHEY_DUPLEX, 0.7, Scalar(colorR, colorG, colorB), 2, 8, false);

  //imshow("tongue", imgTongue);
  //waitKey(0);
}

float getAlpha(){

  //If command from ITCI is available, alpha has a higher value
  if(auPosReceived == true){
    fadeAlpha = 10;
    return(0.5);
  }
  fadeAlpha--;
  if (fadeAlpha < 2){
    fadeAlpha = 1;
  }
  return (0.1+(fadeAlpha*0.04));
}

void splitRGBAlpha(Mat& img, Mat& mask){
  vector<Mat> rgbLayer;
  split(img,rgbLayer);         // seperate channels
  Mat cs[3] = { rgbLayer[0],rgbLayer[1],rgbLayer[2] };
  merge(cs,3,img);  // glue together again
  mask = rgbLayer[3];       // png's alpha channel used as mask
}

void drawVelocity(){

  cv::Mat imgArrow, mask;

  //Check what commands is sent to the drone
  if(cmdLinX > 0){
    //Read front image file
    imgArrow = imread("/home/albert/P4Ros/src/P4GUI/src/imgArrowFront.png",-1);
    resize(imgArrow, imgArrow, cv::Size(), 0.5, 0.5);

    splitRGBAlpha(imgArrow, mask);
    imgArrow.copyTo(imgFront(cv::Rect(imgFront.cols*0.465, imgFront.rows*0.85, imgArrow.cols, imgArrow.rows)),mask);


    //Add forward velocity to GUI
    cv::putText(imgFront, xVelStr, Point2f(imgFront.cols*0.48, imgFront.rows*0.84), FONT_HERSHEY_DUPLEX, 0.9, Scalar(255, 255, 255), 2, 8, false);
  }
  else if (cmdAngZ < 0){
    ///Read right arrow image file
    imgArrow = imread("/home/albert/P4Ros/src/P4GUI/src/imgArrowTurnRight.png",-1);

    resize(imgArrow, imgArrow, cv::Size(), 0.5, 0.5);
    splitRGBAlpha(imgArrow, mask);
    imgArrow.copyTo(imgFront(cv::Rect(imgFront.cols*0.53, imgFront.rows*0.84, imgArrow.cols, imgArrow.rows)),mask);
  }
  else if (cmdAngZ > 0){
    ///Read right arrow image file
    imgArrow = imread("/home/albert/P4Ros/src/P4GUI/src/imgArrowTurnLeft.png",-1);

    resize(imgArrow, imgArrow, cv::Size(), 0.5, 0.5);
    splitRGBAlpha(imgArrow, mask);
    imgArrow.copyTo(imgFront(cv::Rect(imgFront.cols*0.43, imgFront.rows*0.84, imgArrow.cols, imgArrow.rows)),mask);
  }

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



int main(int argc, char **argv) {

  //Initialize some starting images
  imgFront = imread("../flowers.jpg");
  resize(imgFront, imgFront, cv::Size(), 3, 3);
  imgBottom = imread("../flowers.jpg");
  resize(imgBottom, imgBottom, cv::Size(), 0.1, 0.1);

  createTongueImg();
  imgTongueDraw = imgTongue.clone();


  //hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());


  ros::init(argc, argv,"Cool");

  //All the relevant ros subscribers declared here
  ros::NodeHandle nh;
  ros::Subscriber sub_ImageFront = nh.subscribe("/ardrone/front/image_raw", 1, callbackImageFront);
  ros::Subscriber sub_ImageBottom = nh.subscribe("/ardrone/bottom/image_raw", 1, callbackImageBottom);
  //ros::Subscriber sub_SonarHeight = nh.subscribe("/sonar_height", 5, callbackSonarHeight);
  ros::Subscriber sub_CMDVel = nh.subscribe("/cmd_vel", 1, callbackCMDVel);
  ros::Subscriber sub_TakeOff = nh.subscribe("/ardrone/takeoff", 1, callbackTakeOff);
  ros::Subscriber sub_Land = nh.subscribe("/ardrone/land", 1, callbackLand);
  ros::Subscriber sub_tongueCoordinates = nh.subscribe("/AU_position", 1, callbackTongue);
  ros::Subscriber sub_Navdata = nh.subscribe("/ardrone/navdata", 1, callbackNavdata);

  ros::Rate loop_rate(30);

    while (ros::ok()){

      //Unit conversion
      float alt=0.7-altitude*0.00025;
      float batOrig=0.95+0.00033*batteryPercentValue;

      //Add readings from the drone to the GUI in form of text

      //cv::putText(imgFront, sonarString, Point2f(imgFront.cols*0.05, imgFront.rows*0.95), FONT_HERSHEY_DUPLEX, 0.7, Scalar(255, 255, 255), 2, 8, false);
      if (takeOffLandStr=="Grounded")
      {
        cv::putText(imgFront, "GROUNDED", Point2f(imgFront.cols*0.01, imgFront.rows*0.78), FONT_HERSHEY_DUPLEX, 0.7, Scalar(255, 255, 255), 2, 8, false);
      }
      else
      {
        cv::putText(imgFront, "FLYING", Point2f(imgFront.cols*0.01, imgFront.rows*0.13), FONT_HERSHEY_DUPLEX, 0.7, Scalar(255, 255, 255), 2, 8, false);
      }
      cv::putText(imgFront, batteryPercentString, Point2f(imgFront.cols*0.91, imgFront.rows*0.047), FONT_HERSHEY_DUPLEX, 0.7, Scalar(255, 255, 255), 2, 8, false);

      //Update GUI icons and images
      updateBatteryImage();

      rectangle(imgFront, Point(imgFront.cols*0.01, imgFront.rows*0.20), Point(imgFront.cols*0.06, imgFront.rows*0.70),(0,0,255),-1); //Altitude scala
      rectangle(imgFront, Point(imgFront.cols*0.02,imgFront.rows*alt), Point(imgFront.cols*0.05, imgFront.rows*0.70),(0,255,0),-1);
      cv::putText(imgFront, altidString, Point2f(imgFront.cols*0.03, imgFront.rows*(alt-0.02)), FONT_HERSHEY_DUPLEX, 0.7, Scalar(255, 255, 255), 2, 8, false);
      rectangle(imgFront, Point(imgFront.cols*0.955,imgFront.rows*0.025), Point(imgFront.cols*batOrig, imgFront.rows*0.058),(0,0,255),-1);

      //Add picture-in-picture of bottom camera
      imgBottom.copyTo(imgFront(cv::Rect(imgFront.cols*0.72, imgFront.rows*0.72, imgBottom.cols, imgBottom.rows)));
      rectangle(imgFront,Point((imgFront.cols*0.72)-1,(imgFront.rows*0.72)-1),Point((imgFront.cols*0.72+imgBottom.cols)+1,(imgFront.rows*0.72+imgBottom.rows)+1),(0,255,0),2);

      // min_x, min_y should be valid in imgFront and [width height] = size(imgTongue)
      cv::Rect roi = cv::Rect(imgFront.cols*0.82, imgFront.rows*0.03, imgTongue.cols, imgTongue.rows);

      // Blend the ROI of imgFront with imgTongueDraw into the ROI of out_image
      float alpha = getAlpha();
      cv::addWeighted(imgFront(roi),0.8,imgTongueDraw,alpha,0.0,imgFront(roi));
      drawVelocity();

      //Show the GUI
      cv::imshow("Ar Drone",imgFront);

      waitKey(1);

        ros::spinOnce();
        loop_rate.sleep();
    }

  return 0;
}
