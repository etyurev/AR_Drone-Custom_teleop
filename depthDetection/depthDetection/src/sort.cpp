#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>

#include <string>
#include <sstream>
#include <vector>

#include "depthImage.h"


using namespace cv;
using namespace std;


int main(int argc, char **argv) {


  ros::init(argc, argv,"Cool");

  //All the relevant ros subscribers declared here
  ros::NodeHandle nh;

  depthImage depth_img;
  Mat imgMM, imgVisualize;
  int counter = 0;
  vector<String> filenames;

  string pathPos = "../data/positive/";
  string pathNeg = "../data/negative/";
  string fileName = "data";
  string fileFormat = ".png";

  // Get all png in the folder
  cv::glob("../data/rawData/*.png", filenames);

  for (size_t i=0; i<filenames.size(); i++)
  {
      Mat imgRaw = imread(filenames[i]);
      imgMM = Mat(imgRaw.rows, imgRaw.cols, CV_32FC1);
      imgVisualize = Mat(imgMM.rows, imgMM.cols, CV_32FC1);

      depth_img.restore(imgRaw, imgMM);
      depth_img.norm_visualize(imgMM, imgVisualize);

      label:

      imshow("visualized", imgVisualize);
      int key = waitKey(0);
      //a = 97
      //b = 98
      //c = 99

      if(key == 97){
        ROS_INFO("Positive");

        std::ostringstream ss;
        ss << pathPos << fileName << counter << fileFormat;
        imwrite(ss.str(), imgRaw);

      }
      else if (key == 98){
        ROS_INFO("Negative");

        std::ostringstream ss;
        ss << pathNeg << fileName << counter << fileFormat;
        imwrite(ss.str(), imgRaw);
      }
      else if (key == 99){
        return 0;
      }
      else{
        ROS_INFO("Please press a for positive image, or b for negative image, or c to terminate.");
        goto label;
      }
      counter++;

  }

  return 0;
}
