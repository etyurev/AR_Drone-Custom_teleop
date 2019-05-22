#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <string>
#include <sstream>

#include "depthImage.h"

using namespace cv;
using namespace std;



int main(int argc, char **argv) {

  depthImage depth_img;

  ros::init(argc, argv,"Cool");

  // Get all png in the folder
  vector<String> filenameNeg, filenamePos;
  cv::glob("../data/negative/*.png", filenameNeg);
  cv::glob("../data/positive/*.png", filenamePos);

  string pathNeg = "../data/negative/";
  string pathPos = "../data/positive/";
  string fileName = "sample_flipped_";
  string fileFormat = ".png";
  int counter = 0;

  for (size_t i=0; i<filenameNeg.size(); i++)
  {

      //Read file name and load it as CV_32FC1 in millimeters
      Mat imgRaw = imread(filenameNeg[i], -1);
      Mat imgDepthMM = Mat(imgRaw.rows, imgRaw.cols, CV_32FC1);
      depth_img.restore(imgRaw, imgDepthMM);
      flip(imgDepthMM, imgDepthMM, +1);

      //Bit shift for storage
      Mat imgStorage;
      depth_img.convert_for_storage(imgDepthMM, imgStorage);

      //Save
      counter++;
      std::ostringstream ss;
      ss << pathNeg << fileName << counter << fileFormat;
      imwrite(ss.str(), imgStorage);
  }
  counter = 0;

  for (size_t i=0; i<filenamePos.size(); i++)
  {

      //Read file name and load it as CV_32FC1 in millimeters
      Mat imgRaw = imread(filenamePos[i], -1);
      Mat imgDepthMM = Mat(imgRaw.rows, imgRaw.cols, CV_32FC1);
      depth_img.restore(imgRaw, imgDepthMM);
      flip(imgDepthMM, imgDepthMM, +1);

      //Bit shift for storage
      Mat imgStorage;
      depth_img.convert_for_storage(imgDepthMM, imgStorage);

      //Save
      counter++;
      std::ostringstream ss;
      ss << pathPos << fileName << counter << fileFormat;
      imwrite(ss.str(), imgStorage);
  }

  return 0;
}
