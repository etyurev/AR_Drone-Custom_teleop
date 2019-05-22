#ifndef DEPTHIMAGE_H
#define DEPTHIMAGE_H

#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <string>
#include <sstream>
#include <cmath>


using namespace cv;
using namespace std;

class depthImage
{
private:


public:

  void norm_visualize(Mat& imgIn, Mat& imgOut);
  void invert_black_white(Mat& imgIn);
  void convert_for_storage(Mat imgIn, Mat& imgOut);
  void restore(Mat& imgIn, Mat& imgOut);
  void copy_to_CV8UC3(Mat& imgIn, Mat& imgOut);
  void U16To32F(Mat& imgIn, Mat& imgOut);


};


#endif
