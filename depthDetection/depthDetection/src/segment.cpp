#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>

#include <string>
#include <sstream>
#include <cmath>

#include "preProcess.h"
#include "depthImage.h"


using namespace cv;
using namespace std;

//Global variables
cv::Mat imgDepthMM, imgVis;

void draw_head_rectangles(Mat& imgIn, vector<int> head_points_x, vector<int> head_points_y, vector<float> gamma_values){
  //Input rectangles to be visualized
  //Draws all given rectangles on the given image

  for(size_t i=0; i<gamma_values.size(); i++){

    rectangle(imgIn,Point((head_points_x[i]-gamma_values[i]),head_points_y[i]),Point((head_points_x[i]+gamma_values[i]), (head_points_y[i]+(2*gamma_values [i]))),(255,255,255),1);

  }
}

int main(int argc, char **argv) {

  preProcess preProcessObject;
  depthImage depth_img;

  ros::init(argc, argv,"Cool");

  // Get all png in the folder
  vector<String> filenames, fileNamesViss;
  cv::glob("../data/raw/*.png", filenames);
  //cv::glob("../data/raw/visualized/*.png", fileNamesViss);

  string pathData = "../data/rawData/";
  string pathVis = "../data/rawData/";
  string fileNameData = "scene_Depth_";
  string fileNameVis = "scene_RGB_";
  string fileFormat = ".png";

  int counter = 0;
  int frameCount = 0;

  for (size_t i=0; i<filenames.size(); i++)
  {

      //Read file name and load it as CV_32FC1 in millimeters
      Mat imgRaw = imread(filenames[i], -1);
      //resize(imgRaw, imgRaw, Size(224, 171));
      imgDepthMM = Mat(imgRaw.rows, imgRaw.cols, CV_32FC1);
      depth_img.restore(imgRaw, imgDepthMM);;
      //depth_img.U16To32F(imgRaw, imgDepthMM);

      vector<int> points_x, points_y, depth_value_vector;
      vector<float> gamma_val;

      imgVis = imgDepthMM.clone();

      //Preprocess image
      //preProcessObject.fill_img(imgDepthMM, imgDepthMM, 1);
      //medianBlur( imgDepthMM, imgDepthMM, 3);

      //Segmentize the depth image
      preProcessObject.find_heads_and_process(imgDepthMM, points_x, points_y, gamma_val, depth_value_vector, 40, 450);

      //Visualization purposes
      //imgVis = imread(fileNamesViss[i], -1);
      //resize(imgVis, imgVis, Size(224, 171));
      imgVis = imgDepthMM.clone();
      depth_img.norm_visualize(imgVis, imgVis);
      depth_img.invert_black_white(imgVis);

      //Bit shift for storage
      Mat imgStorage;
      depth_img.convert_for_storage(imgDepthMM, imgStorage);

      //Crop all found heads and saved the cropped part of the depth image, the
      //region of interest

      for(int i = 0; i < points_x.size(); i++){
        Mat imgROIVis, imgROIData;
        preProcessObject.crop_roi(imgVis, imgROIVis, points_x[i], points_y[i], gamma_val[i]);
        preProcessObject.crop_roi(imgStorage, imgROIData, points_x[i], points_y[i], gamma_val[i]);

        //Save the visualization, used for sorting
        std::ostringstream ss, ss1;
        ss << pathVis << fileNameVis << counter << fileFormat;
        //imwrite(ss.str(), imgROIVis);

        //Save the depth image, bit shifted
        ss1 << pathData << fileNameData << counter << fileFormat;
        //imwrite(ss1.str(), imgROIData);
        counter++;

      }

      draw_head_rectangles(imgVis, points_x, points_y, gamma_val);
      frameCount++;
      resize(imgVis, imgVis, cv::Size(), 3, 3);
      imshow("boxes", imgVis);
      waitKey(0);


  }

  return 0;
}
