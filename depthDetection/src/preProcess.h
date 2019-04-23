#ifndef PREPROCESS_H
#define PREPROCESS_H

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

class preProcess
{
private:
  int _head_radius;

public:

  bool fill_img(Mat& imgIn, Mat& imgOut, int win_size);
  bool find_heads(Mat& imgIn, vector<int>& head_points_x, vector<int>& head_points_y, vector<float>& gamma_values, vector<int>&depth_value_vec, float min_depth, float max_depth);
  bool find_heads_and_process(Mat& imgIn, vector<int>& head_points_x, vector<int>& head_points_y, vector<float>& gamma_values, vector<int>&depth_value_vec, float min_depth, float max_depth);
  bool process_head_points_gamma_thres(vector<int>& head_points_x, vector<int>& head_points_y, vector<float>& gamma_values, vector<int>&depth_value_vec, int gam_threshold_min, int gam_threshold_max);
  bool process_head_points_overlapping(vector<int>& head_points_x, vector<int>& head_points_y, vector<float>& gamma_values, vector<int>&depth_value_vec);
  bool process_head_points_adjacent_x(vector<int>& head_points_x, vector<int>& head_points_y, vector<float>& gamma_values, vector<int>&depth_value_vec);
  bool process_head_points_adjacent_y(vector<int>& head_points_x, vector<int>& head_points_y, vector<float>& gamma_values, vector<int>&depth_value_vec);
  bool crop_roi(Mat imgIn, Mat& imgOut, int head_points_x, int head_points_y, float gamma_values);
};


#endif
