#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ml.hpp>

//https://github.com/kuntoro-adi/opencv-HOG/blob/master/HOG.h

#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

void computeMagAngle(InputArray src, OutputArray mag, OutputArray ang);

void computeHOG(InputArray mag, InputArray ang, OutputArray dst, int dims, bool isWeighted);
