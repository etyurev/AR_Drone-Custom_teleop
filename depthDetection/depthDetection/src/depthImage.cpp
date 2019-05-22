#include  "depthImage.h"

void depthImage::norm_visualize(Mat& imgIn, Mat& imgOut){
  //Input: CV_32FC1 in mm.
  //Output: CV_8UC1 in pixels between 0-255
  //Divides every value with 7 m. Therefore 255 equals 7 meters

  for(int i = 0; i < imgIn.cols; i++){
     for(int j = 0; j < imgIn.rows; j++){
         imgOut.at<float>(j,i) = imgIn.at<float>(j,i)/7000;
     }
   }
   imgOut.convertTo(imgOut, CV_8UC1, 255);
 }

 void depthImage::invert_black_white(Mat& imgIn){
   for (int x = 0; x < imgIn.cols; x++) {
       for (int y = 0; y < imgIn.rows; y++){

          if (imgIn.at<uchar>(y,x) > 0){
            imgIn.at<uchar>(y,x) = 255 - imgIn.at<uchar>(y,x);
            }
       }
     }
 }

 void depthImage::convert_for_storage(Mat imgIn, Mat& imgOut){

   imgOut = cv::Mat(imgIn.rows, imgIn.cols, CV_8UC3);
   for(int i = 0; i < imgOut.cols; i++){
      for(int j = 0; j < imgOut.rows; j++){
          float orig = imgIn.at<float>(j,i);

          uint32_t orig_int = int(orig);

          imgOut.at<Vec3b>(j,i)[0] = (uint8_t)((orig_int&0xFF0000) >> 16);
          imgOut.at<Vec3b>(j,i)[1] = (uint8_t)((orig_int&0x00FF00) >> 8);
          imgOut.at<Vec3b>(j,i)[2] = (uint8_t)((orig_int&0x0000FF) >> 0);

      }
  }
 }


 void depthImage::restore(Mat& imgIn, Mat& imgOut){
   //Input: CV_8UC3 stored with bits in different vectors
   //Output: CV_32FC1 each pixel equals distance in mm

   for(int i = 0; i < imgIn.cols; i++){
      for(int j = 0; j < imgIn.rows; j++){

          uint64_t orig_int = 0;
          orig_int = imgIn.at<Vec3b>(j,i)[0] << 16;
          orig_int = orig_int+imgIn.at<Vec3b>(j,i)[1] << 8;
          orig_int = orig_int+imgIn.at<Vec3b>(j,i)[2] << 0;

          imgOut.at<float>(j,i) = orig_int;
      }
    }
  }

  void depthImage::U16To32F(Mat& imgIn, Mat& imgOut){
    //Input: CV_16UC1 stored with bits in different vectors
    //Output: CV_32FC1 each pixel equals distance in mm

    for(int i = 0; i < imgIn.cols; i++){
       for(int j = 0; j < imgIn.rows; j++){
           uint16_t orig_int = imgIn.at<uint16_t>(j,i);
           imgOut.at<float>(j,i) = orig_int;
       }
     }
   }

  void depthImage::copy_to_CV8UC3(Mat& imgIn, Mat& imgOut){
    //Input: CV_8UC1
    //Output: Copies the grey vector to CV_8UC3 format

    for(int i = 0; i < imgIn.cols; i++){
       for(int j = 0; j < imgIn.rows; j++){
           imgOut.at<Vec3b>(j,i)[0] = imgIn.at<uchar>(j,i);
           imgOut.at<Vec3b>(j,i)[1] = imgIn.at<uchar>(j,i);
           imgOut.at<Vec3b>(j,i)[2] = imgIn.at<uchar>(j,i);
       }
     }
   }
