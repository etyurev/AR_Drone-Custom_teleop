#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/ml.hpp>

#include "sensor_msgs/Image.h"

#include <string>
#include <sstream>
#include <vector>
#include <chrono>
#include <cmath>

#include "depthImage.h"


using namespace cv;
using namespace cv::ml;
using namespace std;
using namespace std::chrono;

bool fill_Mat_float(Mat imgIn, Mat& imgOut, vector<int> index, float data_size, float kfold){

  for(int y = 0; y < imgOut.rows; y++){

      for(int x = 0; x <  imgOut.cols; x++){

        int index_pos = y + int(index.size() * ((kfold*data_size)));
        imgOut.at<float>(y,x) = imgIn.at<float>(index[index_pos],x);
      }
    }
    return(1);
}

bool fill_Mat_short_int(Mat imgIn, Mat& imgOut, vector<int> index, float data_size, float kfold){

  for(int y = 0; y < imgOut.rows; y++){

      for(int x = 0; x <  imgOut.cols; x++){

        int index_pos = y + int(index.size() * ((kfold*data_size)));
        imgOut.at<int>(y,x) = imgIn.at<int>(index[index_pos],x);
      }
    }
    return(1);
}

int main(){

  depthImage depth_img;

  //Find the size of the hog feature vector
  Mat imgSetupRaw = imread("../data/positive/data9.png");
  Mat imgSetupMM = Mat(imgSetupRaw.rows, imgSetupRaw.cols, CV_32FC1);
  depth_img.restore(imgSetupRaw, imgSetupMM);

  Mat imgVis = imgSetupMM.clone();

  depth_img.norm_visualize(imgVis, imgVis);
  depth_img.invert_black_white(imgVis);

  HOGDescriptor hog( Size(60,60), Size(20,20), Size(10,10), Size(10,10), 9);
  vector<float> ders;
  vector<Point> locs;

  hog.compute(imgVis, ders, Size(10,10), Size(0, 0), locs);
  ROS_INFO("Number of features per sample: %i", ders.size());
  ROS_INFO("Extracting features for positive samples.");


  //Read samples
  vector<String> filenames_positive, filenames_negative;
  cv::glob("../data/positive/*.png", filenames_positive);
  cv::glob("../data/negative/*.png", filenames_negative);

  int totalSamples = filenames_negative.size() + filenames_positive.size();
  Mat labels = Mat(totalSamples, 1, CV_32SC1);
  Mat features = Mat(totalSamples, ders.size(), CV_32FC1);

  //Compute features for all positive samples
  for (size_t i=0; i<filenames_positive.size(); i++)
  {
    //Read sample from storage
    Mat imgRaw = imread(filenames_positive[i]);
    Mat imgMM = Mat(imgRaw.rows, imgRaw.cols, CV_32FC1);
    depth_img.restore(imgRaw, imgMM);

    Mat imgVis = imgMM.clone();

    depth_img.norm_visualize(imgVis, imgVis);
    depth_img.invert_black_white(imgVis);

    //Attach positive labels
    labels.at<int>(i,0) = 1;

    //Extract features
    HOGDescriptor hog( Size(60,60), Size(20,20), Size(10,10), Size(10,10), 9);
    vector<float> hogFeatures;
    vector<Point> locations;

    hog.compute(imgVis, hogFeatures, Size(10,10), Size(0, 0), locations);

    //Append features
    for(int j=0;j<ders.size();j++){
      features.at<float>(i,j)=hogFeatures.at(j);
      }

  }

  ROS_INFO("Extracting features for negative samples.");

  for (size_t i=0; i<filenames_negative.size()-1; i++)
  {

    //Read sample from storage
    Mat imgRaw = imread(filenames_negative[i]);
    Mat imgMM = Mat(imgRaw.rows, imgRaw.cols, CV_32FC1);
    depth_img.restore(imgRaw, imgMM);

    Mat imgVis = imgMM.clone();

    depth_img.norm_visualize(imgVis, imgVis);
    depth_img.invert_black_white(imgVis);

    //Attach negative labels
    int label_pos = filenames_positive.size() + i;
    labels.at<int>(label_pos,1) = -1;

    //Extract features
    HOGDescriptor hog( Size(60,60), Size(20,20), Size(10,10), Size(10,10), 9);
    vector<float> hogFeatures;
    vector<Point> locations;

    hog.compute(imgVis, hogFeatures, Size(10,10), Size(0, 0), locations);

    //Append feature
    for(int j=0;j<ders.size();j++){
      features.at<float>(i+filenames_positive.size(),j)=hogFeatures.at(j);
      }
  }

  ROS_INFO("Hyper parameter optimization.");

  float k_fold = 5;
  float data_size = 0.8;
  long long int c_log = 0;
  int count_positive = 0;
  int count_negative = 0;
  int response;
  float accuracy = 0;
  int index_high = 0;

  vector<int> c_list;
  vector<float> accuracy_list;


  for(int c_int = 1; c_int <= 1000; c_int++){

    ROS_INFO("%i", c_int);

    Mat feature_train = Mat(features.rows*data_size, features.cols, CV_32FC1);
    Mat feature_test = Mat(features.rows*(1-data_size), features.cols, CV_32FC1);
    Mat labels_train = Mat(labels.rows*data_size, 1, CV_32SC1);
    Mat labels_test = Mat(labels.rows*(1-data_size), 1, CV_32SC1);

    vector<int> indexes;
    indexes.reserve(features.rows);

    for (int i = 0; i < features.rows; i++){
      indexes.push_back(i);
    }

    srand(unsigned(time(NULL))); //A random seed for every shuffle
    random_shuffle(indexes.begin(), indexes.end());

    fill_Mat_float(features, feature_train, indexes, data_size, 0);
    fill_Mat_float(features, feature_test, indexes, data_size, 1);
    fill_Mat_short_int(labels, labels_train, indexes, data_size, 0);
    fill_Mat_short_int(labels, labels_test, indexes, data_size, 1);

    //train and save the classifier
    Ptr<SVM> svm = SVM::create();
      svm->setType(SVM::C_SVC);
      svm->setKernel(SVM::LINEAR);
      //Ptr<ml::TrainData> td = ml::TrainData::create(feature_train, ROW_SAMPLE, labels_train);
      svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, 100, 1e-6));
      c_log = c_int;
      svm->setC(c_int);
      svm->train(feature_train, ROW_SAMPLE, labels_train);


      for (int i = 0; i < labels_test.rows; i++){
        Mat hog_feat = Mat(1, feature_test.cols, CV_32FC1);
        for(int j = 0; j < feature_test.cols; j++){
          hog_feat.at<float>(0,j)=feature_test.at<float>(i,j);
        }
        response = svm->predict(hog_feat);
        //ROS_INFO("predicted: %f", response);
        //ROS_INFO("was: %i", labels_test.at<int>(i,0));
        if (response == labels_test.at<int>(i,0)){
          count_positive++;
        }
        else{
          count_negative++;
        }
      }
      accuracy = 1 - (float(count_negative) / float(count_positive));

      accuracy_list.push_back(accuracy);
      c_list.push_back(c_int);
      if (accuracy > accuracy_list[index_high]){
        index_high = c_int-1;
      }

    }

    ROS_INFO("Correct: %i", count_positive);
    ROS_INFO("Incorrect: %i", count_negative);
    ROS_INFO("C int: %i", c_list[index_high]);

    ROS_INFO("Accuracy: %0.2f", accuracy_list[index_high]);

    Ptr<SVM> svm = SVM::create();
    svm->setType(SVM::C_SVC);
    svm->setKernel(SVM::LINEAR);
    svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, 100, 1e-6));
    svm->setC(c_list[index_high]);
    svm->train(features, ROW_SAMPLE, labels);
    svm->save("classifier_no_preprocess.svm");

    /*
    vector<String> filenames_test;

    //Test classifier on raw data
    cv::glob("../data/rawData/*.png", filenames_test);
    int count_pos = 0;
    for (size_t i=0; i<filenames_test.size(); i++)
    {
      Mat imgRaw = imread(filenames_test[i]);
      Mat imgMM = Mat(imgRaw.rows, imgRaw.cols, CV_32FC1);
      depth_img.restore(imgRaw, imgMM);

      Mat imgVis = imgMM.clone();

      depth_img.norm_visualize(imgVis, imgVis);
      depth_img.invert_black_white(imgVis);

      //Extract features

      HOGDescriptor hog( Size(60,60), Size(20,20), Size(10,10), Size(10,10), 9);
      vector<float> hogFeatures;
      vector<Point> locations;

      hog.compute(imgVis, hogFeatures, Size(10,10), Size(0, 0), locations);
        Mat Hogfeat = Mat(1, hogFeatures.size(), CV_32FC1);

      for(int j=0;j<hogFeatures.size();j++){
        Hogfeat.at<float>(0,j)=hogFeatures.at(j);
        }
        float response = svm->predict(Hogfeat);
        if(response == 1){
          count_pos++;
          ROS_INFO("humans found: %i", count_pos);
          imshow("human", imgVis);
          waitKey(0);
        }

    }
    */

  return 0;
}
