#include  "preProcess.h"

/*
preProcess::preProcess(){
}

preProcess::~preProcess(){
}
*/

bool preProcess::fill_img(Mat& imgIn, Mat& imgOut, int win_size){



  imgOut = imgIn.clone();
  for(int y = 0; y < imgIn.rows; y++){
      for(int x = 0; x < imgIn.cols; x++){
        //https://www.utdallas.edu/~cxc123730/SPIE2016.pdf

          float val = imgIn.at<float>(y,x);
          vector<float> pixel_values;

          //cout<<"Max value: "<<max<<endl;

          if(val == 0){


            if((x+win_size <= imgIn.cols) && (x- win_size >= 0)){

              if((y+win_size <= imgIn.rows) && (y - win_size >= 0)){

                for(int yy = y-win_size; yy < y+1+win_size; yy++){
                    for(int xx = x-win_size; xx < x+1+win_size; xx++){

                      pixel_values.push_back(imgIn.at<float>(yy,xx));

                    }
                  }

                  float max = *max_element(pixel_values.begin(), pixel_values.end());
                  if(max > 0){

                    imgOut.at<float>(y,x) = max;
                  }
                  else{
                    imgOut.at<float>(y,x) = 0;

                  }

              }
            }

          }
      }
  }
}


bool preProcess::find_heads(Mat& imgIn, vector<int>& head_points_x, vector<int>& head_points_y, vector<float>& gamma_values, vector<int>&depth_value_vec, float min_depth, float max_depth){
  //Input 32FC1 where every pixel is distance in mm
  //Output: coordinates for head location
  //Finds the point location of a human head in image. Will produce lots of
  //false positives, requires processing afterwards

  float k_camera_parameter = 300; //Camera intrinsic parameter To do
  float head_radius = 11; //real world head head_radius in cm.

  //Scan through the image row by row
  for (int y = 1; y < imgIn.rows; y++) {
      for (int x = 0; x < imgIn.cols; x++){

        float depth_val = imgIn.at<float>(y,x) * 0.1;
        if((depth_val > min_depth) && (depth_val < max_depth)){
          float gamma = (k_camera_parameter / depth_val) * head_radius;


          if((y+2*gamma) < imgIn.rows){
            if ((x > int(gamma)) && ((int(gamma) + x) < imgIn.cols))
            {
              bool head_point_bool = true;
              for (int xc = x - int(gamma); xc < x + int(gamma); xc++){

                float row_val = abs((imgIn.at<float>(y-1, xc)*0.1) - depth_val);
/*
                if(imgIn.at<float>(y-1, xc) == 0){
                  head_point_bool = false;
                }
*/
                if(row_val < head_radius){
                  head_point_bool = false;
                }
              }

              if(head_point_bool == true){

                head_points_x.push_back(x);
                head_points_y.push_back(y);
                gamma_values.push_back(gamma);
                depth_value_vec.push_back(depth_val);

              }
            }
          }
        }
      }
    }
    return true;
}


bool preProcess::process_head_points_gamma_thres(vector<int>& head_points_x, vector<int>& head_points_y, vector<float>& gamma_values, vector<int>&depth_value_vec, int gam_threshold_min, int gam_threshold_max){

  if(head_points_x.size()== 0){
    return false;
  }
  int iterations = gamma_values.size()-1;
  vector<int> heads_for_deletion;
  //Remove boxes with a gamma value lower than the given threshold

  for(size_t i=0; i<iterations; i++){
    int gam_value_int = gamma_values[i];
    if(gam_value_int < gam_threshold_min){
        heads_for_deletion.push_back(i);
    }
    else if(gam_value_int > gam_threshold_max){
      heads_for_deletion.push_back(i);
    }

    for(size_t j = 0; j < heads_for_deletion.size(); j++){
      if(heads_for_deletion[j] < head_points_x.size()){
        head_points_x.erase(head_points_x.begin()+heads_for_deletion[j]);
        head_points_y.erase(head_points_y.begin()+heads_for_deletion[j]);
        gamma_values.erase(gamma_values.begin()+heads_for_deletion[j]);
        depth_value_vec.erase(depth_value_vec.begin()+heads_for_deletion[j]);
        i--;

      }
    }

    iterations = iterations - (heads_for_deletion.size()-1);
    heads_for_deletion.clear();
  }
  return true;
}


bool preProcess::process_head_points_overlapping(vector<int>& head_points_x, vector<int>& head_points_y, vector<float>& gamma_values, vector<int>&depth_value_vec){

  if(head_points_x.size()== 0){
    return false;
  }

  int k = 0;
  int countHeads = 0;
  int iterations = gamma_values.size();
  vector<int> heads_for_deletion;

  for(size_t i=0; i<iterations; i++){
    for(size_t j = 0; j < iterations; j++){

      int widthI = gamma_values[i]*2;
      int heightI = widthI;

      int widthJ = gamma_values[j]*2;
      int heightJ = widthI;

      Rect iRect(head_points_x[i]-gamma_values[i], head_points_y[i], widthI, heightI);
      Rect jRect(head_points_x[j]-gamma_values[j], head_points_y[j], widthJ, heightJ);
      bool intersect = ((iRect & jRect).area() > 0);
      int depth_val_difference = abs(depth_value_vec[i]-depth_value_vec[j]);

      if(i != j){
        if((intersect == true) && (depth_val_difference < 30)){
          heads_for_deletion.push_back(j);

        }
      }


    }

    for(size_t j = heads_for_deletion.size()-1; j < heads_for_deletion.size(); j--){
      if(heads_for_deletion[j] < head_points_x.size()){
        head_points_x.erase(head_points_x.begin()+heads_for_deletion[j]);
        head_points_y.erase(head_points_y.begin()+heads_for_deletion[j]);
        gamma_values.erase(gamma_values.begin()+heads_for_deletion[j]);
        depth_value_vec.erase(depth_value_vec.begin()+heads_for_deletion[j]);

      }
    }

    iterations = iterations - heads_for_deletion.size();
    heads_for_deletion.clear();


  }
  return true;

}

bool preProcess::process_head_points_adjacent_x(vector<int>& head_points_x, vector<int>& head_points_y, vector<float>& gamma_values, vector<int>&depth_value_vec){
//Output removes the head locations in the adjacent x columns.

  if(head_points_x.size()== 0){
    return false;
  }

  int iterations = gamma_values.size()-1;
  vector<int> heads_for_deletion, x_location_adjacent;

  //Remove row adjacent boxes

  for(size_t i=0; i<iterations; i++){
    //Find adjacent x
    x_location_adjacent.push_back(i);
    for(size_t j = i+1; j < iterations; j++){

      if(head_points_y[i] == head_points_y[j]){

          if((head_points_x[i]) == head_points_x[j]){
            x_location_adjacent.push_back(j);
          }
      }
    }

    //Mark everything except for middle x for deletion
    x_location_adjacent.erase(x_location_adjacent.begin() + int(x_location_adjacent.size()/2));
    for(size_t j = 0; j < x_location_adjacent.size(); j++){
          heads_for_deletion.push_back(x_location_adjacent[j]);
    }

    for(size_t j = heads_for_deletion.size()-1; j < heads_for_deletion.size(); j--){
      if(heads_for_deletion[j] < head_points_x.size()){
        head_points_x.erase(head_points_x.begin()+heads_for_deletion[j]);
        head_points_y.erase(head_points_y.begin()+heads_for_deletion[j]);
        gamma_values.erase(gamma_values.begin()+heads_for_deletion[j]);
        depth_value_vec.erase(depth_value_vec.begin()+heads_for_deletion[j]);
        i--;
      }
    }

    //Prepare the variables for another loop
    iterations = iterations - heads_for_deletion.size();
    heads_for_deletion.clear();
  }
  return true;
}

bool preProcess::process_head_points_adjacent_y(vector<int>& head_points_x, vector<int>& head_points_y, vector<float>& gamma_values, vector<int>&depth_value_vec){
  if(head_points_x.size()== 0){
    return false;
  }

  int iterations = gamma_values.size()-1;
  vector<int> heads_for_deletion;

  //Remove row adjacent boxes

  for(size_t i=0; i<iterations; i++){

    for(size_t j = i+1; j < iterations; j++){
      if(head_points_y[i] + 3 > head_points_y[j]){
        if((head_points_x[i]+gamma_values[i]) > head_points_x[j]){
          heads_for_deletion.push_back(j);
        }
      }
      else if(head_points_y[i] - 3 > head_points_y[j]){
        if((head_points_x[i]+gamma_values[i]) > head_points_x[j]){
          heads_for_deletion.push_back(j);
        }
      }
    }

    for(size_t j = heads_for_deletion.size()-1; j < heads_for_deletion.size(); j--){
      if(heads_for_deletion[j] < head_points_x.size()){
        head_points_x.erase(head_points_x.begin()+heads_for_deletion[j]);
        head_points_y.erase(head_points_y.begin()+heads_for_deletion[j]);
        gamma_values.erase(gamma_values.begin()+heads_for_deletion[j]);
        depth_value_vec.erase(depth_value_vec.begin()+heads_for_deletion[j]);
        i--;
      }
    }


    iterations = iterations - heads_for_deletion.size();
    heads_for_deletion.clear();
  }
  return true;
}

bool preProcess::find_heads_and_process(Mat& imgIn, vector<int>& head_points_x, vector<int>& head_points_y, vector<float>& gamma_values, vector<int>&depth_value_vec, float min_depth, float max_depth){

  find_heads(imgIn, head_points_x, head_points_y, gamma_values, depth_value_vec, 80, 350);
  process_head_points_adjacent_x(head_points_x, head_points_y, gamma_values, depth_value_vec);
  process_head_points_adjacent_y(head_points_x, head_points_y, gamma_values, depth_value_vec);
  process_head_points_overlapping(head_points_x, head_points_y, gamma_values, depth_value_vec);

  return true;
}

bool preProcess::crop_roi(Mat imgIn, Mat& imgOut, int head_points_x, int head_points_y, float gamma_values){

    Rect roi = Rect((head_points_x-gamma_values), head_points_y, (2*gamma_values), (2*gamma_values));
    Mat temp (imgIn, roi);
    imgOut = temp.clone();
    resize(imgOut, imgOut, Size(60, 60));
    //ROS_INFO("i: %i, w: %i, h: %i", roi.width, roi.height);

}
