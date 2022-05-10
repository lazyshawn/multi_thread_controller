#pragma once

#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/hpp/rs_sensor.hpp>
#include <librealsense2/hpp/rs_types.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
// #include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <map>

class Camera {
private:
  std::vector<std::string> serials;
  std::map<std::string, int> devMap;
  rs2::frameset fs;
  std::vector<rs2::frame> newFrames;
  // 相机内参
  std::vector<rs2_intrinsics> intrPara;
  // 相机外参数矩阵
  std::vector<Eigen::Matrix<double, 4, 4>> extrMat;

public:
  int devNum;
  rs2::context ctx;
  rs2::pipeline pipe;
  std::vector<rs2::pipeline> pipelines;

  Camera();
  Camera(std::string serialNum);
  ~Camera();

  void active(std::vector<std::string> devSerials);
  void auto_active();
  int get_dev_index(std::string serial);
  void get_frameset();
  cv::Mat get_frame(int devIndex);
  cv::VideoWriter create_recorder(std::string videoSavePath);
  void set_extrMat(int devIndex, Eigen::Matrix<double, 4, 4> extr);
};
