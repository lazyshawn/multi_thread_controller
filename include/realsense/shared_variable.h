#pragma once

#include "miniros/thread_manager.h"
#include "camera_driver.h"

typedef Eigen::Matrix<double,4,4> Mat4d;

class ObjState {
public:
  struct Data {
    Mat4d obj2elk;
    bool flag = false;
    Eigen::Matrix<double,4,4> markerPose;
  };
  Data get_data(void);
  void update(Data* Data_);

private:
  Data data;
  std::mutex cameraMutex;
  std::condition_variable cameraCond;
};

