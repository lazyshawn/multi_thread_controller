#pragma once

#include "miniros/miniros.h"
#include <eigen3/Eigen/Dense>

typedef Eigen::Matrix<double,4,4> Mat4d;

class ObjState {
public:
  struct Data {
    Mat4d obj2elk;
  };
  Data get_data(void);
  void update(Data* Data_);
  void set_check_marker();
  void reset_check_marker();
  bool check_marker();
  Mat4d get_marker();

private:
  Data data;
  std::atomic<bool> checkMark{false};
  std::mutex cameraMutex;
  std::condition_variable cameraCond;
};

