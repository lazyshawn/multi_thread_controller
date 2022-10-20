#pragma once

#include "miniros/miniros.h"
#include "dydw/dydw_driver.h"

typedef std::vector<double> DydwData;

class DydwShared {
public:
  std::vector<double> F0 = {0.500275, 32.98784, -0.13148458, 30.568432};
  std::vector<double> G = {32.91205, 30.928371};
  std::vector<double> copy_data();
  void update_data(std::vector<double> dydwData);

private:
  // 线程数据
  std::vector<double> data = std::vector<double>(6, 0);
  std::mutex dydwMutex;
  std::condition_variable dydwCond;
};

