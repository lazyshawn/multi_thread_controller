#pragma once

#include "miniros/miniros.h"
#include "dydw/dydw_driver.h"

typedef std::vector<double> DydwData;

class DydwShared {
public:
  std::vector<double> copy_data();
  void update_data(std::vector<double> dydwData);

private:
  // 线程数据
  std::vector<double> data = std::vector<double>(6, 0);
  std::mutex dydwMutex;
  std::condition_variable dydwCond;
};

