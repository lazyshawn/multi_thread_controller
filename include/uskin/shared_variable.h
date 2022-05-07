#pragma once

#include <vector>
#include <queue>

#include "miniros/miniros.h"

// 封装共享数据的类
class Force {
public:
  /* 传感器采集的共享数据 */
  struct Data {
    Data() {};
    int forceMat[2][8][3], timestamp[2][8];
  };
  // 复制 Config
  Force::Data copy_data(void);
  // 更新 Config
  void update(Force::Data* ForceData_);

private:
  Force::Data forceData;
  std::mutex forceMutex;
  std::condition_variable forceCond;

};

