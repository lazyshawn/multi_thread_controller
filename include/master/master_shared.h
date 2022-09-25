#pragma once

#include "miniros/miniros.h"
#include <queue>

typedef std::vector<double> HfvcData, HfvcConfig, HfvcQueue;

class HfvcShared {
public:
  // Data
  std::vector<double> copy_data();
  void update_data(HfvcData hfvcData);
  // Queue
  void push_queue(HfvcConfig hfvcConfig);
  bool pop_queue(HfvcConfig& hfvcConfig);
  bool empty () const;
  bool clean();

private:
  // 线程数据
  timespec ts;
  std::vector<double> data;
  std::vector<double> config;
  std::queue<std::vector<double>> queue;
  // 数据保护
  std::mutex hfvcMutex;
  std::condition_variable hfvcCond;
};

