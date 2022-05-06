#pragma once

#include <vector>
#include <queue>

#include "miniros/thread_manager.h"

typedef std::vector<double> WSGCMD;

class WSGConfig {
private:
  // pos + vel
  std::queue<WSGCMD> data;
  double pos, vel;
  std::mutex wsgMutex;
  std::condition_variable wsgCond;

public:
  WSGConfig() : vel(20) {};

  void push(WSGCMD wsgcmd);
  bool pop(WSGCMD& wsgcmd);
  void set(WSGCMD wsgcmd);
  bool empty() const;
  bool clean();
};

