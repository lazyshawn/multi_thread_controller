#pragma once

#include <vector>
#include <queue>
#include "miniros/miniros.h"

typedef std::vector<double> WSGCMD;

class WSGConfig {
private:
  bool isReady;
  // pos(mm) + vel(mm/s)
  std::queue<WSGCMD> data;
  double pos, vel;
  std::mutex wsgMutex;
  std::condition_variable wsgCond;

public:
  WSGConfig() : vel(20) {};

  void set_ready();
  bool is_ready();
  void push(WSGCMD wsgcmd);
  bool pop(WSGCMD& wsgcmd);
  void set(WSGCMD wsgcmd);
  bool empty() const;
  bool clean();
  void update_state(double position);
  double get_state();
};

