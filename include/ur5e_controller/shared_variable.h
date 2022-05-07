#pragma  once

#include <queue>
#include "miniros/miniros.h"
#include "ur5e_kinematics.h"

class UrConfig {
private:
  std::queue<THETA> path;
  THETA jointState;
  std::mutex urMutex;
  std::condition_variable urCond;

public:
  UrConfig();

  THETA get_state();
  void update_state(THETA jointState_);
  void push(THETA refJoint);
  bool pop(THETA& refJoint);
  bool empty() const;
  bool clean();
};

