#pragma  once

#include <queue>
#include "miniros/miniros.h"
#include "ur5e_kinematics.h"

class Ur5eShared {
private:
  bool isReady;
  std::queue<THETA> path;
  THETA jointState;
  std::mutex urMutex;
  std::condition_variable urCond;

public:
  Ur5eShared();

  void set_ready();
  bool is_ready();
  THETA copy_data();
  void update_data(THETA jointState_);
  void push(THETA refJoint);
  bool pop(THETA& refJoint);
  bool empty() const;
  bool clean();
};

typedef THETA Ur5eData;

