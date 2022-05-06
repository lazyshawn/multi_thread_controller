#ifndef SHARED_VARIABLE_H
#define SHARED_VARIABLE_H

#include <vector>
#include <queue>

#include "miniros/thread_manager.h"

typedef std::vector<double> THETA;

class PathQueue {
private:
  std::queue<THETA> data;
  mutable std::mutex pathMutex;
  std::condition_variable pathCond;

public:
  PathQueue();
  void push(THETA path);
  bool pop(THETA& path);
  bool empty() const;
  bool clean();
};

#endif
