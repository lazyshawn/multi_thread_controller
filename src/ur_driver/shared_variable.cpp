#include "ur_driver/shared_variable.h"

PathQueue pathqueue;

PathQueue::PathQueue() {}

// 添加新路径
void PathQueue::push(THETA path) {
  std::scoped_lock lock(pathMutex);
  data.emplace(std::move(path));
  // pathCond.notify_one();
}

// 尝试弹出路径
bool PathQueue::pop(THETA& path) {
  std::scoped_lock lock(pathMutex);
  if (data.empty()) return false;
  path = std::move(data.front());
  data.pop();
  return true;
}

bool PathQueue::empty() const {
  return data.empty() ? true : false;
}

bool PathQueue::clean() {
  if (!data.empty()) return false;
  std::queue<THETA> empty;
  swap(empty, data);
  return true;
}

