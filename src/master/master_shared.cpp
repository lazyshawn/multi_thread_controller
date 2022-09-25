#include "master/master_shared.h"

/* 添加后续路径 */
void HfvcShared::push_queue(HfvcQueue joint) {
  std::scoped_lock lock(hfvcMutex);
  queue.emplace(std::move(joint));
  // hfvcCond.notify_one();
}

bool HfvcShared::pop_queue(HfvcQueue& refJoint) {
  std::scoped_lock lock(hfvcMutex);
  if (queue.empty()) return false;
  refJoint = std::move(queue.front());
  queue.pop();
  return true;
}

bool HfvcShared::empty() const {
  return queue.empty();
}

bool HfvcShared::clean() {
  if (!queue.empty()) {
    std::cout << "Path is still incomplete: "<< queue.size() << std::endl;
    return false;
  }
  std::scoped_lock lock(hfvcMutex);
  std::queue<HfvcQueue> empty;
  swap(empty, queue);
  return true;
}

HfvcData HfvcShared::copy_data() {
  std::scoped_lock guard(hfvcMutex);
  return data;
}

void HfvcShared::update_data(HfvcData hfvcData){
  std::scoped_lock guard(hfvcMutex);
  data = hfvcData;
}

