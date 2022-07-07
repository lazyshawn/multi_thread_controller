#include "ur5e_controller/shared_variable.h"

UrConfig::UrConfig() : isReady(false){
}

/* **************** 判断机械臂是否已经启动 **************** */
void UrConfig::set_ready() {
  isReady = true;
}

bool UrConfig::is_ready() {
  return isReady;
}

/* 添加后续路径 */
void UrConfig::push(THETA joint) {
  std::scoped_lock lock(urMutex);
  path.emplace(std::move(joint));
  // urCond.notify_one();
}

bool UrConfig::pop(THETA& refJoint) {
  std::scoped_lock lock(urMutex);
  if (path.empty()) return false;
  refJoint = std::move(path.front());
  path.pop();
  return true;
}

bool UrConfig::empty() const {
  return path.empty();
}

bool UrConfig::clean() {
  if (!path.empty()) {
    std::cout << "Path is still incomplete: "<< path.size() << std::endl;
    return false;
  }
  std::scoped_lock lock(urMutex);
  std::queue<THETA> empty;
  swap(empty, path);
  return true;
}

THETA UrConfig::get_state() {
  std::scoped_lock guard(urMutex);
  return jointState;
}

void UrConfig::update_state(THETA jointState_){
  std::scoped_lock guard(urMutex);
  jointState = jointState_;
}

