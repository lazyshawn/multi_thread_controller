#include "ur5e_controller/shared_variable.h"

Ur5eShared::Ur5eShared() : isReady(false){
}

/* **************** 判断机械臂是否已经启动 **************** */
void Ur5eShared::set_ready() {
  isReady = true;
}

bool Ur5eShared::is_ready() {
  return isReady;
}

/* 添加后续路径 */
void Ur5eShared::push(THETA joint) {
  std::scoped_lock lock(urMutex);
  path.emplace(std::move(joint));
  // urCond.notify_one();
}

bool Ur5eShared::pop(THETA& refJoint) {
  std::scoped_lock lock(urMutex);
  if (path.empty()) return false;
  refJoint = std::move(path.front());
  path.pop();
  return true;
}

bool Ur5eShared::empty() const {
  return path.empty();
}

bool Ur5eShared::clean() {
  if (!path.empty()) {
    std::cout << "Path is still incomplete: "<< path.size() << std::endl;
    return false;
  }
  std::scoped_lock lock(urMutex);
  std::queue<THETA> empty;
  swap(empty, path);
  return true;
}

THETA Ur5eShared::copy_data() {
  std::scoped_lock guard(urMutex);
  return jointState;
}

void Ur5eShared::update_data(THETA jointState_){
  std::scoped_lock guard(urMutex);
  jointState = jointState_;
}

