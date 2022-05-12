#include "wsg_controller/shared_variable.h"

/* 添加后续路径 */
void WSGConfig::push(WSGCMD wsgcmd) {
  std::scoped_lock lock(wsgMutex);
  data.emplace(std::move(wsgcmd));
  // wsgcmdCond.notify_one();
}

bool WSGConfig::pop(WSGCMD& wsgcmd) {
  std::scoped_lock lock(wsgMutex);
  if (data.empty()) return false;
  wsgcmd = std::move(data.front());
  data.pop();
  return true;
}

/* 设定新路径 */
void WSGConfig::set(WSGCMD wsgcmd) {
  WSGConfig::clean();
  WSGConfig::push(wsgcmd);
}

bool WSGConfig::empty() const {
  return data.empty();
}

bool WSGConfig::clean() {
  if (!data.empty()) return false;
  std::scoped_lock lock(wsgMutex);
  std::queue<WSGCMD> empty;
  swap(empty, data);
  return true;
}

void WSGConfig::update_state(double position) {
  std::scoped_lock lock(wsgMutex);
  pos = position;
}

double WSGConfig::get_state() {
  std::scoped_lock lock(wsgMutex);
  return pos;
}

