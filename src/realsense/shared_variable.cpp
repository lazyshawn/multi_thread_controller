#include "realsense/shared_variable.h"

ObjState::Data ObjState::get_data(void) {
  std::scoped_lock guard(cameraMutex);
  return data;
}

void ObjState::update(Data* Data_) {
  std::scoped_lock guard(cameraMutex);
  data = *Data_;
  cameraCond.notify_all();
}

void ObjState::set_check_marker() {
  checkMark = true;
}
void ObjState::reset_check_marker() {
  std::scoped_lock guard(cameraMutex);
  checkMark = false;
  cameraCond.notify_all();
}
bool ObjState::check_marker() {
  return checkMark;
}
Mat4d ObjState::get_marker() {
  std::unique_lock lock(cameraMutex);
  set_check_marker();
  cameraCond.wait(lock);
  return data.obj2elk;
}

