#include "realsense/shared_variable.h"

ObjState::Data ObjState::get_data(void) {
  std::unique_lock lock(cameraMutex);
  cameraCond.wait(lock);
  return data;
}

void ObjState::update(Data* Data_) {
  std::scoped_lock guard(cameraMutex);
  data = *Data_;
  cameraCond.notify_one();
}

