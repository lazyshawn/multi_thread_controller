#include "realsense/shared_variable.h"

ObjState::Data ObjState::get_data(void) {
  std::scoped_lock guard(cameraMutex);
  return data;
}

void ObjState::update(Data* Data_) {
  std::scoped_lock guard(cameraMutex);
  data = *Data_;
}

