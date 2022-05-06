#include "miniros/miniros.h"
#include "uskin/shared_variable.h"

Force force;

/*************************************************************************
* @class: Force;
*************************************************************************/
Force::Data Force::copy_data (void) {
  std::scoped_lock guard(forceMutex);
  return forceData;
}

void Force::update(Force::Data* ForceData_) {
  std::scoped_lock guard(forceMutex);
  forceData = *ForceData_;
}

