#include "dydw/dydw_shared.h"

std::vector<double> DydwShared::copy_data (void) {
  std::scoped_lock guard(dydwMutex);
  return data;
}

void DydwShared::update_data(std::vector<double> dydwData) {
  std::scoped_lock guard(dydwMutex);
  data = dydwData;
}

