#include "miniros/miniros.h"

namespace miniROS {
  std::atomic<bool> isOK{true};
  bool OK() {return isOK;}
  void shutdown() {isOK = false;}
}

