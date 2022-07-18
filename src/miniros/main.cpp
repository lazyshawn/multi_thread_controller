#include "miniros/miniros.h"
#include <vector>

ThreadManager threadmanager(8);

void hello() {
  std::cout << "hello" << std::endl;
}

int main(int argc, char** argv) {
  // miniROS::node(hello);
  // sleep(1);
  // miniROS::join();
}

