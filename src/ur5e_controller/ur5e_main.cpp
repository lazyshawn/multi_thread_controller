#include "miniros/miniros.h"
#include "ur5e_controller/ur5e_interface.h"

extern Ur5eShared ur5eShared;
ThreadManager threadmanager(1);

int main(int argc, char** argv) {
  double time;
  std::vector<std::thread> threadPool;
  std::thread urThread;
  threadPool.push_back(std::thread(ur5e_controller));

  std::vector<double> ori_angle = {0, -98.9*deg2rad, 117.8*deg2rad, -108.9*deg2rad, -90*deg2rad, 90*deg2rad};
  std::vector<double> off_angle = {0, -98.9*deg2rad, 117.8*deg2rad, -108.9*deg2rad, -90*deg2rad, 70*deg2rad};

  while (miniROS::OK()) {
    int key = scanKeyboard();
    switch (key) {
      case 'g':
        ur5e::go_to_joint(off_angle, 3);
        break;
      case 'h':
        ur5e::go_to_joint(ori_angle, 3);
        break;
      case 27: case 'q':
        miniROS::shutdown();
        break;
      default:
        break;
    }
  }
  for (int i=0; i<threadPool.size(); ++i) {
    threadPool[i].join();
  }
  return 0;
}

