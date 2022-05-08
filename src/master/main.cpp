#include "master/interface.h"

/* 线程管理 */
ThreadManager threadmanager(1);

int main(int argc, char** argv) {
  /* 开启线程 */
  std::vector<std::thread> threadPool;
  // threadPool.push_back(std::thread(wsg_controller));
  // threadPool.push_back(std::thread(uskin_controller));
  threadPool.push_back(std::thread(ur5e_controller));
  // threadPool.push_back(std::thread(camera_controller));

  std::vector<double> ori_angle = {0, -98.9*deg2rad, 117.8*deg2rad, -108.9*deg2rad, -90*deg2rad, 90*deg2rad};
  std::vector<double> off_angle = {0, -98.9*deg2rad, 117.8*deg2rad, -108.9*deg2rad, -90*deg2rad, 70*deg2rad};

  /* Initial Grasp */
  THETA jointState = urConfig.get_state();
  while (miniROS::OK()) {
    int key = scanKeyboard();
    switch (key) {
      case 'g':
        ur5e::go_to_joint(off_angle, 3);
        break;
      case 'h':
        ur5e::go_home(3);
        break;
      case 27: case 'q':
        miniROS::shutdown();
        break;
      default:
        break;
    }
  }
  /* 等待线程结束 */
  for (int i=0; i<threadPool.size(); ++i) {
    threadPool[i].join();
  }
  return 0;
}

