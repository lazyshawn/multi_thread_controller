#include "miniros/miniros.h"
#include "master/wsg_thread.h"
#include "master/uskin_thread.h"
#include "ur5e_controller/ur5e_thread.h"
#include "realsense/camera_thread.h"

/* 线程管理 */
ThreadManager threadmanager(2);
/* 夹爪数据 */
extern WSGConfig wsgConfig;
WSGCMD cmd;
/* 传感器数据 */
extern Force force;
/* 机械臂数据 */
extern UrConfig urConfig;

int main(int argc, char** argv) {
  /* 开启线程 */
  std::vector<std::thread> threadPool;
  std::thread uskinThread, wsgThread, urThread;
  // threadPool.push_back(std::thread(wsg_controller));
  // threadPool.push_back(std::thread(uskin_controller));
  threadPool.push_back(std::thread(ur5e_controller));
  threadPool.push_back(std::thread(camera_controller));

  /* Initial Grasp */
  THETA jointState = urConfig.get_state();
  while (miniROS::OK()) {
    int key = scanKeyboard();
    switch (key) {
      case 'g':
        cmd = {50,100};
        wsgConfig.push(cmd);
        break;
      case 'h':
        cmd = {100,100};
        wsgConfig.push(cmd);
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

