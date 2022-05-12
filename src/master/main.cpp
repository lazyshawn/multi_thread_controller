#include "master/interface.h"

/* 线程管理 */
ThreadManager threadmanager(3);

int main(int argc, char** argv) {
  /* 开启线程 */
  std::vector<std::thread> threadPool;
  threadPool.emplace_back(std::thread(camera_controller));
  threadPool.push_back(std::thread(wsg_controller));
  // threadPool.push_back(std::thread(uskin_controller));
  threadPool.emplace_back(std::thread(ur5e_controller));

  ROS_INFO("P[%d] Main thread is Ready!", get_tid());
  threadmanager.wait_for_syc(0);

  main_menu();

  /* 等待线程结束 */
  for (int i=0; i<threadPool.size(); ++i) {
    if(threadPool[i].joinable()) threadPool[i].join();
  }
  return 0;
}

