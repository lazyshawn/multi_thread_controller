#include "master/interface.h"

/* 线程管理 */
// ThreadManager threadmanager(0);

int main(int argc, char** argv) {
  /* 开启线程 */
  // 调用函数node_test, 传入参数 1;
  miniROS::node(std::bind(miniROS::node_test, 1));
  // std::vector<std::thread> threadPool;
  // threadPool.emplace_back(std::thread(camera_controller));
  // threadPool.emplace_back(std::thread(wsg_controller));
  // threadPool.emplace_back(std::thread(uskin_controller));
  // threadPool.emplace_back(std::thread(ur5e_controller));

  ROS_INFO("P[%d] Main thread is Ready!", get_tid());
  threadmanager.set_threadNum(threadPool.size());
  threadmanager.wait_for_syc(0);

  // main_menu();

  /* 等待线程结束 */
  miniROS::joinall();
  return 0;
}

