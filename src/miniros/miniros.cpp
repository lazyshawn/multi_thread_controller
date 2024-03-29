#include "miniros/miniros.h"

ThreadManager threadmanager;
std::vector<std::thread> threadPool;

namespace miniROS {
  std::atomic<bool> isOK{true};
  // ThreadManager threadManager;

  bool OK() {return isOK;}
  void shutdown() {isOK = false;}

  // 调用函数node_test, 传入参数 1;
  // miniROS::node(std::bind(miniROS::node_test, 1));
  void node_test(int num) {
    ROS_INFO("P[%d] node_test thread is Ready!", get_tid());
    threadmanager.wait_for_syc(1);
    std::cout << "The input number is: " << num << std::endl;
  }

  void joinall() {
    for (int i=0; i<threadPool.size(); ++i) {
      if(threadPool[i].joinable()) threadPool[i].join();
    }
  }
} // namespace miniROS

