#include "master/master_interface.h"
enum class Color{red, blue, green};

int main(int argc, char** argv) {
  /* 开启线程 */
  miniROS::node(ur5e_controller);
  miniROS::node(dydw_controller);
  // miniROS::node(hfvc_controller);

  ROS_INFO("P[%d] Main thread is Ready!", get_tid());
  threadmanager.set_threadNum(threadPool.size());
  threadmanager.wait_for_syc(0);

  main_menu();

  /* 等待线程结束 */
  miniROS::joinall();
  return 0;
}

