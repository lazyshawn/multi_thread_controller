#include "master/master_interface.h"

int main(int argc, char** argv) {
  /* 开启线程 */
  miniROS::node(ur5e_controller);
  miniROS::node(dydw_controller);
  miniROS::node(planner_controller);
  // miniROS::node(wsg_controller);
  // miniROS::node(camera_controller);

  ROS_INFO("P[%d] Main thread is Ready!", get_tid());
  threadmanager.set_threadNum(threadPool.size());
  threadmanager.wait_for_syc(0);

  main_menu();

  /* 等待线程结束 */
  miniROS::joinall();
  return 0;
}

