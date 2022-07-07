#include "wsg_controller/wsg_thread.h"

WSGConfig wsgConfig;

/* 夹爪线程 */
void wsg_controller(){
  /* 线程初始化 */
  struct timespec ts;
  WSGGripper gripper("10.249.180.222", 1000);
  wsgConfig.set_ready();
  WSGCMD cmd = {0,0};
  // 完成初始化
  ROS_INFO("P[%d] T[%Lf] WSG thread is Ready!", get_tid(), get_current_time());

  /* 等待线程同步 */
  ts = threadmanager.wait_for_syc();

  /* 开始伺服周期 */
  while (miniROS::OK()) {
    /* Wait until next shot | 休眠到下个周期开始 */
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);

    /* 伺服线程主程序 */
    wsgConfig.update_state(gripper.read_pos());
    if (wsgConfig.pop(cmd)) {
      gripper.move(cmd[0], cmd[1]);
    }

    /* calculate next shot | 设置下一个线程恢复的时间 */
    timer_incre(ts, WSG_PERIOD);
  } // while (1)
};

