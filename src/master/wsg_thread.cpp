#include "master/wsg_thread.h"

extern ThreadManager threadmanager;
WSGConfig wsgConfig;

/* 夹爪线程 */
void wsg_controller(){
  /* 线程初始化 */
  long double time;
  struct timespec t;
  WSGGripper gripper("10.249.180.222", 1000);
  WSGCMD cmd = {0,0};
  time = get_current_time();
  ROS_INFO("[%Lf] WSG thread is Ready!", time);

  /* 等待线程同步 */
  t = threadmanager.wait_for_syc();

  /* 开始伺服周期 */
  while (miniROS::OK()) {
    /* Wait until next shot | 休眠到下个周期开始 */
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

    /* 伺服线程主程序 */
    gripper.read_pos();
    if (wsgConfig.pop(cmd)) {
      gripper.move(cmd[0], cmd[1]);
    }

    /* calculate next shot | 设置下一个线程恢复的时间 */
    t.tv_nsec += WSG_PERIOD;
    // 时间进位
    while (t.tv_nsec >= NSEC_PER_SEC) {
      t.tv_nsec -= NSEC_PER_SEC;
      t.tv_sec++;
    }
  } // while (1)
};

