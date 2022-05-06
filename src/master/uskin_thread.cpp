#include "master/uskin_thread.h"

extern ThreadManager threadmanager;
extern Force force;

/*************************************************************************
* @func : sensor_thread_function;
* @brief: 传感器线程;
*************************************************************************/
void uskin_controller(){
  /* 线程初始化 */
  long double time;
  struct timespec t;
  Force::Data forceData;
  Uskin uskin;   // Uskin uskin(0, 0);
  if (!uskin.init_dev()) {
    ROS_ERROR("Can't find the sensor: uskin");
    return;
  };
  time = get_current_time();
  ROS_INFO("[%Lf] Uskin thread is Ready!", time);

  /* 等待线程同步 */
  t = threadmanager.wait_for_syc();

  /* 开始伺服周期 */
  while (miniROS::OK()) {
    /* Wait until next shot | 休眠到下个周期开始 */
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

    /* 伺服线程主程序 */
    uskin.read_force(forceData.forceMat, forceData.timestamp);
    force.update(&forceData);

    /* calculate next shot | 设置下一个线程恢复的时间 */
    t.tv_nsec += USKIN_PERIOD;
    // 时间进位
    while (t.tv_nsec >= NSEC_PER_SEC) {
      t.tv_nsec -= NSEC_PER_SEC;
      t.tv_sec++;
    }
  } // while (1)
};

