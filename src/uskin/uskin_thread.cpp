#include "uskin/uskin_thread.h"

Force force;

/*************************************************************************
* @func : sensor_thread_function;
* @brief: 传感器线程;
*************************************************************************/
void uskin_controller(){
  /* 线程初始化 */
  long double time;
  struct timespec ts;
  Force::Data forceData;
  Uskin uskin;   // Uskin uskin(0, 0);
  if (!uskin.init_dev()) {
    ROS_ERROR("Can't find the sensor: uskin");
    return;
  };
  // 完成初始化
  ROS_INFO("P[%d] T[%Lf] Uskin thread is Ready!",
           get_current_time(), get_tid());

  /* 等待线程同步 */
  ts = threadmanager.wait_for_syc();

  /* 开始伺服周期 */
  while (miniROS::OK()) {
    /* Wait until next shot | 休眠到下个周期开始 */
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);

    /* 伺服线程主程序 */
    uskin.read_force(forceData.forceMat, forceData.timestamp);
    force.update(&forceData);

    /* calculate next shot | 设置下一个线程恢复的时间 */
    timer_incre(ts, USKIN_PERIOD);
  } // while (1)
};

