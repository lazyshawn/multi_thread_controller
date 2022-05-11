#include "ur5e_controller/ur5e_thread.h"

UrConfig urConfig;
Recorder urRecord(7);

/*************************************************************************
* @func : sensor_thread_function;
* @brief: 传感器线程;
*************************************************************************/
void ur5e_controller() {
  /* 线程初始化 */
  struct timespec ts;
  long double time, startTime;
  // UR通信的条件变量
  std::condition_variable rt_ur_msg_cond, ur_msg_cond;
  UrDriver ur(rt_ur_msg_cond, ur_msg_cond, "10.249.181.201");
  THETA jointState, refJoint;
  double delQ, maxDq = 2 * deg2rad;
  ur.start();
  usleep(500); // 机械臂初始化，确保能TCP通信连上(accept)
  ur.setServojTime((double)UR_PERIOD / NSEC_PER_SEC);
  ur.uploadProg();
  // 完成初始化
  ROS_INFO("P[%d] T[%Lf] UR5e thread is Ready!", get_current_time(), get_tid());

  /* 等待线程同步 */
  ts = threadmanager.wait_for_syc();

  /* 开始伺服周期 */
  startTime = timespec2time(ts);
  while (miniROS::OK()) {
    /* Wait until next shot | 休眠到下个周期开始 */
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);

    /* 伺服线程主程序 */
    // 每个伺服周期开始的时间
    // time = get_current_time() - startTime;
    // 读取机械臂关节角位置
    jointState = ur.rt_interface_->robot_state_->getQActual();
    urConfig.update_state(jointState);
    // 弹出路径并发送 servoj 指令
    if (urConfig.pop(refJoint)) {
      // 急停保护
      for (int i = 0; i < 6; ++i) {
        delQ = abs(refJoint[i] - jointState[i]);
        if (delQ > maxDq) {
          ROS_ERROR("Excessive speed at Joint %d: %lf / %lf", i, delQ * rad2deg,
                    maxDq * rad2deg);
          miniROS::shutdown();
        }
      }
      ur.servoj(refJoint);
    }

    /* calculate next shot | 设置下一个线程恢复的时间 */
    ts.tv_nsec += UR_PERIOD;
    // 时间进位
    time_wrap(ts);
  } // while (1)
}

