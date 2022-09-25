#include "master/master_thread.h"

PlannerFlag planner = PlannerFlag::interface;

HfvcShared hfvcShared;
Recorder hfvcRecord(5);

void hfvc_controller() {
  /* 线程初始化 */
  struct timespec ts;
  long double time, startTime;
  // 其他线程数据
  std::vector<double> dydwData, jointState, ur5eData, hfvcQueue;
  Mat4d tranM;
  Arr3d tcpState;
  // 完成初始化
  ROS_INFO("P[%d] T[%Lf] HFVC thread is Ready!", get_tid(), get_current_time());

  /* 等待线程同步 */
  ts = threadmanager.wait_for_syc();

  /* 开始伺服周期 */
  startTime = timespec2time(ts);
  while (miniROS::OK()) {
    /* Wait until next shot | 休眠到下个周期开始 */
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);

    /* 伺服线程主程序 */
    // 每个伺服周期开始的时间
    time = get_current_time() - startTime;
    hfvcRecord.push_item(std::vector<double>(1, time));

    /* calculate next shot | 设置下一个线程恢复的时间 */
    timer_incre(ts, HFVC_PERIOD);
    if (planner != PlannerFlag::hfvc) {break;}

    // 读取其他线程的数据
    dydwData = dydwShared.copy_data();
    ur5eData = ur5eShared.copy_data();
    plane_kinematics(ur5eData, tcpState);
    // 获取控制指令
    if(hfvcShared.pop_queue(hfvcQueue)) {
      // 计算: 末端状态(2d) / 位姿矩阵(3d)
      tcpState = hfvc_executer(hfvcQueue, tcpState, dydwData);
      jointState = plane_inv_kinematics(tcpState);
      ur5e::go_to_joint(jointState, (double)HFVC_PERIOD/NSEC_PER_SEC);
    }

    // 同步数据

    // 记录数据
    hfvcRecord.data_record();
  } // while (1)
  // hfvcRecord.data_export("hfvc.txt");
}

