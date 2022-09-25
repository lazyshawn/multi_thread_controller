#include "dydw/dydw_thread.h"

DYDWDriver dydw;
DydwShared dydwShared;
Recorder dydwRecord(5);

/*************************************************************************
* @func : dydw_controller;
* @brief: 传感器线程;
*************************************************************************/
void dydw_controller() {
  /* 线程初始化 */
  struct timespec ts;
  long double time, startTime;
  dydw.init("/dev/ttyUSB0");
  std::vector<float> dataF = std::vector<float>(4,0);
  std::vector<double> dataD = std::vector<double>(4,0);
  // 完成初始化
  ROS_INFO("P[%d] T[%Lf] DYDW thread is Ready!", get_tid(), get_current_time());

  /* 等待线程同步 */
  ts = threadmanager.wait_for_syc();

  /* 开始伺服周期 */
  startTime = timespec2time(ts);
  // dydwRecord.flag_reset();
  while (miniROS::OK()) {
    /* Wait until next shot | 休眠到下个周期开始 */
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);

    /* 伺服线程主程序 */
    // 每个伺服周期开始的时间
    time = get_current_time() - startTime;
    dydwRecord.push_item(std::vector<double>(1, time));
    // 读取传感器数据
    dataF = dydw.read();
    dataD = std::vector<double>(dataF.begin(), dataF.end());
    dydwRecord.push_item(dataD);
    // 同步数据
    dydwShared.update_data(dataD);

    // 记录数据
    dydwRecord.data_record();
    /* calculate next shot | 设置下一个线程恢复的时间 */
    timer_incre(ts, DYDW_PERIOD);
  } // while (1)
  dydwRecord.data_export("dydwData.txt");
}

