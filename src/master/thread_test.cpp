#include "miniros/miniros.h"
#include "master/wsg_thread.h"

ThreadManager threadmanager(1);

WSGCMD cmd;
WSGConfig wsgConfig;

/* 夹爪线程 */
void wsg_controller(){
  /* 线程初始化 */
  // WSGGripper gripper("10.249.180.222", 1000);
  // gripper.home(1);
  long double time;
  struct timespec t;
  WSGCMD cmd = {0,0};

  /* 等待线程同步 */
  t = threadmanager.wait_for_syc();
  time = get_current_time();
  printf("Thread start at: %Lf\n", time);

  /* 开始伺服周期 */
  while (true) {
    /* Wait until next shot | 休眠到下个周期开始 */
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

    /* 伺服线程主程序 */
    if (wsgConfig.pop(cmd)) {
      printf("WSGCMD: %f\n", cmd[0]);
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

int main(int argc, char** argv) {
  std::thread wsgThread(wsg_controller);

  while (true) {
    int key = scanKeyboard();
    switch (key) {
      case 'g':
        cmd = {10,10};
        break;
      case 'h':
        cmd = {0,0};
        break;
      case 27: case 'q':
        return 0;
        break;
      default:
        break;
    }
    wsgConfig.push(cmd);
  }
  wsgThread.join();
  return 0;
}

