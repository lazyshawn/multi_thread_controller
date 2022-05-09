#include "master/interface.h"

/* 线程管理 */
ThreadManager threadmanager(1);

int main(int argc, char** argv) {
  /* 开启线程 */
  std::vector<std::thread> threadPool;
  // threadPool.push_back(std::thread(wsg_controller));
  // threadPool.push_back(std::thread(uskin_controller));
  // threadPool.push_back(std::thread(ur5e_controller));
  threadPool.push_back(std::thread(camera_controller));

  cpu_set_t mask;
  // 初始化set集，将set置为空
  CPU_ZERO(&mask);
  // 将1-4号CPU加入集合(有0号CPU)
  CPU_SET(5,&mask);
  // 设置CPU亲和性
  if(pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask) < 0) {
    fprintf(stderr, "### Error ###/n=== Set servo affinity failed\n");
  }

  pid_t mainPid = get_tid();
  ROS_INFO("P[%d] Main thread is Ready!", mainPid);
  threadmanager.wait_for_syc(0);

  main_menu();

  /* 等待线程结束 */
  for (int i=0; i<threadPool.size(); ++i) {
    threadPool[i].join();
  }
  return 0;
}

