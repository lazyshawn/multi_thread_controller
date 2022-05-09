#include "miniros/thread_manager.h"

ThreadManager::ThreadManager()
  : threadNum(1), threadCounter(0) {
}

ThreadManager::ThreadManager(int threadNum_)
  : threadNum(threadNum_), threadCounter(0) {
}

void ThreadManager::timespec_init(){
  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec++;
}

timespec ThreadManager::wait_for_syc(int countIncre) {
  std::unique_lock lock(threadMutex);
  threadCounter += countIncre;
  if(threadCounter==threadNum) {
    threadCondVar.notify_all();
  } else {
    // 等待其他线程启动
    threadCondVar.wait(lock, [this]{return threadCounter==threadNum;});
  }
  // 等待 timespec 初始化
  std::call_once(timespec_init_flag,&ThreadManager::timespec_init,this);
  return ts;
}

timespec ThreadManager::get_timespec() {
  return ts;
}

pid_t get_tid() { return syscall(SYS_gettid); }

