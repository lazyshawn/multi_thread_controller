#pragma once

#include <limits.h>
#include <malloc.h>
#include <thread>
#include <future>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <stdlib.h>
#include <sys/mman.h>     // needed for mlockall()
#include <sys/resource.h> // needed for getrusage
#include <sys/time.h>     // needed for getrusage
#include <unistd.h>       // needed for sysconf(int name);
#include <sys/syscall.h>  // needed for syscall
#include "miniros/user_interface.h"

// Ref: https://stackoverflow.com/questions/24220778/simultaneous-threads-in-c-using-thread
class ThreadManager {
private:
  int threadNum;         // 线程总数
  int threadCounter;     // 已启动的线程计数器
  struct timespec ts;    // 线程同步开始的时间戳
  mutable std::mutex threadMutex;
  std::once_flag timespec_init_flag;
  std::condition_variable threadCondVar;

  void timespec_init();

public:
  ThreadManager();
  ThreadManager(int threadNum_);
  /* 线程同步启动 */
  timespec wait_for_syc(int countIncre = 1);
  timespec get_timespec();
};

// 获取线程pid
pid_t get_tid();

