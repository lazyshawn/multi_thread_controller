#ifndef THREAD_MANAGER_H
#define THREAD_MANAGER_H

#include <limits.h>
#include <malloc.h>
// #include <pthread.h>
#include <thread>
#include <future>
#include <mutex>
#include <condition_variable>
// #include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <sys/mman.h>     // needed for mlockall()
#include <sys/resource.h> // needed for getrusage
#include <sys/time.h>     // needed for getrusage
#include <unistd.h>       // needed for sysconf(int name);

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
  timespec wait_for_syc();
  timespec get_timespec();
};

// class InterruptFlag {
//   public:
//     void set();
//     bool is_set() const;
// };
//
// 指定为线程存储期，每个线程都互补影响
// thread_local InterruptFlag this_thread_interrupt_flag;
// class Node {
//   private:
//     std::thread internalThread;
//     InterruptFlag* flag;
//   public:
//     template<typename FunctionType>
//     Node(FunctionType f) {
//       std::promise<InterruptFlag*> p;
//       internalThread = std::thread([f, &p]{
//           p.set_value(&this_thread_interrupt_flag);
//           f();
//       });
//       flag = p.get_future().get();
//     }
//     void interrupt(){
//       if(flag) {flag->set();}
//     };
//     void check_interrupt(){
//       if (this_thread_interrupt_flag.is_set()) {
//         throw thread_interrupted();
//       }
//     };
// };

#endif
