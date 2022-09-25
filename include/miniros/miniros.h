#ifndef MINIROS_H
#define MINIROS_H

#include "thread_manager.h"
#include "time_manager.h"
#include "user_interface.h"
#include "data_export.h"
#include <functional>   // std::bind

extern ThreadManager threadmanager;
extern std::vector<std::thread> threadPool;

/*******************************************************
* Todo:
* 1. 线程同步
* 1. 创建线程
*******************************************************/
namespace miniROS {
  bool OK();
  void shutdown();

  template<typename F>
  void node(F func) {
    threadPool.emplace_back(std::thread(func));
  }

  void node_test(int num);
  void joinall();
} // namespace miniROS

#endif
