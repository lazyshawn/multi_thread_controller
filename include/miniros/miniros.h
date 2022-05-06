#ifndef MINIROS_H
#define MINIROS_H

#include "thread_manager.h"
#include "time_manager.h"
#include "user_interface.h"
#include "data_export.h"

/*******************************************************
* Todo:
* 1. 线程同步
* 1. 创建线程
*******************************************************/
namespace miniROS {
  bool OK();
  void shutdown();
}

#endif
