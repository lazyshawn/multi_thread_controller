#ifndef TIME_MANAGER_H
#define TIME_MANAGER_H

#include <time.h>
#include <sys/time.h>     // needed for getrusage

#define NSEC_PER_SEC (1000000000)

/*****************************************************************************
* @ Func:   GetCurrentTime
* @ Brief:  获取当前时间 Wall time
* @ Return: long double
*****************************************************************************/
long double get_current_time();

void time_wrap(timespec& ts);

void timer_incre(timespec& ts, long long int period);

long double timespec2time(timespec ts);

#endif
