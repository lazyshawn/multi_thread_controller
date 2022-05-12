#include "miniros/time_manager.h"

long double get_current_time() {
  struct timespec time;
  long double curTime;
  unsigned long long int tsc1;

  clock_gettime(CLOCK_REALTIME, &time);
  return timespec2time(time);
}

void time_wrap(timespec& ts) {
  while (ts.tv_nsec >= NSEC_PER_SEC) {
    ts.tv_nsec -= NSEC_PER_SEC;
    ts.tv_sec++;
  }
}

void timer_incre(timespec& ts, long long int period) {
  ts.tv_nsec += period;
  time_wrap(ts);
}

long double timespec2time(timespec ts) {
  unsigned long long int tsc1;
  tsc1 = (long long int)(ts.tv_sec) * NSEC_PER_SEC +
         (long long int)(ts.tv_nsec);

  return (long double)tsc1/NSEC_PER_SEC;
}

