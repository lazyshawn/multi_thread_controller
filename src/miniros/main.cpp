#include "miniros/miniros.h"
#include <vector>

ThreadManager threadmanager(8);
unsigned long long int tsc0;

void func(int i) {
  long double time;
  struct timespec t;
  threadmanager.wait_for_syc();
  t = threadmanager.get_timespec();
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
  time = get_current_time();
  printf("Thread [%d] start at: %Lf\n", i, time);
}

int main(int argc, char** argv) {
  std::vector<std::thread> threads;
  for (int i=0; i<8; ++i) {
    threads.emplace_back(std::thread(func,i));
    for (int j; j<100000000; ++j) {
    }
    printf("delay for a while\n");
  }
  for (int i=0; i<8; ++i) {
    threads[i].join();
  }
  return 0;
}

