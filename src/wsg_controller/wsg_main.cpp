#include "miniros/miniros.h"
#include "wsg_controller/wsg_driver.h"

ThreadManager threadmanager(1);

int main(int argc, char** argv) {
  WSGGripper gripper("10.249.180.222", 1000);
  gripper.home(1);
  float pos = 100;

  while (true) {
    int key = scanKeyboard();
    switch (key) {
      case 'j':
        gripper.grip();
        break;
      case 'k':
        pos -= 0.5;
        gripper.move(pos, 100);
        printf("%f\n", pos);
        break;
      case 'h':
        gripper.home(1);
        break;
      case 27: case 'q':
        return 0;
    }
  }

  return 0;
}

