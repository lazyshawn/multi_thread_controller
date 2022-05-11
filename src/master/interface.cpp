#include "master/interface.h"

void main_menu() {
  while (miniROS::OK()) {
    int key = scanKeyboard();
    switch (key) {
      case 'h':
        break;
      case 'r':
        ROS_INFO("You're in ur5e teleoperate mode.");
        ur5e::teleoperate();
        break;
      case 't':
        pick_and_place(300);
        break;
      case 27: case 'q':
        miniROS::shutdown();
        break;
      default:
        break;
    }
  }
}

void devices_home() {
  ur5e::go_home(3);
}

void pick_and_place(double hoverHeight) {
  Mat4d obj2elk = objState.get_marker();
  std::cout << obj2elk << std::endl;
}

