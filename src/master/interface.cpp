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
        pick_and_place(126);
        // object_rotation();
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
  Mat4d obj2elk, elk2base, hoverPose, gripPose, rlsPose;
  double peakHeight = 26, pickHeight, gripWidth = 52, rlsWidth = 60;
  rlsPose << 1,0,0,350, 0,-1,0,DH_D4, 0,0,-1,peakHeight, 0,0,0,1;
  THETA jointState = urConfig.get_state();
  ur_kinematics(jointState, elk2base);
  obj2elk = objState.get_marker();

  gripPose = hoverPose = elk2base*obj2elk;
  hoverPose(2,3) = hoverHeight;
  pickHeight = gripPose(2,3);

  // Pick
  ur5e::go_to_pose(hoverPose, 3);
  ur5e::wait_path_clear();
  ur5e::go_to_pose(gripPose, 3);
  ur5e::wait_path_clear();
  wsgConfig.push({gripWidth,40});
  sleep(2);
  // Transmit
  gripPose(2,3) = peakHeight;
  ur5e::go_to_pose(gripPose, 3);
  ur5e::wait_path_clear();
  ur5e::go_to_pose(rlsPose, 3);
  ur5e::wait_path_clear();
  // Pick
  rlsPose(2,3) = pickHeight;
  ur5e::go_to_pose(rlsPose, 3);
  ur5e::wait_path_clear();
  wsgConfig.push({rlsWidth,40});
}

void object_rotation() {
  Arr3d screw = {0, -50, -30*deg2rad};
  ur5e::plane_screw(screw, 5);
  /* **************** Debug **************** */
  // wsgConfig.push({52,40});
  // Mat4d pinPose;
  // pinPose << 1,0,0,350, 0,-1,0,DH_D4, 0,0,-1,179, 0,0,0,1;
  // ur5e::go_to_pose(pinPose, 5);
  // ur5e::wait_path_clear();
  // wsgConfig.push({73.54,20});
}

void finger_pivot() {
  THETA jointState = urConfig.get_state();
  Arr3d state;
  plane_kinematics(jointState, state);
  ROS_INFO("JointState");
  for (int i=0; i<6; ++i) {
    std::cout << jointState[i]*rad2deg << "  ";
  }
  std::cout << std::endl;
  ROS_INFO("state");
  for (int i=0; i<3; ++i) {
    std::cout << state[i] << "  ";
  }
  std::cout << std::endl;
  jointState = plane_inv_kinematics(state);
  ROS_INFO("JointState");
  for (int i=0; i<6; ++i) {
    std::cout << jointState[i]*rad2deg << "  ";
  }
  std::cout << std::endl;
}

