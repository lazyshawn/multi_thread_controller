#include "master/interface.h"

THETA rotAftJnt = {0, - 1.76592, 2.52261, -2.43964, -1.57081,1.5708};
// 翻转圆顶矩形 - 开环
THETA rotBegJnt_2 = {0, -81.3037, 124.034, -113.608, -90, 90};

Mat4d rlsPose;
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
        // pick_and_place(126);
        wsgConfig.push({71, 40});
        // rlsPose << 1,0,0,350, 0,-1,0,DH_D4, 0,0,-1,-40, 0,0,0,1;
        // ur5e::go_to_pose(rlsPose, 5);
        for (int i=0; i<6; ++i) { rotBegJnt_2[i] *= deg2rad; }
        ur5e::go_to_joint(rotBegJnt_2, 8);
        if (scanKeyboard() == 'q') {return;}
        ur5e::tcp_pivot_2d({330, -70, -15*deg2rad}, 10);
        break;
      case 'p':
        // ur5e::tcp_pivot_2d({330, -60, -4*deg2rad}, 5);
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
  // Place
  rlsPose(2,3) = pickHeight;
  ur5e::go_to_pose(rlsPose, 3);
  ur5e::wait_path_clear();
  wsgConfig.push({rlsWidth,40});
}

void object_rotation() {
}

void finger_pivot() {
  THETA jointState = urConfig.get_state();
  Arr3d state;

  double leftFingerX, leftFingerZ, tipAngle;
  plane_kinematics(jointState, state);
  tipAngle = state[2] + M_PI/2;
  leftFingerX = state[0] - cos(tipAngle * 40);
  leftFingerZ = state[1] + sin(tipAngle * 40);
  std::cout << "leftFingerX, leftFingerZ, q = " << leftFingerX << leftFingerZ << state[2] << std::endl;

  ur5e::tcp_pivot_2d({leftFingerX, leftFingerZ, 15*deg2rad}, 10);
}

// 魔方翻转实验 - 橡胶片
void cube_test_1() {
  // object_rotation
  wsgConfig.push({60, 40});
  THETA rotInitJnt = {-0.0014653, -70.0052, 108.694, -90.1435, -89.9986, 90};
  for (int i=0; i<6; ++i) { rotInitJnt[i] *= deg2rad; }
  ur5e::go_to_joint(rotInitJnt, 8);
  if (scanKeyboard() == 'q') {return;}
  ur5e::tcp_pivot_2d({335, -60, -45*deg2rad}, 40);
  // finger_pivot
  if (scanKeyboard() == 'q') {return;}
  ur5e::body_twist(6, 1.2);
  wsgConfig.push({72, 10});
  if (scanKeyboard() == 'q') {return;}
  ur5e::tcp_pivot_2d({275, -10.0, 18*deg2rad}, 10);
  if (scanKeyboard() == 'q') {return;}
  ur5e::body_twist(-2, 0.4);
  wsgConfig.push({68, 10});
  // final_grip
  if (scanKeyboard() == 'q') {return;}
  ur5e::body_twist(-18, 1.6);
  wsgConfig.push({52, 10});
}

