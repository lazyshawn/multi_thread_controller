#include "master/master_interface.h"

THETA rotAftJnt = {0, - 1.76592, 2.52261, -2.43964, -1.57081,1.5708};
// 翻转圆顶矩形 - 开环
// THETA rotBegJnt_2 = {0, -81.3037, 124.034, -113.608, -90, 90};
THETA rotBegJnt_2 = {0, -76.319, 117.504, -113.314, -90, 90};

Mat4d zeroPose, rlsPose;
std::vector<double> data = std::vector<double>(6,0);

void main_menu() {
  THETA joint;
  while (miniROS::OK()) {
    int key = scanKeyboard();
    switch (key) {
      case 'h':
        zeroPose << 1, 0, 0, 350, 0, -1, 0, DH_D4, 0, 0, -1, 0, 0, 0, 0, 1;
        ur5e::go_to_pose(zeroPose, 8);
        break;
      case 'r':
        ROS_INFO("You're in ur5e teleoperate mode.");
        ur5e::teleoperate();
        break;
      case 't':
        for (int i=0; i<1000; ++i) {
          hfvcShared.push_queue({0,-0.5, 0,0});
        }
        break;
      case 'p':
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
  THETA jointState = ur5eShared.copy_data();
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
  THETA jointState = ur5eShared.copy_data();
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

// 力位混合伺服控制
Arr3d hfvc_executer(std::vector<double> hfvcCmd, Arr3d state, std::vector<double> force) {
  double fdx=hfvcCmd[0], fdz=hfvcCmd[1], vdx = hfvcCmd[2], vdz = hfvcCmd[3];
  double fx=force[0], fz=force[1];
  // 力控
  double fdPrj = fx*fdx + fz*fdz;
  if (fabs(fdPrj) < 200) {
    state[0] += fdx;
    state[1] += fdz;
  } else if (fabs(fdPrj) > 700) {
    state[0] -= fdx;
    state[1] -= fdz;
  }
  // 位置控制
  state[0] += vdx;
  state[1] += vdz;
  return state;
}

