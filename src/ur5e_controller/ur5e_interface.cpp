#include "ur5e_controller/ur5e_interface.h"

namespace ur5e {
/*************************************************************************
 * 轨迹规划
*************************************************************************/
void go_to_joint(THETA joint, double time) {
  THETA jntCmd = std::vector<double>(6), origJoint = urConfig.get_state();
  double prop, dt;
  bool finish = false;
  prop = dt = UR_SERVO_TIME / time;
  while (!finish) {
    finish = traj_interpolate(origJoint, joint, prop, jntCmd);
    urConfig.push(jntCmd);
    prop += dt;
  }
}

void go_to_pose(Mat4d tranMat, double time) {
  THETA curTheta = urConfig.get_state();
  THETA jntCmd = ur_InverseKinematics(tranMat, curTheta);
  go_to_joint(jntCmd, time);
}

void go_home(double time) {
  Mat4d homePose;
  homePose << 1, 0, 0, 400,  0, -1, 0, DH_D4, 0, 0, -1, 300,  0, 0, 0, 1;
  go_to_pose(homePose, time);
}

void print_current_info() {
  THETA jointState = urConfig.get_state();
  Mat4d tranMat;
  ur_kinematics(jointState, tranMat);
  ROS_INFO("Current Information:");
  std::cout << "---------- Joint State [deg] ----------" << std::endl;
  for (int i=0; i<6; ++i) {
    std::cout << jointState[i]*rad2deg << "    ";
  }
  std::cout << std::endl;
  std::cout << "-------- Transform Matrix --------\n" << tranMat << std::endl;
}

void teleoperate() {
  std::vector<double> ori_angle = {
      0, -98.9*deg2rad, 117.8*deg2rad, -108.9*deg2rad, -90*deg2rad, 90*deg2rad};
  std::vector<double> off_angle = {
      0, -98.9*deg2rad, 117.8*deg2rad, -108.9*deg2rad, -90*deg2rad, 70*deg2rad};
  while (miniROS::OK()) {
    int key = scanKeyboard();
    switch (key) {
      case 'g':
        ur5e::go_to_joint(off_angle, 3);
        break;
      case 'h':
        ur5e::go_home(3);
        break;
      case 'p':
        ur5e::print_current_info();
        break;
      case 'q':
        ROS_INFO("Back to main menu.");
        return;
      case 27:
        miniROS::shutdown();
        break;
      default:
        break;
    }
  }
}
} // namespace ur5e
