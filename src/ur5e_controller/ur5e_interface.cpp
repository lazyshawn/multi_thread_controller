#include "ur5e_controller/ur5e_interface.h"

namespace ur5e {
/*************************************************************************
 * 轨迹规划
*************************************************************************/
void go_to_joint(THETA joint, double time) {
  THETA jntCmd=std::vector<double>(6), origJoint = urConfig.get_state();
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

void go_home(double time){
  Mat4d homePose;
  homePose << 1, 0, 0, 400,  0, -1, 0, DH_D4, 0, 0, -1, 300,  0, 0, 0, 1;
  go_to_pose(homePose, time);
}
} // namespace ur5e
