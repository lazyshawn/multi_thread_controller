#include "ur5e_controller/ur5e_interface.h"

namespace ur5e {
bool check_ur_state() {
  if(!ur5eShared.is_ready()) {
    ROS_WARN("UR is not ready!");
    // miniROS::shutdown();
  }
  return ur5eShared.is_ready();
}

/*************************************************************************
 * 轨迹规划
*************************************************************************/
void go_to_joint(THETA joint, double time) {
  check_ur_state();
  THETA jntCmd = std::vector<double>(6), origJoint = ur5eShared.copy_data();
  double prop, dt;
  bool finish = false;
  prop = dt = UR_SERVO_TIME / time;
  while (!finish) {
    finish = traj_interpolate(origJoint, joint, prop, jntCmd);
    ur5eShared.push(jntCmd);
    prop += dt;
  }
}

void go_to_pose(Mat4d tranMat, double time) {
  THETA curTheta = ur5eShared.copy_data();
  THETA jntCmd = ur_InverseKinematics(tranMat, curTheta);
  go_to_joint(jntCmd, time);
}

void go_home(double time) {
  check_ur_state();
  // Mat4d homePose;
  // homePose << 1, 0, 0, 400,  0, -1, 0, DH_D4, 0, 0, -1, 126,  0, 0, 0, 1;
  THETA homeJnt = {0, -98.9285, 117.772, -108.845, -89.9979, 89.998};
  for (int i=0; i<6; ++i) {
    homeJnt[i] *= deg2rad;
  }
  go_to_joint(homeJnt, time);
  // go_to_pose(homePose, time);
}

/*************************************************************************
 * @ brief: 末端以给定物体运动旋量进行运动(目前只是沿末端x轴平移)
 * @ param: time - 运动总时间
*************************************************************************/
void body_twist(double distance, double time) {
  THETA jointState = ur5eShared.copy_data();
  Mat4d tranMat;
  ur_kinematics(jointState, tranMat);
  ur5e::tcp_move_2d({tranMat(0,0)*distance, tranMat(2,0)*distance, 0}, time);
}

void tcp_move_2d(Arr3d movement, double time){
  THETA theta = {0, 0, 0, 0, -M_PI/2, M_PI/2};
  // 开始时系统的状态
  THETA curTheta = ur5eShared.copy_data(), jntCmd;
  Arr3d stateInit, refState;
  plane_kinematics(curTheta, stateInit);
  double x0 = stateInit[0], z0 = stateInit[1], q0 = stateInit[2];

  double prop, dt;
  prop = dt = UR_SERVO_TIME / time;
  while (prop<=1) {
    for (int i=0; i<3; ++i) {
      refState[i] = stateInit[i] + movement[i]*prop;
    }
    jntCmd = plane_inv_kinematics(refState);
    ur5eShared.push(jntCmd);
    prop += dt;
  }
}

/*************************************************************************
 * @ brief: tcp 绕定点转动
 * @ param: twist[x0, y0, dq] - 定点位置(x,y); time - 时间;
 * @ note : tcpState 与 twist 的转角方向定义相反;
*************************************************************************/
void tcp_pivot_2d(Arr3d twist, double time) {
  THETA jointState = ur5eShared.copy_data();
  THETA jntCmd = {0, 0, 0, 0, -M_PI/2, M_PI/2};
  // 初始状态
  Arr3d curState, refState;
  plane_kinematics(jointState, curState);
  double x0 = curState[0], z0 = curState[1], q0 = curState[2];
  double xc = twist[0], zc = twist[1], dq = -twist[2];
  // 圆环路径参数
  double alpha0 = atan2(z0-zc, x0-xc);
  double radius = sqrt((x0-xc)*(x0-xc) + (z0-zc)*(z0-zc));

  // 按圆心角插值(圆的参数方程)
  double prop, dt;
  prop = dt = UR_SERVO_TIME / time;
  while (prop <= 1) {
    refState[2] = q0 - dq*prop;
    refState[0] = xc + radius*cos(alpha0+dq*prop);
    refState[1] = zc + radius*sin(alpha0+dq*prop);
    jntCmd = plane_inv_kinematics(refState);
    ur5eShared.push(jntCmd);
    prop += dt;
  }
}

/*************************************************************************
* @ Param: screw(xIncre, yIncre, q); time;
* @ Note : q_elk = q_234; q_actuator = q_234 + pi;
*************************************************************************/
void plane_screw(Arr3d screw, double time) {
  THETA jointState = ur5eShared.copy_data(), jntCmd;
  Arr3d state, detState;
  plane_kinematics(jointState, state);
  for (int i=0; i<3; ++i) detState[i] = screw[i] * UR_SERVO_TIME / time;
  double prop, dt;
  prop = dt = UR_SERVO_TIME / time;
  while (prop <= 1) {
    for (int i=0; i<3; ++i) state[i] += detState[i];
    jntCmd = plane_inv_kinematics(state);
    ur5eShared.push(jntCmd);
    prop += dt;
  }
}

std::vector<double> print_tcp_position(double gripWidth) {
  THETA jointState = ur5eShared.copy_data();
  Arr3d state;
  double leftFingerX, leftFingerZ, tipAngle, rightFingerX, rightFingerZ;
  double offset = gripWidth/2 + 2;
  std::vector<double> fingerPos;

  plane_kinematics(jointState, state);
  tipAngle = state[2] + M_PI/2;
  leftFingerX = state[0] - cos(tipAngle) * offset;
  leftFingerZ = state[1] + sin(tipAngle) * offset;
  std::cout << "angle of elk [deg]: " << state[2]*rad2deg << std::endl;
  std::cout << "leftFingerX, leftFingerZ : " << leftFingerX << ", "<< leftFingerZ << std::endl;
  rightFingerX = state[0] + cos(tipAngle) * offset;
  rightFingerZ = state[1] - sin(tipAngle) * offset;
  std::cout << "rightFingerX, rightFingerZ : " << rightFingerX << ", "<< rightFingerZ << std::endl;

  fingerPos = {leftFingerX, leftFingerZ, rightFingerX, rightFingerZ};
  return fingerPos;
}

void wait_path_clear() {
  while(!ur5eShared.empty());
}

void print_current_info() {
  THETA jointState = ur5eShared.copy_data();
  Mat4d tranMat;
  ur_kinematics(jointState, tranMat);
  ROS_INFO("Current Information:");
  std::cout << "---------- Joint State [deg] ----------" << std::endl;
  for (int i=0; i<6; ++i) {
    std::cout << jointState[i]*rad2deg << "    ";
  }
  std::cout << std::endl;
  std::cout << "-------- Transform Matrix --------\n" << tranMat << std::endl;
  Arr3d state;
  plane_kinematics(jointState, state);
  std::cout << "-------- Grip State --------\n"
            << state[0] << "    " << state[1] << "    " << state[2] * rad2deg
            << std::endl;
}

void teleoperate() {
  check_ur_state();
  std::vector<double> ori_angle = {
      0, -98.9*deg2rad, 117.8*deg2rad, -108.9*deg2rad, -90*deg2rad, 90*deg2rad};
  std::vector<double> off_angle = {
      0, -98.9*deg2rad, 117.8*deg2rad, -108.9*deg2rad, -90*deg2rad, 70*deg2rad};
  THETA jointState;
  Mat4d tranMat;
  while (miniROS::OK()) {
    int key = scanKeyboard();
    switch (key) {
      case 'g':
        ur5e::go_home(3);
        break;
      // tcp 在 {base} 下的平移
      case 'h': ur5e::tcp_move_2d({-1,0,0}, 0.2); break;
      case 'H': ur5e::tcp_move_2d({-10,0,0}, 2); break;
      case 'j': ur5e::tcp_move_2d({0,-1,0}, 0.2); break;
      case 'J': ur5e::tcp_move_2d({0,-10,0}, 2); break;
      case 'k': ur5e::tcp_move_2d({0,1,0}, 0.2); break;
      case 'K': ur5e::tcp_move_2d({0,10,0}, 2); break;
      case 'l': ur5e::tcp_move_2d({1,0,0}, 0.2); break;
      case 'L': ur5e::tcp_move_2d({10,0,0}, 2); break;
      // 旋转
      case 'u': ur5e::tcp_move_2d({0,0,1*deg2rad}, 0.2); break;
      case 'U': ur5e::tcp_move_2d({0,0,5*deg2rad}, 1); break;
      case 'i': ur5e::tcp_move_2d({0,0,-1*deg2rad}, 0.2); break;
      case 'I': ur5e::tcp_move_2d({0,0,-5*deg2rad}, 1); break;

      case 't':
        jointState = ur5eShared.copy_data();
        ur_kinematics(jointState, tranMat);
        ur5e::tcp_move_2d({tranMat(0,0)*10, tranMat(2,0)*10, 0}, 0.2*10);
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
