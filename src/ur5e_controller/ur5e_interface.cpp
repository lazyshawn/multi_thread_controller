#include "ur5e_controller/ur5e_interface.h"

extern UrConfig urConfig;

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

void go_to_pose(Mat4d tranMat, THETA curTheta, double time) {
  // PATH path;
  // Mat4d kinematics;
  // // 角度伺服标志置位
  // path.servoMode = ANGLE_SERVO;
  // path.interpMode = 2;
  // path.freq = 1.0/time;
  // path.goal = ur_InverseKinematics(tranMat, curTheta);
  // pathQueue.push(path);
}

void go_home(THETA curTheta){
  // PATH path;
  // Mat4d homePose;
  // homePose << 1, 0, 0, 400,  0, -1, 0, DH_D4, 0, 0, -1, 300,  0, 0, 0, 1;
  // go_to_pose(homePose, curTheta, 5);
}

// void pivot_about_points(TRIARR& state, TRIARR command, double time) {
  // double xr = state[0], zr = state[1], qr = state[2];
  // double x0 = command[0], z0 = command[1], q0 = command[2];
  // double alpha0 = atan2(zr-z0, xr-x0);
  // double radius = sqrt((zr-z0)*(zr-z0)+(xr-x0)*(xr-x0));
  //
  // // 按圆心角插值(圆的参数方程)
  // THETA theta;
  // double n = floor(time/SERVO_TIME);
  // double da = command[2]/n;
  //
  // for (int i=0; i<n; ++i) {
  //   state[0] = x0 + radius*cos(alpha0+da*(i+1));
  //   state[1] = z0 + radius*sin(alpha0+da*(i+1));
  //   state[2] = qr + da*(i+1);
  //   // state[2] = qr;
  //   theta = plane_invese_kinematics(state);
  //   instant_command(theta);
  // }
  // printf("done\n");
// }

/*************************************************************************
 * @func : plane_pivot(Arr3d command, float time);
 * @brief: 平面定点转动
 * @param: command [x0, y0, dq]: 定点位置(x,y), 转动角度dq;
*************************************************************************/
bool plane_pivot(Arr3d command, float time){
  // // 开始时系统的状态
  // THETA theta = {0, 0, 0, 0, -M_PI/2, M_PI/2};
  // Arr3d stateInit, state;
  // plane_kinematics(stateInit);
  // double x0 = stateInit[0], z0 = stateInit[1], q0 = stateInit[2];
  // double xc = command[0], zc = command[1];
  // // 圆环路径参数
  // double alpha0 = atan2(z0-zc, x0-xc);
  // double radius = sqrt((zc-z0)*(zc-z0)+(xc-x0)*(xc-x0));
  //
  // // 按圆心角插值(圆的参数方程)
  // double n = floor(time/SERVO_TIME), dq = command[2]/n;
  // for (int i=0; i<n; ++i) {
  //   state[0] = xc + radius*cos(alpha0+dq*(i+1));
  //   state[1] = zc + radius*sin(alpha0+dq*(i+1));
  //   state[2] = q0 + dq*(i+1);
  //   theta = plane_invese_kinematics(state);
  //   instant_command(theta);
  // }
  // return true;
}

/*************************************************************************
 * @func : plane_screw(Arr3d screw, float time);
 * @brief: 平面螺旋运动
 * @param: screw为末端相对于世界坐标系的位移增量 [dx, dz, dq], dq以逆时针为正;
*************************************************************************/
bool plane_screw(Arr3d screw, float time){
  // // 开始时系统的状态
  // THETA theta = {0, 0, 0, 0, -M_PI/2, M_PI/2};
  // Arr3d stateInit, state;
  // plane_kinematics(stateInit);
  // double x0 = stateInit[0], z0 = stateInit[1], q0 = stateInit[2], len = 172;
  //
  // // 按圆心角插值(圆的参数方程)
  // double n = floor(time/SERVO_TIME);
  // double dx = screw[0]/n, dz = screw[1]/n, dq = screw[2]/n;
  // for (int i=0; i<n; ++i) {
  //   state[2] = q0 + dq*(i+1);
  //   state[0] = x0 + len*(sin(q0) - sin(state[2])) + dx*(i+1);
  //   state[1] = z0 - len*(cos(q0) - cos(state[2])) + dz*(i+1);
  //   theta = plane_invese_kinematics(state);
  //   instant_command(theta);
  // }
  // return true;
}

/*************************************************************************
 * @func : plane_translate(Arr3d command, float time);
 * @brief: 平面定点平动
 * @param: command [x0, y0, dq]: 定点位置(x,y), 转动角度dq;
*************************************************************************/
// bool plane_gripper_translate(double command, float time){
//   // 开始时系统的状态
//   THETA theta = {0, 0, 0, 0, -M_PI/2, M_PI/2};
//   Arr3d stateInit, state;
//   plane_kinematics(stateInit);
//   double x0 = stateInit[0], z0 = stateInit[1], q0 = stateInit[2];
//
//   // 按圆心角插值(圆的参数方程)
//   double n = floor(time/SERVO_TIME);
//   for (int i=0; i<n; ++i) {
//     state[0] = x0 + command*cos(q0)*i/n;
//     state[1] = z0 + command*sin(q0)*i/n;
//     state[2] = q0;
//     theta = plane_invese_kinematics(state);
//     instant_command(theta);
//   }
//   return true;
// }

// void instant_command(THETA refTheta) {
//   PATH pathLocal;
//   // pathLocal.fingerPos = 70;
//   for (int i=0; i<6; ++i) {
//     pathLocal.goal[i] = refTheta[i];
//   }
//   pathQueue.push(pathLocal);
// }

// bool wait_for_path_clear(void) {
//   urConfig::Data urConfigData;
//   usleep(200);
//   for (int i=0; ; ++i) {
//     if (threadManager.process == THREAD_EXIT) return false;
//     if (pathQueue.empty()) {
//       urConfigData = urconfig.get_data();
//       if(urConfigData.path.status == PATH_CLEAR) return true;
//     }
//   }
// }

/*************************************************************************
 * @func : move_ref_to_end(Arr3d direction, float time)
 * @brief: 相对末端运动
 * @param: direction [x, y, z]: 运动方向在末端坐标系下的表示, time: 时间;
*************************************************************************/
// // 这只是沿末端x轴方向运动
// bool move_ref_to_end(double distance, float time) {
//   // 开始时系统的状态
//   THETA theta = {0, 0, 0, 0, -M_PI/2, M_PI/2};
//   Arr3d stateInit, state;
//   plane_kinematics(stateInit);
//   double x0 = stateInit[0], z0 = stateInit[1], q0 = stateInit[2];
//   urConfig::Data urConfigData = urconfig.get_data();
//   double dirX = urConfigData.tranMat(0,0);
//   double dirZ = urConfigData.tranMat(2,0);
//   double dr = sqrt(dirX*dirX + dirZ*dirZ);
//   std::vector<PATH> pathBuf;
//   PATH path;
//
//   double gain, temp;
//   // 取消角度伺服标志
//   path.servoMode = VELOCITY_SERVO;
//   // 读入路径信息
//   path.freq = 1/time;
//   gain = path.freq * path.delT;
//   path.velocity[0] = distance*dirX/dr;
//   path.velocity[3] = distance*dirZ/dr;
//   // 归一化: 转化为一个伺服周期内的位移量
//   for (int i=0; i<6; ++i) path.velocity[i] *= gain;
//   pathQueue.push(path);
//   return true;
// }

