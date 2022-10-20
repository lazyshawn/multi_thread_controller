#include "master/master_thread.h"

volatile Task task = Task::idle;
const int PLAN_PERIOD = 30000000;

/*************************************************************************
* @func : master_planner_thread_function;
* @brief: 传感器线程;
*************************************************************************/
void planner_controller() {
  /* 线程初始化 */
  struct timespec ts;
  long double time, startTime;
  // 完成初始化
  ROS_INFO("P[%d] T[%Lf] UR5e thread is Ready!", get_tid(), get_current_time());

  /* 等待线程同步 */
  ts = threadmanager.wait_for_syc();

  /* 开始伺服周期 */
  startTime = timespec2time(ts);
  while (miniROS::OK()) {
    /* Wait until next shot | 休眠到下个周期开始 */
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);

    /* 伺服线程主程序 */
    // 每个伺服周期开始的时间
    time = get_current_time() - startTime;
    if (task == Task::press) {
      task_press();
    } else if (task == Task::pivot) {
      pivot_finger(310, -31, -0.2*deg2rad);
    } else if (task == Task::rotate) {
      task_rotate(322, -88, 0.1*deg2rad);
    } else if (task == Task::calib) {
    }

    /* calculate next shot | 设置下一个线程恢复的时间 */
    timer_incre(ts, PLAN_PERIOD);
  } // while (1)
}

void task_press() {
  // 读取传感器数据
  std::vector<double> force = dydwShared.copy_data();
  // 读取机械臂关节角
  THETA curJoint = ur5eShared.copy_data();
  THETA jntCmd = curJoint, refJoint = curJoint;
  Arr3d state;
  plane_kinematics(curJoint, state);
  // 计算实际压力
  actual_force(force, state[2]);

  // std::cout << force[0] << std::endl;
  if (force[1] > 2000) {
    // 压力过大直接退出
    ROS_ERROR("Excessive force detected.");
    miniROS::shutdown();
    return;
  } else if (force[1] < 100) {
    state[1] -= 0.8;
  } else if (force[1] > 500){
    state[1] += 1;
  } else {
    ROS_INFO("press finish.");
    task = Task::pivot;
  }
  refJoint = plane_inv_kinematics(state);

  planner2ur(curJoint, refJoint);
}

// 绕定点转动
void pivot_finger(double x, double z, double dq){
  // 读取传感器数据
  std::vector<double> force = dydwShared.copy_data();
  // 读取机械臂关节角
  THETA curJoint = ur5eShared.copy_data();
  THETA jntCmd = {0, 0, 0, 0, -M_PI/2, M_PI/2}, refJoint = curJoint;
  Arr3d curState, refState;
  plane_kinematics(curJoint, curState);
  // 计算实际压力
  actual_force(force, curState[2]);
  double f1 = force[0]*force[0] + force[1]*force[1];

  if (force[3] > 100) {
    task = Task::rotate;
    ROS_INFO("pivot finish");
  }
  if (force[0] > 2000) {
    ROS_ERROR("Excessive force detected.");
    miniROS::shutdown();
    return;
  }

  double x0 = curState[0], z0 = curState[1], q0 = curState[2];
  // 圆环路径参数
  double alpha0 = atan2(z0-z, x0-x);
  double radius = sqrt((x0-x)*(x0-x) + (z0-z)*(z0-z));

  // 按圆心角插值(圆的参数方程)
  refState[0] = x + radius*cos(alpha0+dq);
  refState[1] = z + radius*sin(alpha0+dq);
  refState[2] = q0+dq;
  refJoint = plane_inv_kinematics(refState);
  if (f1 < 40000) {
    refState[0] += 3;
    refState[1] -= 3;
  }

  planner2ur(curJoint, refJoint);
}

void task_rotate(double x, double z, double dq) {
  // 读取传感器数据
  std::vector<double> force = dydwShared.copy_data();
  // 读取机械臂关节角
  THETA curJoint = ur5eShared.copy_data();
  THETA jntCmd = {0, 0, 0, 0, -M_PI/2, M_PI/2}, refJoint = curJoint;
  Arr3d curState, refState;
  plane_kinematics(curJoint, curState);
  // 计算实际压力
  actual_force(force, curState[2]);
  double f1 = force[0]*force[0] + force[1]*force[1];
  double f2 = force[2]*force[2] + force[3]*force[3];

  if (f1 > 1500*1500 || f2 > 1500*1500) {
    ROS_ERROR("Excessive force detected.");
    miniROS::shutdown();
    return;
  }

  double x0 = curState[0], z0 = curState[1], q0 = curState[2];
  // 圆环路径参数
  double alpha0 = atan2(z0-z, x0-x);
  double radius = sqrt((x0-x)*(x0-x) + (z0-z)*(z0-z));

  // 按圆心角插值(圆的参数方程)
  refState[0] = x + radius*cos(alpha0+dq);
  refState[1] = z + radius*sin(alpha0+dq);
  refState[2] = q0+dq;
  refJoint = plane_inv_kinematics(refState);

  if (force[2] < 100) {
    // task = Task::idle;
    // ROS_INFO("pivot finish");
    pivot_finger(310, -31, -0.2*deg2rad);
  }

  planner2ur(curJoint, refJoint);
}

void init_grasp() {
  // 读取传感器数据
  std::vector<double> force = dydwShared.copy_data();
}

void actual_force(std::vector<double>& force, double theta) {
  force[0] = force[0] - dydwShared.F0[0] + cos(theta)*dydwShared.G[0];
  force[1] = force[1] - dydwShared.F0[1] + sin(theta)*dydwShared.G[0];
  force[2] = force[2] - dydwShared.F0[2] - cos(theta)*dydwShared.G[1];
  force[3] = force[3] - dydwShared.F0[3] + sin(theta)*dydwShared.G[1];
}

void planner2ur(THETA curJoint, THETA goalJoint){
  THETA jntCmd = {0, 0, 0, 0, -M_PI/2, M_PI/2};
  int n = 3;
  for (int j=0; j<n; ++j) {
    for (int i=0; i<6; ++i) {
      jntCmd[i] = curJoint[i] + (goalJoint[i] - curJoint[i])*j/(n-1);
    }
    ur5eShared.push(jntCmd);
  }
}

