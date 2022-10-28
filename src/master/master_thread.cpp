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
    // 任务1
    if (task == Task::press) {
      task_press();
    } else if (task == Task::pivot_side) {
      task_pivot_side();
    } else if (task == Task::rotate) {
      task_rotate();
    } else if (task == Task::expand) {
      task_expand(0.26);
    } else if (task == Task::pivot_down) {
      task_pivot_down();
    } else if (task == Task::grasp) {
      task_grasp(1.2);
    }
    // 任务2
    else if (task == Task::task2_press) {
      task2_press();
    } else if (task == Task::task2_pivot_side) {
      task2_pivot_side();
    } else if (task == Task::task2_rotate_1) {
      task2_rotate();
    } else if (task == Task::task2_pivot) {
      task2_pivot();
    } else if (task == Task::task2_press_2) {
      task2_press_2();
    } else if (task == Task::task2_pivot_side_2) {
      task2_pivot_side_2();
    } else if (task == Task::task2_rotate_2) {
      task2_rotate_2();
    } else if (task == Task::task2_pivot_2) {
      task2_pivot_2();
    } else if (task == Task::task2_lay_down_2) {
      task2_lay_down_2();
    }
    // 任务4
    else if (task == Task::task4_press) {
      task4_press();
    } else if (task == Task::task4_rotate) {
      task4_rotate();
    } else if (task == Task::task4_pivot) {
      task4_pivot();
    } else if (task == Task::task4_grasp) {
      task4_grasp();
    }

    /* calculate next shot | 设置下一个线程恢复的时间 */
    timer_incre(ts, PLAN_PERIOD);
  } // while (1)
}

double dx, dz;
void task4_press() {
  // 读取传感器数据
  std::vector<double> force = dydwShared.copy_data();
  // 读取机械臂关节角
  THETA curJoint = ur5eShared.copy_data();
  THETA jntCmd = {0, 0, 0, 0, -M_PI/2, M_PI/2}, refJoint = curJoint;
  Arr3d curState;
  plane_kinematics(curJoint, curState);
  // 计算实际压力
  actual_force(force, curState[2]);
  std::vector<double> fDirs = contact_norm(force, curState[2]);
  double f1 = fDirs[4], f2 = fDirs[5];

  if (abs(f1) > 2000) {
    // 压力过大直接退出
    ROS_ERROR("Excessive force detected. f1 = %lf", f1);
    miniROS::shutdown();
    return;
  } else if (force[0] > -1500) {
    curState[0] -= 0.2;
  } else if (force[0] < -2000){
    curState[0] += 0.2;
  } else {
    ROS_INFO("task_press finish. Fy1 = %lf", force[0]);
    task = Task::task4_rotate;
    dx = curState[0] - 335;
    dz = curState[1] + 57;
    std::cout << dx << ", " << dz << std::endl;
    // task = Task::idle;
  }
  refJoint = plane_inv_kinematics(curState);

  planner2ur(curJoint, refJoint);
}

void task4_rotate() {
  static double xo = 135, zo = -99;
  // static double l1 = 200.06, l2 = 205, alpha = 0.025, beta = 0.2213;
  static double l1 = 200.25, l2 = 203.96, alpha = 0.050, beta = 0.1974;
  static double theta = 0, dq = 0.04*deg2rad;
  static double a11 = l2*sin(beta), a12 = l1*cos(alpha), a21 = l1*sin(alpha),
                a22 = l2*cos(beta);

  // 读取传感器数据
  std::vector<double> force = dydwShared.copy_data();
  // 读取机械臂关节角
  THETA curJoint = ur5eShared.copy_data();
  THETA jntCmd = {0, 0, 0, 0, -M_PI/2, M_PI/2}, refJoint = curJoint;
  Arr3d curState;
  plane_kinematics(curJoint, curState);
  // 计算实际压力
  actual_force(force, curState[2]);
  std::vector<double> fDirs = contact_norm(force, curState[2]);
  double f1 = fDirs[4], f2 = fDirs[5];

  // 接触点位置
  double x = curState[0]-dx, z = curState[1]-dz;
  double sq = abs((x-xo)*a11-(z-zo)*a12), cq = abs((z-zo)*a21-(x-xo)*a22);
  theta = atan2(sq, cq);

  // std::cout << "x = " << x << ", z = " << z << std::endl;
  // std::cout << "sq = " << sq << ", cq = " << cq << std::endl;
  // std::cout << "theta = " << theta << std::endl;
  // std::cout << "state[0] = " << curState[0] << ", state[1] = " << curState[1] << std::endl;

    theta += dq;
    curState[0] = l1*cos(theta-alpha) + xo + dx;
    curState[1] = l2*sin(theta+beta) + zo + dz;
  // 终止条件
  if (theta >= 56*deg2rad) {
    ROS_INFO("task4_rotate finish. F1 = %lf", f1);
    task = Task::task4_pivot;
  }
  if (abs(f1) > 2500) {
    // 压力过大直接退出
    ROS_ERROR("Excessive force detected. F1 = %lf", f1);
    miniROS::shutdown();
    return;
  } else if (abs(f1) < 1200 || abs(force[0])<800) {
    curState[0] -= fDirs[0]*0.42;
    curState[1] -= fDirs[1]*0.18;
  } else if (abs(f1) > 2000) {
    curState[0] += fDirs[0]*0.02;
    curState[1] += fDirs[1]*0.02;
  } else {
    // theta += dq;
    // curState[0] = l1*cos(theta-alpha) + xo + dx;
    // curState[1] = l2*sin(theta+beta) + zo + dz;
  }

  refJoint = plane_inv_kinematics(curState);

  planner2ur(curJoint, refJoint);
}

void task4_pivot() {
  // 读取传感器数据
  std::vector<double> force = dydwShared.copy_data();
  // 读取机械臂关节角
  THETA curJoint = ur5eShared.copy_data();
  THETA jntCmd = curJoint, refJoint = curJoint;
  Arr3d curState;
  plane_kinematics(curJoint, curState);
  // 计算实际压力
  actual_force(force, curState[2]);
  std::vector<double> fDirs = contact_norm(force, curState[2]);
  double f1 = fDirs[4], f2 = fDirs[5];

  if (f1 > 3200 || f2 > 2000) {
    ROS_ERROR("Excessive force detected.\nf1 = %lf, f2 = %lf", f1, f2);
    miniROS::shutdown();
    return;
  }

  // 终止条件
  if (f2 > 40) {
    ROS_INFO("task4_pivot finish. F1 = %lf", f1);
    wsgConfig.push({64,10});
    std::cout << "(x,z) = " << curState[0] << ", " << curState[1] << std::endl;
    task = Task::task4_grasp;
  }
  // 计算手指位置
  std::vector<double> figPos = calc_finger_pos(curState, 41);
  tcp_pivot(figPos[0], figPos[1], -0.2*deg2rad, curState);

  curState[0] -= 0.11;
  // curState[1] += 0.01;
  if (abs(f1) < 1000) {
    curState[0] -= fDirs[0]*0.2;
    curState[1] -= fDirs[1]*0.2;
  } else if (abs(f1) > 1200) {
    // curState[0] += fDirs[0]*0.8;
    // curState[1] += fDirs[1]*0.8;
    curState[1] += 0.24;
  }

  refJoint = plane_inv_kinematics(curState);

  planner2ur(curJoint, refJoint);
}

void task4_grasp() {
  // 读取传感器数据
  std::vector<double> force = dydwShared.copy_data();
  // 读取机械臂关节角
  THETA curJoint = ur5eShared.copy_data();
  THETA jntCmd = curJoint, refJoint = curJoint;
  Arr3d curState;
  plane_kinematics(curJoint, curState);
  // 计算实际压力
  actual_force(force, curState[2]);
  std::vector<double> fDirs = contact_norm(force, curState[2]);
  double f1 = fDirs[4], f2 = fDirs[5];

  if (f1 > 2000 || f2 > 2000) {
    ROS_ERROR("Excessive force detected.\nf1 = %lf, f2 = %lf", f1, f2);
    miniROS::shutdown();
    return;
  }

  // 终止条件
  if (curState[1] <= 78) {
    ROS_INFO("task4_grasp finish. F1 = %lf", f1);
    wsgConfig.push({62,10});
    task = Task::idle;
  }
  if (f1 > 400) {
    curState[1] += 0.2;
  } else {
    // 运动控制
    curState[0] -= cos(curState[2])*0.1;
    curState[1] -= cos(curState[2])*0.1;
    if (force[3] > 40) {
      curState[2] += 0.2*deg2rad;
    }
  }

  refJoint = plane_inv_kinematics(curState);

  planner2ur(curJoint, refJoint);
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

  if (force[1] > 2000) {
    // 压力过大直接退出
    ROS_ERROR("Excessive force detected.");
    miniROS::shutdown();
    return;
  } else if (force[1] < 60) {
    state[1] -= 0.8;
  } else if (force[1] > 400){
    state[1] += 0.8;
  } else {
    ROS_INFO("task_press finish. Fz1 = %lf", force[1]);
    task = Task::pivot_side;
  }
  refJoint = plane_inv_kinematics(state);

  planner2ur(curJoint, refJoint);
}

// 绕定点转动
void task_pivot_side(){
  // 读取传感器数据
  std::vector<double> force = dydwShared.copy_data();
  // 读取机械臂关节角
  THETA curJoint = ur5eShared.copy_data();
  THETA jntCmd = {0, 0, 0, 0, -M_PI/2, M_PI/2}, refJoint = curJoint;
  Arr3d curState;
  plane_kinematics(curJoint, curState);
  // 计算实际压力
  actual_force(force, curState[2]);
  std::vector<double> fDirs = contact_norm(force, curState[2]);
  double f1 = fDirs[4], f2 = fDirs[5];

  if (f1 > 2000 || f2 > 2000) {
    ROS_ERROR("Excessive force detected.\nf1 = %lf, f2 = %lf", f1, f2);
    miniROS::shutdown();
    return;
  }

  if (force[3] > 60) {
    task = Task::rotate;
    ROS_INFO("task_pivot finish. F1 = %lf", sqrt(f1));
  }

  tcp_pivot(310, -36, -0.2*deg2rad, curState);

  // 运动补偿
  if (f1 < 100) {
    curState[0] -= fDirs[0]*0.2;
    curState[1] -= fDirs[1]*0.2;
  } else if (f1 > 500) {
    curState[0] += fDirs[0]*0.1;
    curState[1] += fDirs[1]*0.1;
  }

  refJoint = plane_inv_kinematics(curState);
  planner2ur(curJoint, refJoint);
}

void task_rotate() {
  // 读取传感器数据
  std::vector<double> force = dydwShared.copy_data();
  // 读取机械臂关节角
  THETA curJoint = ur5eShared.copy_data();
  THETA jntCmd = {0, 0, 0, 0, -M_PI/2, M_PI/2}, refJoint = curJoint;
  Arr3d curState;
  plane_kinematics(curJoint, curState);
  // 计算实际压力
  actual_force(force, curState[2]);
  std::vector<double> fDirs = contact_norm(force, curState[2]);
  double f1 = fDirs[4], f2 = fDirs[5];

  if (f1 > 2000 || f2 > 2000) {
    ROS_ERROR("Excessive force detected.\nf1 = %lf, f2 = %lf", f1, f2);
    miniROS::shutdown();
    return;
  }

  // 终止条件
  if (curState[2] > 75 * deg2rad) {
    task = Task::expand;
    wsgConfig.push({88, 10});
    return;
  }
  if (f2 > 50) {
    // Rotate
    tcp_pivot(324, -88, 0.2 * deg2rad, curState);
  } else {
    // 计算手指位置
    std::vector<double> figPos = calc_finger_pos(curState, 78);
    tcp_pivot(figPos[0], figPos[1], -0.1*deg2rad, curState);
  }
  // 手指 1 补偿
  if (f1 > 300) {
    curState[0] += fDirs[0]*0.2;
    curState[1] += fDirs[1]*0.2;
  }

  refJoint = plane_inv_kinematics(curState);
  planner2ur(curJoint, refJoint);
}


void task_expand(double ds) {
  // 读取传感器数据
  std::vector<double> force = dydwShared.copy_data();
  // 读取机械臂关节角
  THETA curJoint = ur5eShared.copy_data();
  // 读取夹爪数据
  double dis = wsgConfig.get_state();
  THETA jntCmd = {0, 0, 0, 0, -M_PI/2, M_PI/2}, refJoint = curJoint;
  Arr3d curState;
  plane_kinematics(curJoint, curState);
  // 计算实际压力
  actual_force(force, curState[2]);
  std::vector<double> fDirs = contact_norm(force, curState[2]);
  double f1 = fDirs[4], f2 = fDirs[5];

  if (f1 > 2000 || f2 > 2000) {
    ROS_ERROR("Excessive force detected.\nf1 = %lf, f2 = %lf", f1, f2);
    miniROS::shutdown();
    return;
  }

  // 终止条件
  if (dis >= 87.8) {
    ROS_INFO("task_expand finish. dis = %lf", dis);
    task = Task::pivot_down;
  }

  // 运动控制
  curState[0] += sin(curState[2])*ds;
  curState[1] -= cos(curState[2])*ds;

  // 运动补偿
  if (f1<200) {
    curState[0] -= fDirs[0]*1;
    curState[1] -= fDirs[1]*1;
  } else if (f1 > 500) {
    curState[0] += fDirs[0]*0.8;
    curState[1] += fDirs[1]*0.8;
  }

  refJoint = plane_inv_kinematics(curState);
  planner2ur(curJoint, refJoint);
}

void task_pivot_down() {
  // 读取传感器数据
  std::vector<double> force = dydwShared.copy_data();
  // 读取机械臂关节角
  THETA curJoint = ur5eShared.copy_data();
  THETA jntCmd = {0, 0, 0, 0, -M_PI/2, M_PI/2}, refJoint = curJoint;

  Arr3d curState;
  plane_kinematics(curJoint, curState);
  // 计算实际压力
  actual_force(force, curState[2]);
  double f1 = force[0]*force[0] + force[1]*force[1];

  /// 终止条件
  if (curState[2] <= 70*deg2rad) {
    task = Task::grasp;
    ROS_INFO("task_pivot_down finish. F1 = %lf", sqrt(f1));
    wsgConfig.push({71, 10});
  }
  if (f1 > 4000000) {
    ROS_ERROR("Excessive force detected.");
    miniROS::shutdown();
    return;
  }

  // 计算手指位置
  std::vector<double> figPos = calc_finger_pos(curState, 90);
  tcp_pivot(figPos[0], figPos[1], -0.1*deg2rad, curState);

  // 运动补偿
  if (f1 < 30000) {
    curState[0] += 0.1;
    curState[1] -= 0.1;
  } else if (f1 > 90000) {
    curState[0] -= 0.2;
    curState[1] += 0.2;
  }

  refJoint = plane_inv_kinematics(curState);
  planner2ur(curJoint, refJoint);
}

void task_grasp(double ds) {
  // 读取传感器数据
  std::vector<double> force = dydwShared.copy_data();
  // 读取机械臂关节角
  THETA curJoint = ur5eShared.copy_data();
  // 读取夹爪数据
  double dis = wsgConfig.get_state();
  THETA jntCmd = {0, 0, 0, 0, -M_PI/2, M_PI/2}, refJoint = curJoint;
  Arr3d curState;
  plane_kinematics(curJoint, curState);
  // 计算实际压力
  actual_force(force, curState[2]);
  std::vector<double> fDirs = contact_norm(force, curState[2]);
  double f1 = fDirs[4], f2 = fDirs[5];

  if (f1 > 1000 || f2 > 1000) {
    ROS_ERROR("Excessive force detected.\nf1 = %lf, f2 = %lf", f1, f2);
    miniROS::shutdown();
    return;
  }

  // 终止条件
  if (dis <= 71.5 && dis >= 10) {
    ROS_INFO("task_grasp finish. dis = %lf", dis);
    task = Task::idle;
  }

  // 运动控制
  curState[0] -= sin(curState[2])*ds;
  curState[1] += cos(curState[2])*ds;

  // 运动补偿
  if (f1 > 100) {
    curState[0] -= sin(curState[2])*ds;
    curState[1] += cos(curState[2])*ds;
  // } else if (f1 < 10) {
  }

  refJoint = plane_inv_kinematics(curState);
  planner2ur(curJoint, refJoint);
}

/*************************************************************************
  ********************************* Task 2: *****************************
*************************************************************************/
void task2_press() {
  // 读取传感器数据
  std::vector<double> force = dydwShared.copy_data();
  // 读取机械臂关节角
  THETA curJoint = ur5eShared.copy_data();
  THETA jntCmd = curJoint, refJoint = curJoint;
  Arr3d curState;
  plane_kinematics(curJoint, curState);
  // 计算实际压力
  actual_force(force, curState[2]);
  std::vector<double> fDirs = contact_norm(force, curState[2]);
  double f1 = fDirs[4], f2 = fDirs[5];

  if (f1 > 1000 || f2 > 1000) {
    ROS_ERROR("Excessive force detected.\nf1 = %lf, f2 = %lf", f1, f2);
    miniROS::shutdown();
    return;
  }

  if (force[1] < 30) {
    curState[1] -= 0.4;
  } else if (force[1] > 200){
    curState[1] += 0.8;
  } else {
    ROS_INFO("task_press finish. Fz1 = %lf", force[1]);
    task = Task::task2_pivot_side;
  }
  refJoint = plane_inv_kinematics(curState);

  planner2ur(curJoint, refJoint);
}

void task2_pivot_side() {
  // 读取传感器数据
  std::vector<double> force = dydwShared.copy_data();
  // 读取机械臂关节角
  THETA curJoint = ur5eShared.copy_data();
  THETA jntCmd = curJoint, refJoint = curJoint;
  Arr3d curState;
  plane_kinematics(curJoint, curState);
  // 计算实际压力
  actual_force(force, curState[2]);
  std::vector<double> fDirs = contact_norm(force, curState[2]);
  double f1 = fDirs[4], f2 = fDirs[5];

  if (f1 > 1500 || f2 > 1500) {
    ROS_ERROR("Excessive force detected.\nf1 = %lf, f2 = %lf", f1, f2);
    miniROS::shutdown();
    return;
  }

  if (f2 > 60) {
    ROS_INFO("task2_pivot_side finish. F1 = %lf, F2 = %lf", f1, f2);
    task = Task::task2_rotate_1;
  }

  if (f1 < 60) {
    curState[0] -= fDirs[0]*0.1;
    curState[1] -= fDirs[1]*0.6;
    // curState[1] -= 0.4;
  } else {
    // 计算手指位置
    std::vector<double> figPos = calc_finger_pos(curState, 78);
    tcp_pivot(figPos[0], figPos[1], -0.2*deg2rad, curState);
  }

  refJoint = plane_inv_kinematics(curState);

  planner2ur(curJoint, refJoint);
}

void task2_rotate() {
  // 读取传感器数据
  std::vector<double> force = dydwShared.copy_data();
  // 读取机械臂关节角
  THETA curJoint = ur5eShared.copy_data();
  THETA jntCmd = curJoint, refJoint = curJoint;
  Arr3d curState;
  plane_kinematics(curJoint, curState);
  // 计算实际压力
  actual_force(force, curState[2]);
  std::vector<double> fDirs = contact_norm(force, curState[2]);
  double f1 = fDirs[4], f2 = fDirs[5];

  if (f1 > 2000 || f2 > 2000) {
    ROS_ERROR("Excessive force detected.\nf1 = %lf, f2 = %lf", f1, f2);
    miniROS::shutdown();
    return;
  }

  // 终止条件
  if (curState[2] >= 116*deg2rad) {
    ROS_INFO("task2_rotate finish. F2 = %lf", f2);
    task = Task::task2_pivot;
    // wsgConfig.push({42,10});
  }
  if (f2 > 40) {
    // Rotate
    tcp_pivot(426, -60, 0.2 * deg2rad, curState);
    // tcp_pivot(420, -64, 0.2 * deg2rad, curState);
  } else {
    // 计算手指位置
    std::vector<double> figPos = calc_finger_pos(curState, 78);
    tcp_pivot(figPos[0], figPos[1], -0.1*deg2rad, curState);
  }
  // 手指 1 补偿
  // if (f1 > 100) {
  //   curState[0] += fDirs[0]*1;
  //   curState[1] += fDirs[1]*1;
  // }
  if (f1 < 50) {
    curState[0] += fDirs[0]*0.4;
    curState[1] += fDirs[1]*0.6;
  }

  refJoint = plane_inv_kinematics(curState);

  planner2ur(curJoint, refJoint);
}

void task2_pivot() {
  // 读取传感器数据
  std::vector<double> force = dydwShared.copy_data();
  // 读取机械臂关节角
  THETA curJoint = ur5eShared.copy_data();
  THETA jntCmd = curJoint, refJoint = curJoint;
  Arr3d curState;
  plane_kinematics(curJoint, curState);
  // 计算实际压力
  actual_force(force, curState[2]);
  std::vector<double> fDirs = contact_norm(force, curState[2]);
  double f1 = fDirs[4], f2 = fDirs[5];

  if (f1 > 2000 || f2 > 2000) {
    ROS_ERROR("Excessive force detected.\nf1 = %lf, f2 = %lf", f1, f2);
    miniROS::shutdown();
    return;
  }

  // 终止条件
  if (curState[2] <= 90*deg2rad) {
    ROS_INFO("task2_rotate_2 finish. F2 = %lf", f2);
    // wsgConfig.push({52,40});
    task = Task::idle;
    for (int i=0; i<3; ++i) {
      std::cout << curState[i] << " ";
    }
    std::cout << std::endl;
  }
  // 计算手指位置
  std::vector<double> figPos = calc_finger_pos(curState, 52);
  tcp_pivot(figPos[2], figPos[3], -0.6*deg2rad, curState);
  curState[0] -= 0.42;

  if (f2 < 100) {
    curState[0] -= fDirs[0]*0.4;
    curState[1] -= fDirs[1]*0.4;
  } else if (f2 > 400) {
    curState[0] += fDirs[0]*1.8;
    curState[1] += fDirs[1]*1.8;
  }

  refJoint = plane_inv_kinematics(curState);

  planner2ur(curJoint, refJoint);
}

void task2_press_2() {
  // 读取传感器数据
  std::vector<double> force = dydwShared.copy_data();
  // 读取机械臂关节角
  THETA curJoint = ur5eShared.copy_data();
  THETA jntCmd = curJoint, refJoint = curJoint;
  Arr3d curState;
  plane_kinematics(curJoint, curState);
  // 计算实际压力
  actual_force(force, curState[2]);
  std::vector<double> fDirs = contact_norm(force, curState[2]);
  double f1 = fDirs[4], f2 = fDirs[5];

  if (f1 > 1000 || f2 > 1000) {
    ROS_ERROR("Excessive force detected.\nf1 = %lf, f2 = %lf", f1, f2);
    miniROS::shutdown();
    return;
  }

  if (force[1] < 30) {
    curState[1] -= 0.4;
  } else if (force[1] > 200){
    curState[1] += 0.8;
  } else {
    ROS_INFO("task_press finish. Fz1 = %lf", force[1]);
    task = Task::task2_pivot_side_2;
  }
  refJoint = plane_inv_kinematics(curState);

  planner2ur(curJoint, refJoint);
}

void task2_pivot_side_2() {
  // 读取传感器数据
  std::vector<double> force = dydwShared.copy_data();
  // 读取机械臂关节角
  THETA curJoint = ur5eShared.copy_data();
  THETA jntCmd = curJoint, refJoint = curJoint;
  Arr3d curState;
  plane_kinematics(curJoint, curState);
  // 计算实际压力
  actual_force(force, curState[2]);
  std::vector<double> fDirs = contact_norm(force, curState[2]);
  double f1 = fDirs[4], f2 = fDirs[5];

  if (f1 > 1000 || f2 > 1000) {
    ROS_ERROR("Excessive force detected.\nf1 = %lf, f2 = %lf", f1, f2);
    miniROS::shutdown();
    return;
  }

  if (f2 > 60 || curState[2] >120*deg2rad) {
    ROS_INFO("task2_pivot_side finish. F1 = %lf, F2 = %lf", f1, f2);
    task = Task::task2_rotate_2;
  }

  if (f1 < 60) {
    // curState[0] -= fDirs[0]*0.1;
    // curState[1] -= fDirs[1]*0.6;
    curState[1] -= 0.2;
  } else {
    // 计算手指位置
    std::vector<double> figPos = calc_finger_pos(curState, 42);
    tcp_pivot(figPos[0], figPos[1], -0.2*deg2rad, curState);
  }

  refJoint = plane_inv_kinematics(curState);

  planner2ur(curJoint, refJoint);
}

void task2_rotate_2() {
  // 读取传感器数据
  std::vector<double> force = dydwShared.copy_data();
  // 读取机械臂关节角
  THETA curJoint = ur5eShared.copy_data();
  THETA jntCmd = curJoint, refJoint = curJoint;
  Arr3d curState;
  plane_kinematics(curJoint, curState);
  // 计算实际压力
  actual_force(force, curState[2]);
  std::vector<double> fDirs = contact_norm(force, curState[2]);
  double f1 = fDirs[4], f2 = fDirs[5];

  if (f1 > 2000 || f2 > 2000) {
    ROS_ERROR("Excessive force detected.\nf1 = %lf, f2 = %lf", f1, f2);
    miniROS::shutdown();
    return;
  }

  // 终止条件
  if (curState[2] >= 100*deg2rad) {
    ROS_INFO("task2_rotate finish. F2 = %lf", f2);
    task = Task::task2_pivot_2;
    wsgConfig.push({44,10});
  }
  if (f2 > 40) {
    // Rotate
    tcp_pivot(410, -108, 0.2*deg2rad, curState);
  } else {
    // 计算手指位置
    std::vector<double> figPos = calc_finger_pos(curState, 78);
    tcp_pivot(figPos[0], figPos[1], -0.1*deg2rad, curState);
  }
  // 手指 1 补偿
  if (f1 < 60) {
    curState[0] -= fDirs[0]*0.1;
    curState[1] -= fDirs[1]*0.8;
  }

  refJoint = plane_inv_kinematics(curState);

  planner2ur(curJoint, refJoint);
}

void task2_pivot_2() {
  // 读取传感器数据
  std::vector<double> force = dydwShared.copy_data();
  // 读取机械臂关节角
  THETA curJoint = ur5eShared.copy_data();
  THETA jntCmd = curJoint, refJoint = curJoint;
  Arr3d curState;
  plane_kinematics(curJoint, curState);
  // 计算实际压力
  actual_force(force, curState[2]);
  std::vector<double> fDirs = contact_norm(force, curState[2]);
  double f1 = fDirs[4], f2 = fDirs[5];

  if (f1 > 2000 || f2 > 2000) {
    ROS_ERROR("Excessive force detected.\nf1 = %lf, f2 = %lf", f1, f2);
    miniROS::shutdown();
    return;
  }

  // 终止条件
  if (curState[2] <= 95*deg2rad) {
    ROS_INFO("task2_rotate_2 finish. F2 = %lf", f2);
    // wsgConfig.push({48,10});
    task = Task::task2_lay_down_2;
  }
  // 计算手指位置
  std::vector<double> figPos = calc_finger_pos(curState, 52);
  tcp_pivot(figPos[2], figPos[3], -0.8*deg2rad, curState);
  // curState[0] -= 0.42;

  if (f2 < 100) {
    curState[0] -= fDirs[0]*0.4;
    curState[1] -= fDirs[1]*0.4;
  } else if (f2 > 400) {
    curState[0] += fDirs[0]*1;
    curState[1] += fDirs[1]*1;
  }

  refJoint = plane_inv_kinematics(curState);

  planner2ur(curJoint, refJoint);
}

void task2_lay_down_2() {
  // 读取传感器数据
  std::vector<double> force = dydwShared.copy_data();
  // 读取机械臂关节角
  THETA curJoint = ur5eShared.copy_data();
  THETA jntCmd = curJoint, refJoint = curJoint;
  Arr3d curState;
  plane_kinematics(curJoint, curState);
  // 计算实际压力
  actual_force(force, curState[2]);
  std::vector<double> fDirs = contact_norm(force, curState[2]);
  double f1 = fDirs[4], f2 = fDirs[5];

  if (f1 > 2000 || f2 > 2000) {
    ROS_ERROR("Excessive force detected.\nf1 = %lf, f2 = %lf", f1, f2);
    miniROS::shutdown();
    return;
  }

  // 终止条件
  if (curState[2] <= 90*deg2rad) {
    ROS_INFO("task2_rotate_2 finish. F2 = %lf", f2);
    task = Task::idle;
  }
  // 计算手指位置
  std::vector<double> figPos = calc_finger_pos(curState, 44);
  tcp_pivot(figPos[2], figPos[3], -0.2*deg2rad, curState);
  curState[0] -= 0.6;
  curState[1] -= 0.2;

  // if (f2 > 100) {
  //   curState[1] += 0.4;
  // }
  // if (f2 < 40) {
  //   curState[1] -= 0.8;
    // curState[0] -= fDirs[0]*1.8;
    // curState[1] -= fDirs[1]*2.8;
  // } else if (f2 > 400) {
  //   curState[0] += fDirs[0]*1;
  //   curState[1] += fDirs[1]*1;
  // }

  refJoint = plane_inv_kinematics(curState);

  planner2ur(curJoint, refJoint);
}

/*************************************************************************
*************************************************************************/
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

// 平面定点转动
void tcp_pivot(double x, double z, double dq, Arr3d& state) {
  double x0 = state[0], z0 = state[1], q0 = state[2];
  // 圆环路径参数
  double alpha0 = atan2(z0-z, x0-x);
  double radius = sqrt((x0-x)*(x0-x) + (z0-z)*(z0-z));

  // 按圆心角插值(圆的参数方程)
  state[0] = x + radius*cos(alpha0+dq);
  state[1] = z + radius*sin(alpha0+dq);
  state[2] = q0+dq;
}

// 计算手指位置
std::vector<double> calc_finger_pos(Arr3d state, double gripWidth) {
  double leftFingerX, leftFingerZ, tipAngle, rightFingerX, rightFingerZ;
  double offset = gripWidth/2 - 3;
  std::vector<double> fingerPos;

  tipAngle = state[2] + M_PI/2;
  // 左手指
  leftFingerX = state[0] + cos(tipAngle) * offset;
  leftFingerZ = state[1] + sin(tipAngle) * offset;
  // 右手指
  rightFingerX = state[0] - cos(tipAngle) * offset;
  rightFingerZ = state[1] - sin(tipAngle) * offset;

  fingerPos = {leftFingerX, leftFingerZ, rightFingerX, rightFingerZ};
  return fingerPos;
}

// 接触面上的力控: 传感器测量的力在世界坐标系下的表示(方向、大小)
std::vector<double> contact_norm(std::vector<double> force, double theta) {
  std::vector<double> dirs(6,0), ff(2,0);
  // 传感器合力大小
  ff[0] = sqrt(force[0]*force[0] + force[1]*force[1]);
  ff[1] = sqrt(force[2]*force[2] + force[3]*force[3]);
  for (int i=0; i<4; ++i) force[i] /= ff[i/2];

  dirs[0] = -sin(theta)*force[0] + cos(theta)*force[1];
  dirs[1] = cos(theta)*force[0] + cos(theta)*force[1];
  dirs[2] = sin(theta)*force[2] + cos(theta)*force[3];
  dirs[3] = -cos(theta)*force[2] + sin(theta)*force[3];

  dirs[4] = ff[0];
  dirs[5] = ff[1];
  return dirs;
}

