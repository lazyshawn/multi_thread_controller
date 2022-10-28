#pragma once

#include "master/master_shared.h"
#include "wsg_controller/wsg_thread.h"
#include "ur5e_controller/ur5e_interface.h"
#include "realsense/camera_thread.h"
#include "dydw/dydw_thread.h"

enum class Task {
  idle,    // 初始状态
  // 任务1
  press,   // 下压
  pivot_side,   // 倾斜
  rotate,  // 转动物体
  expand,  // 张开手指
  teleopr, // 遥操作
  pivot_down,
  grasp,
  // 任务2-1
  task2_press,
  task2_pivot_side,
  task2_rotate_1,
  task2_pivot,
  // 任务2-2
  task2_press_2,
  task2_pivot_side_2,
  task2_rotate_2,
  task2_pivot_2,
  task2_lay_down_2,
  // 任务4
  task4_press,
  task4_rotate,
  task4_pivot,
  task4_grasp,
};

void planner_controller();

// 任务1
void task_press();
void task_pivot_side();
void task_rotate();
void task_expand(double ds);
void task_pivot_down();
void task_grasp(double ds);

// 任务2
void task2_press();
void task2_pivot_side();
void task2_rotate();
void task2_pivot();
void task2_press_2();
void task2_pivot_side_2();
void task2_rotate_2();
void task2_pivot_2();
void task2_lay_down_2();

// 任务4
void task4_press();
void task4_rotate();
void task4_pivot();
void task4_grasp();

void actual_force(std::vector<double>& force, double theta);
void planner2ur(THETA curJoint, THETA goalJoint);
void tcp_pivot(double x, double z, double dq, Arr3d& state);
std::vector<double> calc_finger_pos(Arr3d state, double gripWidth);
std::vector<double> contact_norm(std::vector<double> force, double theta);

