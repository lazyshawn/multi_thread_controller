#include "master/master_interface.h"

THETA rotAftJnt = {0, - 1.76592, 2.52261, -2.43964, -1.57081,1.5708};
// 翻转圆顶矩形 - 开环
// THETA rotBegJnt_2 = {0, -81.3037, 124.034, -113.608, -90, 90};
THETA rotBegJnt_2 = {0, -76.319, 117.504, -113.314, -90, 90};

Mat4d zeroPose, rlsPose;
std::vector<double> data = std::vector<double>(6,0);
extern Task task;

void main_menu() {
  THETA joint;
  Mat4d pose;
  while (miniROS::OK()) {
    int key = scanKeyboard();
    switch (key) {
      case 'h':
        zeroPose << 1, 0, 0, 350, 0, -1, 0, DH_D4, 0, 0, -1, 0, 0, 0, 0, 1;
        ur5e::go_to_pose(zeroPose, 8);
        break;
      case 'r':
        task = Task::teleopr;
        wsgConfig.push({58,40});
        ROS_INFO("You're in ur5e teleoperate mode.");
        ur5e::teleoperate();
        break;
      case 'f':
        ROS_INFO("Ready to calibrate force sensor.");
        calib_force_sensor();
        break;
      // 任务1: 翻转魔方并抓取
      case '1':
        wsgConfig.push({78,40});
        pose << 1,0,0,350, 0,-1,0,DH_D4, 0,0,-1,-20, 0,0,0,1;
        ur5e::go_to_pose(pose, 5);
        ur5e::wait_path_clear();
        task = Task::press;
        break;
      // 任务2: 翻转圆顶矩形
      case '2':
        wsgConfig.push({52,40});
        pose << 1,0,0,416, 0,-1,0,DH_D4, 0,0,-1,-60, 0,0,0,1;
        ur5e::go_to_pose(pose, 5);
        // joint = plane_inv_kinematics({418, -30, 70*deg2rad});
        // ur5e::go_to_joint(joint, 5);
        ur5e::wait_path_clear();
        task = Task::task2_press;
        break;
      // 任务3: 翻转圆顶矩形2
      case '3':
        // wsgConfig.push({52,40});
        // pose << 1,0,0,436, 0,-1,0,DH_D4, 0,0,-1,-52, 0,0,0,1;
        // ur5e::go_to_pose(pose, 5);

        pose << 1,0,0,410, 0,-1,0,DH_D4, 0,0,-1,-52, 0,0,0,1;
        ur5e::go_to_pose(pose, 1);
        ur5e::wait_path_clear();
        pose << 1,0,0,434, 0,-1,0,DH_D4, 0,0,-1,-52, 0,0,0,1;
        ur5e::go_to_pose(pose, 2);

        ur5e::wait_path_clear();
        task = Task::task2_press_2;
      case '4':
        wsgConfig.push({53,40});
        // wsgConfig.push({58,40});
        pose << 1,0,0,362, 0,-1,0,DH_D4, 0,0,-1,-57, 0,0,0,1;
        // pose << 1,0,0,362, 0,-1,0,DH_D4, 0,0,-1,-52, 0,0,0,1;
        ur5e::go_to_pose(pose, 5);
        ur5e::wait_path_clear();
        task = Task::task4_press;
        // task = Task::task4_rotate;
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

void calib_force_sensor() {
  Recorder calibRecord1(4), calibRecord2(4);
  std::vector<double> data1(4,0), data2(4,0);
  bool flag = false;

  // 读取传感器数据
  std::vector<double> force = dydwShared.copy_data();
  // 读取机械臂关节角
  THETA curJoint = ur5eShared.copy_data();
  THETA jntCmd = {0, 0, 0, 0, -M_PI/2, M_PI/2}, refJoint = curJoint;
  Arr3d curState, refState;

  while (miniROS::OK()) {
    if (flag) break;
    int key = scanKeyboard();
    switch (key) {
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
      case 10: // <ENTER> 记录一组数据
        curJoint = ur5eShared.copy_data();
        plane_kinematics(curJoint, curState);
        force = dydwShared.copy_data();
        for (int i=0; i<4; ++i) {
          std::cout << force[i] << " ";
        }
        std::cout << std::endl;
        // 第一行
        data1[0] = force[0];
        data1[1] = -cos(curState[2]);  // x2
        data1[2] = 1; data1[3] = 0;
        calibRecord1.data_record(data1);
        // 第二行
        data1[0] = force[1];
        data1[1] = -sin(curState[2]);  // z2
        data1[2] = 0; data1[3] = 1;
        calibRecord1.data_record(data1);
        // 传感器2
        data2[0] = force[2];
        data2[1] = cos(curState[2]);   // -x2
        data2[2] = 1; data2[3] = 0;
        calibRecord2.data_record(data2);
        data2[0] = force[3];
        data2[1] = -sin(curState[2]);  // z2
        data2[2] = 0; data2[3] = 1;
        calibRecord2.data_record(data2);
        break;
      case 'p':
        curJoint = ur5eShared.copy_data();
        plane_kinematics(curJoint, curState);
        force = dydwShared.copy_data();
        actual_force(force,curState[2]);
        for (int i=0; i<4; ++i) {
          std::cout << force[i] << " ";
        }
        std::cout << std::endl;
        break;
      case 27: case 'q': // <ESC>
        ROS_INFO("Back to main menu.");
        flag = true;
        break;
      default:
        break;
    }
  }
  calibRecord1.data_export("calib_f1.txt");
  calibRecord2.data_export("calib_f2.txt");
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

