#include "master/interface.h"

void devices_home() {
  ur5e::go_home(3);
}

void pick_and_place(double hoverHeight) {
  Mat4d elk2base, obj2elk, obj2base;
  Mat4d hoverPose;

  /* 获取系统状态 */
  // 机械臂末端位置
  THETA jointState = urConfig.get_state();
  // ur5e::ur_kinematics(jointState, elk2base);
  std::cout << elk2base << std::endl;
  // 物体在相机坐标系下的位姿
  ObjState::Data objStateData = objState.get_data();
  obj2elk = objStateData.obj2elk;
  obj2base = elk2base * obj2elk;
  std::cout << obj2base << std::endl;

  hoverPose = obj2base;
  hoverPose(2,3) = hoverHeight;
  ur5e::go_to_pose(hoverPose, 3);
}

