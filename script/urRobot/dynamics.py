####################################################################
# 导入库
####################################################################
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import sympy as sym
from sympy import sin, cos, sqrt, pprint
import cv2
from calibration_sensor import read2Mat


####################################################################
# 设置字体
####################################################################
# 正常显示中文标签
plt.rcParams['font.sans-serif'] = ['SimHei']
# 正常显示负号
plt.rcParams['axes.unicode_minus'] = False

DH_D1 = 43.459; DH_A2 = 425.00; DH_A3 = 392.25; DH_D4 = 109.15; DH_D5 = 94.65;
DH_D6 = 271.3  # 有传感器

def plane_kinematics(joint):
    c1 = cos(joint[0]);  s1 = sin(joint[0]);
    c2 = cos(joint[1]);  s2 = sin(joint[1]);
    c3 = cos(joint[2]);  s3 = sin(joint[2]);
    c4 = cos(joint[3]);  s4 = sin(joint[3]);
    c5 = cos(joint[4]);  s5 = sin(joint[4]);
    c6 = cos(joint[5]);  s6 = sin(joint[5]);
    c23 = cos(joint[1]+joint[2]); s23 = sin(joint[1]+joint[2]);
    c234 = cos(joint[1]+joint[2]+joint[3]); s234 = sin(joint[1]+joint[2]+joint[3]);
    c34 = cos(joint[2]+joint[3]); s34 = sin(joint[2]+joint[3]);
    q2 = joint[1]; q3 = joint[2]; q4 = joint[3];
    state = np.array([0,0,0], dtype=float)
    state[0] = DH_A2*c2 + DH_A3*c23 - DH_D5*s234 - DH_D6*c234;
    state[1] = DH_D1 - DH_A2*s2 - DH_A3*s23 - DH_D5*c234 + DH_D6*s234;
    state[2] = -(q2 + q3 + q4);
    return state

def plane_inv_kinematics():
    return


'''
// 平面内运动的正运动学
// state[0], state[1]: elk 坐标系原点 x、z 坐标
// state[2]: elk 坐标系 z 轴方向与 base 坐标系 x 轴夹角(q_elk)
bool plane_kinematics(THETA jointState, std::array<double,3>& state) {
  calcJnt(jointState);
  state[0] = DH_A2*c2 + DH_A3*c23 - DH_D5*s234 - DH_D6*c234;
  state[1] = DH_D1 - DH_A2*s2 - DH_A3*s23 - DH_D5*c234 + DH_D6*s234;
  state[2] = -(q2 + q3 + q4);
  return true;
}

// 平面内运动的逆解(降低为三自由度)
THETA plane_inv_kinematics(std::array<double,3> state) {
  double x = state[0], z = state[1], joint = -1*state[2];
  double c234 = cos(q), s234 = sin(q), c3, s3;
  double A = x + DH_D5*s234 + DH_D6*c234;
  double B = -z - DH_D5*c234 + DH_D6*s234 + DH_D1;

  // 限制工作空间后，多解问题已忽略
  THETA theta = {0, 0, 0, 0, -M_PI/2, M_PI/2};
  theta[2] = acos((A*A+B*B-DH_A2*DH_A2-DH_A3*DH_A3)/(2*DH_A2*DH_A3));
  c3 = cos(theta[2]); s3 = sin(theta[2]);
  double delta = DH_A2*DH_A2 + 2*DH_A2*DH_A3*c3 + DH_A3*DH_A3;
  double A2 = (DH_A2+DH_A3*c3)*A + DH_A3*s3*B;
  double B2 = -DH_A3 * s3 * A + (DH_A2 + DH_A3 * c3) * B;
  theta[1] = atan2(B2 / delta, A2 / delta);
  theta[3] = q - theta[1] - theta[2];
  // Wrap qJoint[3] to (-pi,pi]
  swap_joint(theta[3]);
  return theta;
}
'''
