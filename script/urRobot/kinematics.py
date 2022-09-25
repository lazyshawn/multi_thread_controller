####################################################################
# 导入库
####################################################################
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import math
import sympy as sym
from sympy import sin,cos

####################################################################
# 自定义函数
####################################################################
# Global varibles
c1 = c2 = c3 = c4 = c5 = c6 = c23 = c234 = 0
s1 = s2 = s3 = s4 = s5 = s6 = s23 = s234 = 0
d1, a2, a3, d4, d5, d6 = 89.459, 425, 392.25, 109.15, 94.65, 82.3
R = np.zeros((3,3)); P = np.zeros((3,1)); Tran = np.zeros((4,4))
jcb = ijcb = tjcb = np.zeros((6,6))

cm = 1/2.54
r2d = 180/math.pi
d2r = math.pi/180

### RPY角转旋转矩阵
def rpy2Rot(rpy):
    [gamma, beta, alpha] = rpy
    rot = np.zeros((3,3))

    # X
    rot[0][0] = math.cos(alpha)*math.cos(beta)
    rot[1][0] = math.sin(alpha)*math.cos(beta)
    rot[2][0] = -1*math.sin(beta)
    # Y
    rot[0][1] = math.cos(alpha)*math.sin(beta)*math.sin(gamma) - math.sin(alpha)*math.cos(gamma)
    rot[1][1] = math.sin(alpha)*math.sin(beta)*math.sin(gamma) + math.cos(alpha)*math.cos(gamma)
    rot[2][1] = math.cos(beta)*math.sin(gamma)
    # Z
    rot[0][2] = math.cos(alpha)*math.sin(beta)*math.cos(gamma) + math.sin(alpha)*math.sin(gamma)
    rot[1][2] = math.sin(alpha)*math.sin(beta)*math.cos(gamma) - math.cos(alpha)*math.sin(gamma)
    rot[2][2] = math.cos(beta)*math.cos(gamma)

    return rot

# 更新关节角
def ur_calcJnt(q):
  global c1, c2, c3, c4, c5, c6, c23, c234, s1, s2, s3, s4, s5, s6, s23, s234
  c1 = math.cos(q[0]);  s1 = math.sin(q[0])
  c2 = math.cos(q[1]);  s2 = math.sin(q[1])
  c3 = math.cos(q[2]);  s3 = math.sin(q[2])
  c4 = math.cos(q[3]);  s4 = math.sin(q[3])
  c5 = math.cos(q[4]);  s5 = math.sin(q[4])
  c6 = math.cos(q[5]);  s6 = math.sin(q[5])
  c23 = math.cos(q[1]+q[2]); s23 = math.sin(q[1]+q[2])
  c234 = math.cos(q[1]+q[2]+q[3]); s234 = math.sin(q[1]+q[2]+q[3])
  return [c1, c2, c3, c4, c5, c6, c23, c234, s1, s2, s3, s4, s5, s6, s23, s234]

# 正运动学
def ur_kinematics(q):
  global R, P, Tran

  ur_calcJnt(q)
  hori = d4 + d6*c5
  vert = a2*c2 + a3*c23 - d5*s234 + d6*c234*s5

  P[0][0] = -s1*hori + c1*vert
  P[1][0] = c1*hori + s1*vert
  P[2][0] = d1 - a2*s2 - a3*s23 - d5*c234 - d6*s234*s5

  R[0][0] = c6*(s1*s5 + c1*c234*c5) - s234*c1*s6
  R[0][1] = -s6*(s1*s5 + c234*c1*c5) - s234*c1*c6
  R[0][2] = c234*c1*s5 - c5*s1
  R[1][0] = -c6*(c1*s5 - c234*c5*s1) - s234*s1*s6
  R[1][1] = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1
  R[1][2] = c1*c5 + c234*s1*s5
  R[2][0] = -c234*s6 - s234*c5*c6
  R[2][1] = s234*c5*s6 - c234*c6
  R[2][2] = -s234*s5

  Tran = np.concatenate((R,P), axis=1)
  Tran = np.concatenate((Tran,np.array([[0,0,0,1]])), axis=0)

  return Tran

def inv_kinematics(Tran, q):
    '''
    逆运动学
    :input: 位姿矩阵，当前角度
    :return: 目标角度
    '''
    global d1, a2, a3, d4, d5, d6
    q_jnt = np.zeros(6)
    nx = Tran[0][0]; ox = Tran[0][1]; ax = Tran[0][2]; px = Tran[0][3]
    ny = Tran[1][0]; oy = Tran[1][1]; ay = Tran[1][2]; py = Tran[1][3]
    nz = Tran[2][0]; oz = Tran[2][1]; az = Tran[2][2]; pz = Tran[2][3]
    # 关节角1 (t15_24)
    A1 = py - ay*d6; B1 = -px + ax*d6
    temp1 = math.atan2(d4,(A1*A1+B1*B1-d4*d4)**(1/2)) - math.atan2(A1,B1)
    temp2 = math.atan2(d4,-(A1*A1+B1*B1-d4*d4)**(1/2)) - math.atan2(A1,B1)
    q_jnt[0] = temp1 if abs(temp1-q[0])<abs(temp2-q[0]) else temp2
    s1 = math.sin(q_jnt[0]); c1 = math.cos(q_jnt[0])
    # 关节角5 (t15_22)
    temp1 = math.acos(-ax*s1+ay*c1)
    temp2 = -math.acos(-ax*s1+ay*c1)
    q_jnt[4] = temp1 if abs(temp1-q[4])<abs(temp2-q[4]) else temp2
    s5 = math.sin(q_jnt[4])
    # 关节角6 (t15_21)
    A6 = ny*c1-nx*s1; B6 = -oy*c1+ox*s1
    q_jnt[5] = math.atan2(-s5,0) - math.atan2(A6,B6)
    #  q_jnt[5] = math.atan2(-B6/s5,-A6/s5)
    s6 = math.sin(q_jnt[5]); c6 = math.cos(q_jnt[5])
    # 关节角3 (t14_14, t14_34)
    a2 = px*c1 +py*s1 -d5*(nx*c1*s6+ox*c1*c6+ny*s1*s6+oy*s1*c6) -d6*(ay*s1+ax*c1)
    B3 = pz -d1 -az*d6 -d5*(oz*c6+nz*s6)
    temp1 = math.acos((a2*a2+B3*B3-a2*a2-a3*a3)/(2*a2*a3))
    temp2 = -math.acos((a2*a2+B3*B3-a2*a2-a3*a3)/(2*a2*a3))
    q_jnt[2] = temp1 if abs(temp1-q[2])<abs(temp2-q[2]) else temp2
    s3 = math.sin(q_jnt[2]); c3 = math.cos(q_jnt[2])
    # 关节角2 (t14_14, t14_34)
    A2 = -(a3*a3 +2*a2*a3*c3 +a2*a2)
    B2 = -(a3*c3+a2)*a2 +a3*s3*B3
    C2 = (a3*c3+a2)*B3 +a3*s3*a2
    q_jnt[1] = math.atan2(C2/A2, B2/A2)
    # 关节角4 (t15_13, t15_33)
    a3 = -(oz*c6 + nz*s6)
    B4 = -c6*(ox*c1+oy*s1) -s6*(nx*c1+ny*s1)
    q_jnt[3] = math.atan2(B4, a3) - q_jnt[1] - q_jnt[2]
    if (q_jnt[3] > math.pi):
        q_jnt[3] = q_jnt[3] -2*math.pi
    elif (q_jnt[3] < -math.pi):
        q_jnt[3] = q_jnt[3] +2*math.pi
    return q_jnt

def ur_jacobian(q):
  global jcb, tjcb, ijcb

  ur_calcJnt(q)

  jcb[:,[0]] = np.array([-(a2*c2+a3*c23-d5*s234+d6*c234*s5)*s1 - (d4+d6*c5)*c1,
    -(d4+d6*c5)*s1+(a2*c2+a3*c23-d5*s234+d6*c234*s5)*c1, 0, 0, 0, 1]).reshape((6,1))
  jcb[:,[1]] = np.array([-c1*(a2*s2+a3*s23+d5*c234+d6*s234*s5),
    -s1*(a2*s2+a3*s23+d5*c234+d6*s234*s5),
    -a2*c2-a3*c23+d5*s234-d6*c234*s5,
    -s1, c1, 0]).reshape(6,1)
  jcb[:,[2]] = np.array([-c1*(a3*s23+d5*c234+d6*s234*s5),
    -s1*(a3*s23+d5*c234+d6*s234*s5),
    -a3*c23+d5*s234-d6*c234*s5,
    -s1, c1, 0]).reshape((6,1))
  jcb[:,[3]] = np.array([-c1*(d5*c234+d6*s234*s5),
    -s1*(d5*c234+d6*s234*s5),
    d5*s234-d6*c234*s5,
    -s1, c1, 0]).reshape((6,1))
  jcb[:,[4]] = np.array([d6*(s1*s5 + c1*c234*c5),
    d6*(-c1*s5 + s1*c234*c5),
    -d6*s234*c5, -c1*s234, -s1*s234, -c234]).reshape((6,1))
  jcb[:,[5]] = np.array([0, 0, 0,
    -s1*c5 + c1*c234*s5, c1*c5 + s1*c234*s5, -s234*s5]).reshape((6,1))

  tjcb = np.transpose(jcb)
  ijcb = np.linalg.inv(jcb)

# 雅克比
def jacobian(twist, q):
    c4 = cos(q[3]); s4 = sin(q[3])
    c34 = cos(q[2]+q[3]); s34 = sin(q[2]+q[3])
    jb4 = sym.Matrix([-1, -82.3, 94.65])
    jb3 = sym.Matrix([-1, jb4[1]+392.25*c4, jb4[2]-392.25*s4])
    #  jb3 = sym.Matrix([0, -1, 0, 392.25*c4-82.3, 0, 94.65-392.25*s4])
    jb2 = sym.Matrix([-1, jb3[1]+425*c34, jb3[2]-425*s34])
    #  jb2 = sym.Matrix([0, -1, 0, 392.25*c4-82.3+425*c34, 0, 94.65-392.25*s4-425*s34])
    jcb = sym.Matrix([[jb2, jb3, jb4]])**(-1)
    #  sym.print_latex(jcb*twist)
    return jcb*twist



