####################################################################
# 导入库
####################################################################
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import sympy as sym
from sympy import sin, cos, sqrt, pprint
import cv2
from calibration_sensor import read2Mat, writeMat
from dynamics import plane_kinematics, plane_inv_kinematics


####################################################################
# 设置字体
####################################################################
# 正常显示中文标签
plt.rcParams['font.sans-serif'] = ['SimHei']
# 正常显示负号
plt.rcParams['axes.unicode_minus'] = False


####################################################################
# 自定义函数
####################################################################
def extract_pic(path, fname):
    '''
    逐帧拆分视频
    :fname: 视频路径
    '''
    video = cv2.VideoCapture(path + "/" + fname)
    cnt = 0
    sep = 1
    if not video.isOpened():
        print("Open video failed.")
        return
    rval = True
    while rval:
        rval, frame = video.read()
        if not rval:
            break
        if cnt % sep == 0:
            cv2.imwrite(path + "/tmp/" +
                        str('%06d' % (cnt/sep))+'.jpg', frame)
        cnt = cnt + 1


def check_timestamp(mat, sep=0.01):
    '''
    检查时间周期
    :mat: 待检查的矩阵
    :dec: 四舍五入的小数点位数
    '''
    for ii in range(len(mat)-1):
        if (np.round(mat[[ii+1], 0], decimals=2) - np.round(mat[[ii], 0], decimals=2) > sep*1.1):
            print(mat[[ii], 0], mat[[ii+1], 0])
            print("---------\n")


def merge_sys_state(folder, saveFlag=False):
    '''
    融合传感器和机械臂数据
    :folder: 数据文件夹路径
    :saveFlag: 是否储存结果
    '''
    # 读传感器数据
    force = read2Mat(folder + "/data/dydwData.txt")
    # 读机械臂数据
    joint = read2Mat(folder + "/data/jointState.txt")

    # 时间戳近似到0.01s
    force[:, 0] = np.round(force[:, 0], decimals=2)
    joint[:, 0] = np.round(joint[:, 0], decimals=2)
    idx = 0    # 机械臂状态索引
    stateList = []; forceList = []
    numF = len(force)
    numJ = len(joint)
    for ii in range(numF):
        for jj in range(len(joint)-idx):
            if force[ii, 0] == joint[idx+jj, 0]:
                state = np.matrix(plane_kinematics(joint[idx+jj, 1:].A.ravel()))
                idx = idx+jj
                break
            # 如果没找到对应的则使用上次的值
            if jj == len(joint)-idx-1:
                state = np.matrix(plane_kinematics(joint[idx, 1:].A.ravel()))
        if idx == numJ:
            state = np.matrix(plane_kinematics(joint[-1, 1:].A.ravel()))
        # 机械臂状态
        stateList.append(state)
        # 实际力
        #  f0 = [0.500275, 32.98784, -0.13148458, 30.568432] # rotate_and_grasp_cube_2
        f0 = [0.500275, -32.98784, -0.13148458, 30.568432] # serial_rotate_2
        G = [32.91205, 30.928371]
        fy1 = force[ii,1] - f0[0] + cos(state[0,2])*G[0]
        fz1 = force[ii,2] - f0[1] + sin(state[0,2])*G[0]
        fy2 = force[ii,3] - f0[2] - cos(state[0,2])*G[1]
        fz2 = force[ii,4] - f0[3] + sin(state[0,2])*G[1]
        forceList.append([force[ii,0],fy1,fz1,fy2,fz2])

    state = np.array(stateList).reshape(numF, 3)
    print(forceList)
    actF = np.array(forceList).reshape(numF,5)
    #  nf = force[0:numF]
    sysConfig = np.column_stack((actF, state))
    if (saveFlag):
        writeMat(sysConfig, folder + '/data/sysConfig.txt')
    return sysConfig


if __name__ == "__main__":
    #  merge_sys_state('./rotate_and_grasp_cube_2', saveFlag=True)
    #  merge_sys_state('./corner_block_grasp', saveFlag=True)
    #  extract_pic("./corner_block_grasp","sideVideo.avi")
    merge_sys_state('./serial_rotate_2', saveFlag=True)
    #  extract_pic("./serial_rotate_2","sideVideo.avi")
