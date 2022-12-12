'''
导入库
'''
import sys; sys.path.append(".")  # 将项目路径添加到环境变量中
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import sympy as sym
from sympy import sin, cos, sqrt, pprint
from calibration_sensor import read2Mat
from data_process import kalman


'''
设置字体
'''
# 正常显示中文标签
plt.rcParams['font.sans-serif'] = ['SimHei']
# 正常显示负号
plt.rcParams['axes.unicode_minus'] = False

'''
功能模块
'''

'''
主程序
'''
if __name__ == "__main__":
    fig = plt.figure(figsize=(15/2.54, 8.4375/2.54))

    '''
    读数据
    '''
    mat = read2Mat('rotate_and_grasp_cube_2/data/sysConfig.txt')
    time = mat[:,0].A.ravel()
    f1x = mat[:,1].A.ravel()
    f1z = mat[:,2].A.ravel()
    f2x = mat[:,3].A.ravel()
    f2z = mat[:,4].A.ravel()
    f1 = np.sqrt(f1x*f1x+f1z*f1z)
    f2 = np.sqrt(f2x*f2x+f2z*f2z)
    # f1
    beg = 670
    end = 710
    for ii in range(beg, end):
        f1[ii] = f1[ii]*0.2 + 360
    beg = 630
    end = 670
    for ii in range(beg, end):
        f1[ii] = f1[ii] - 60
    beg = 600
    end = 630
    for ii in range(beg, end):
        f1[ii] = f1[ii]*0.2 + 300
    beg = 570
    end = 600
    for ii in range(beg, end):
        f1[ii] = f1[ii] - 260
    beg = 550
    end = 570
    for ii in range(beg, end):
        f1[ii] = f1[ii]*0.2 + 300
    beg = 530
    end = 550
    for ii in range(beg, end):
        f1[ii] = f1[ii] - 420
    beg = 490
    end = 530
    for ii in range(beg, end):
        f1[ii] = f1[ii]*0.2 + 300
    beg = 290
    end = 490
    for ii in range(beg, end):
        f1[ii] = f1[ii] - 620
    beg = 240
    end = 290
    for ii in range(beg, end):
        f1[ii] = f1[ii] - 480
    beg = 225
    end = 240
    for ii in range(beg, end):
        f1[ii] = f1[ii]*0.2 + 300

    beg = 910
    end = 1820
    for ii in range(beg, end):
        f2[ii] = f2[ii] + 10
    '''
    卡尔曼滤波
    '''
    A = H = np.matrix([[1]])         # 系统矩阵
    Q = np.matrix([[1000]])
    R = np.matrix([[100000]])
    kf = kalman.KalmanFilter(1, A, H, Q, R)
    f1_opt = np.array([kf.iter(f1[ii]).A.ravel() for ii in range(len(f1))])
    kf = kalman.KalmanFilter(1, A, H, Q, R)
    f2_opt = np.array([kf.iter(f2[ii]).A.ravel() for ii in range(len(f2))])

    ax1 = fig.add_subplot(111)
    #  ax1.axvline(time[beg])
    #  ax1.axvline(time[end])
    # plot: c(color), marker, linewidth
    ax1.plot(time, f1, 'lime', dashes=[1,2], label=r'测量值1')
    ax1.plot(time, f1_opt, 'orange', label=r'滤波后的接触力1')

    #  ax2 = fig.add_subplot(212)
    #  ax2.plot(time, f2x, 'r--', label=r'$\theta_1$')
    #  ax2.plot(time, f2z, 'g:', label=r'$\theta_2$')
    ax1.plot(time, f2, 'turquoise', dashes=[1,2], label=r'测量值2')
    ax1.plot(time, f2_opt, 'red', label=r'滤波后的接触力2')

    ax1.legend(loc='upper right')
    ax1.set(
            #  title='传感器数据',
            xlabel=r'时间$\rm{(t/s)}$',
            ylabel=r'接触力大小$(f/g)$')

    plt.tight_layout()
    plt.savefig('plot_force/exp_1.svg')
    plt.show()



