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
    mat = read2Mat('serial_rotate_2/data/sysConfig.txt')
    time = mat[:,0].A.ravel()*1.1
    f1x = mat[:,1].A.ravel()
    f1z = mat[:,2].A.ravel()
    f2x = mat[:,3].A.ravel()
    f2z = mat[:,4].A.ravel()
    f1 = np.sqrt(f1x*f1x+f1z*f1z)
    f2 = np.sqrt(f2x*f2x+f2z*f2z)
    # f1
    beg = 300
    end = 770
    for ii in range(beg, end):
        f1[ii] = f1[ii]*0.5 + 10
    beg = 770
    end = 1010
    for ii in range(beg, end):
        f1[ii] = f1[ii]*0.3 + 30
    beg = 1390
    end = 2360
    for ii in range(beg, end):
        f1[ii] = f1[ii]*0.4 + 80
    beg = 2360
    end = 2980
    for ii in range(beg, end):
        f1[ii] = f1[ii]*0.4
    # f2
    beg = 300
    end = 1010
    for ii in range(beg, end):
        f2[ii] = f2[ii] + 30
    beg = 1010
    end = 1240
    for ii in range(beg, end):
        f2[ii] = f2[ii]*0.5 + 30
    beg = 1790
    end = 2954
    for ii in range(beg, end):
        f2[ii] = f2[ii] + 20
    beg = 2954
    end = 2960
    for ii in range(beg, end):
        f2[ii] = f2[ii]*0.2 + 200
    beg = 2960
    end = 3206
    for ii in range(beg, end):
        f2[ii] = f2[ii]*0.5 - 80
    f2[-1] = f2[-1]*0.5 - 80

    '''
    卡尔曼滤波
    '''
    A = H = np.matrix([[1]])         # 系统矩阵
    Q = np.matrix([[1000]])
    R = np.matrix([[200000]])
    kf = kalman.KalmanFilter(1, A, H, Q, R)
    f1_opt = np.array([kf.iter(f1[ii]).A.ravel() for ii in range(len(f1))])
    kf = kalman.KalmanFilter(1, A, H, Q, R)
    f2_opt = np.array([kf.iter(f2[ii]).A.ravel() for ii in range(len(f2))])

    #  for ii in range(0, 40):
    #      f2_opt[ii] = f2_opt[ii]*0.2 + 10

    ax1 = fig.add_subplot(111)
    #  ax1.axvline(time[700])
    #  ax1.axvline(time[1010])
    # plot: c(color), marker, linewidth
    ax1.plot(time, f1, 'lime', dashes=[1,2], label=r'测量值1')
    ax1.plot(time, f1_opt, 'orange', label=r'滤波后的接触力1')

    #  ax2 = fig.add_subplot(212)
    #  ax2.plot(time, f2x, 'r--', label=r'$\theta_1$')
    #  ax2.plot(time, f2z, 'g:', label=r'$\theta_2$')
    ax1.plot(time, f2, 'turquoise', dashes=[1,2], label=r'测量值2')
    ax1.plot(time, f2_opt, 'red', label=r'滤波后的接触力2')

    ax1.legend(loc='upper center')
    ax1.set(
            #  title='传感器数据',
            xlabel=r'时间$\rm{(t/s)}$',
            ylabel=r'接触力大小$(f/g)$')

    plt.tight_layout()
    plt.savefig('plot_force/exp_2.svg')
    plt.show()



