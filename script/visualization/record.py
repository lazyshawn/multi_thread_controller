####################################################################
# 导入库
####################################################################
import sys
sys.path.append(".")  # 将项目路径添加到环境变量中
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import urRobot.kinematics as kinematics

####################################################################
# 设置字体
####################################################################
# 正常显示中文标签
plt.rcParams['font.sans-serif'] = ['SimHei']
# 正常显示负号
plt.rcParams['axes.unicode_minus'] = False


####################################################################
# 读入数据
####################################################################
# 相对路径从项目的根目录开始
data_theta  = np.loadtxt("../build/data/jointState.txt", dtype=float)
#  data_theta  = np.loadtxt("../data/data.curtheta", dtype=float)
#  data_refTheta  = np.loadtxt("../data/data.reftheta", dtype=float)
#  data_curPos = np.loadtxt("../data/data.curpos", dtype=float)
time = data_theta[:,0]


####################################################################
# 修改配置文件 | Change default rc settings
####################################################################
#  print(mpl.matplotlib_fname())
#  mpl.rcParams['lines.linewidth'] = 1.5
# X、Y轴标签字体大小
#  mpl.rcParams['xtick.labelsize'] = 10.5
#  mpl.rcParams['ytick.labelsize'] = 10.5
# X、Y轴刻度标签字体大小
#  mpl.rcParams['axes.labelsize'] = 10.5

####################################################################
# 创建图框
####################################################################
# fig:
# 1 inch == 2.54 cm, 单栏8.3 cm(3.27 inch), 双栏17.6 cm(6.9 inch)
# figsize=(x inch, y inch), int dpi,
# facecolor=(r,g,b), edgecolor=(r,g,b),
# bool frameon.
cm = 1/2.54
r2d = 180/math.pi
d2r = math.pi/180
fig = plt.figure(figsize=(17.6*cm, 20*cm))
data = np.linspace(0,10,100)


####################################################################
# 绘制子图
####################################################################
### 子图1: 二维曲线图
time = data_theta[:, 0]
theta = np.array([range(len(data_theta)) for i in range(6)], dtype=float)
for ii in range(6):
    theta[ii, :] = data_theta[:, ii+1]

ax1 = fig.add_subplot(221)
# plot: c(color), marker, linewidth
ax1.plot(time, theta[0], 'r--', label=r'$\theta_1$')
ax1.plot(time, theta[1], 'g:', label=r'$\theta_2$')
ax1.plot(time, theta[2], 'c',   label=r'$\theta_3$')
ax1.plot(time, theta[3], 'y--',   label=r'$\theta_4$')
ax1.plot(time, theta[4], 'm:',   label=r'$\theta_5$')
ax1.plot(time, theta[5], 'b',   label=r'$\theta_6$')

# legend: 
# loc = upper/lower/center, right/left/center, best(default)
ax1.legend(loc='lower right')
ax1.grid(True)
# set:
# ax.set_foo(bar) == ax.set(foo=bar)
# title, xlabel, xlim, xticks, xticklabels
ax1.set(title  = '关节角变化曲线',
        xlabel = r'时间$\rm{(t/s)}$',
        ylabel = r'角度$\rm{(deg/^o)}$')

####################################################################
### 子图2: 末端位置轨迹图
#  hnd_x = data_curPos[:,1]
#  hnd_y = data_curPos[:,2]
#  hnd_z = data_curPos[:,3]
#  # r-gama; p-beta; y-alpha;
#  ori_r = data_curPos[:,6]*d2r
#  ori_p = data_curPos[:,5]*d2r
#  ori_y = data_curPos[:,4]*d2r
#  # 方向向量长度
#  vec_len = 200
#
#  # 起点坐标系 [XYZ]
#  rot_beg = rpy2Rot([ori_r[0], ori_p[0], ori_y[0]])
#  # 起点坐标系 [XYZ]
#  rot_end = rpy2Rot([ori_r[-1], ori_p[-1], ori_y[-1]])
#
#  ax2 = fig.add_subplot(222,projection='3d')
#  # 末端点轨迹
#  ax2.plot(hnd_x, hnd_y, hnd_z, 'r:')
#  # 特殊点
#  ax2.scatter(hnd_x[0], hnd_y[0], hnd_z[0], marker='*', color='darkorange',
#          linewidth=2.5, label='Start point')
#  ax2.scatter(hnd_x[-1], hnd_y[-1], hnd_z[-1], marker='*', color='m',
#          linewidth=2.5, label='End point')
#  ax2.scatter(0,0,0, marker='d', color='k', linewidth=5, label='Base')
#  # 原点坐标系
#  ax2.quiver(0,0,0,1,0,0,length=vec_len,normalize=False)
#  ax2.quiver(0,0,0,0,1,0,length=vec_len, color='g' , normalize=False)
#  ax2.quiver(0,0,0,0,0,1,length=vec_len, color='r' , normalize=False)
#  # 起点姿态
#  ax2.quiver(hnd_x[0], hnd_y[0], hnd_z[0],rot_beg[0][0], rot_beg[1][0], rot_beg[2][0],
#          length=vec_len,normalize=False,alpha=0.5)
#  ax2.quiver(hnd_x[0], hnd_y[0], hnd_z[0],rot_beg[0][1], rot_beg[1][1], rot_beg[2][1],
#          length=vec_len, color='g' , normalize=False,alpha=0.5)
#  ax2.quiver(hnd_x[0], hnd_y[0], hnd_z[0],rot_beg[0][2], rot_beg[1][2], rot_beg[2][2],
#          length=vec_len, color='r' , normalize=False,alpha=0.5)
#  # 终点姿态
#  ax2.quiver(hnd_x[-1], hnd_y[-1], hnd_z[-1],rot_end[0][0], rot_end[1][0], rot_end[2][0],
#          length=vec_len,normalize=False)
#  ax2.quiver(hnd_x[-1], hnd_y[-1], hnd_z[-1],rot_end[0][1], rot_end[1][1], rot_end[2][1],
#          length=vec_len, color='g' , normalize=False)
#  ax2.quiver(hnd_x[-1], hnd_y[-1], hnd_z[-1],rot_end[0][2], rot_end[1][2], rot_end[2][2],
#          length=vec_len, color='r' , normalize=False)
#
#  ax2.set(xlabel = r'X/mm', ylabel = r'Y/mm', zlabel = r'Z/mm',
#          title = '末端位置轨迹图')
#  ax2.legend(loc='best')
#  ax2.set_xlim(0, 800)
#  ax2.set_ylim(-400,400)
#  ax2.set_zlim(0, 800)
#
#  ####################################################################
#  ### 子图3: 参考角度位置
#  ax3 = fig.add_subplot(223,projection='3d')
#
#  ax3.scatter(hnd_x[0], hnd_y[0], hnd_z[0], marker='*', color='darkorange',
#          linewidth=2.5, label='Start point')
#  ax3.scatter(hnd_x[-1], hnd_y[-1], hnd_z[-1], marker='*', color='m',
#          linewidth=2.5, label='End point')
#  ax3.ticklabel_format(useOffset=False, style='plain')
#  xmin, xmax = ax3.get_xlim()
#  ymin, ymax = ax3.get_ylim()
#  zmin, zmax = ax3.get_zlim()
#  ax3.plot(hnd_x, hnd_y, hnd_z, label='3D path')
#  ax3.plot(hnd_y, hnd_z, zs=xmin, zdir='x', linestyle=':', label='(y, z)')
#  ax3.plot(hnd_x, hnd_z, zs=ymax, zdir='y', linestyle=':', label='(x, z)')
#  ax3.plot(hnd_x, hnd_y, zs=zmin, zdir='z', linestyle=':', label='(x, y)')
#
#  ax3.set(xlabel = r'X/mm', ylabel = r'Y/mm', zlabel = r'Z/mm',
#          title = '末端位置轨迹图')
#  ax3.legend(loc='best')
#
#
#  ####################################################################
#  ### 子图4: 末端坐标系姿态
#  ax4 = fig.add_subplot(224,projection='3d')
#
#  # 原点坐标系
#  ax4.quiver(0,0,0,1,0,0,length=vec_len,normalize=False)
#  ax4.quiver(0,0,0,0,1,0,length=vec_len, color='g' , normalize=False)
#  ax4.quiver(0,0,0,0,0,1,length=vec_len, color='r' , normalize=False)
#  # 起点姿态
#  axis_x = ax4.quiver([], [], [], [], [], [],
#          length=vec_len,normalize=False,alpha=0.5)
#  #  axis_y, = ax4.quiver([], [], [],[], [], [],
#  #          length=vec_len, color='g' , normalize=False,alpha=0.5)
#  #  axis_z, = ax4.quiver([], [], [],[], [], [],
#  #          length=vec_len, color='r' , normalize=False,alpha=0.5)
#
#  ax4.set(xlabel = r'X/mm', ylabel = r'Y/mm', zlabel = r'Z/mm',
#          title = '末端位置轨迹图')
#  ax4.legend(loc='best')
#  ax4.set_xlim(0, 800)
#  ax4.set_ylim(-400,400)
#  ax4.set_zlim(0, 800)


####################################################################
# 后处理
####################################################################
# make sure that the plots fit nicely in your figure
plt.tight_layout()
#  plt.savefig('../foo.svg')
plt.show()

