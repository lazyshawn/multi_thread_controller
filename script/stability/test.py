####################################################################
# 导入库
####################################################################
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.patches import Circle, PathPatch
import mpl_toolkits.mplot3d.art3d as art3d
from mpl_toolkits.axes_grid1 import make_axes_locatable
import sympy as sym
from sympy import sin, cos, sqrt

from stablility_analysis import *


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
def test():
    mu = 0.2
    G = [[-1,-mu,-1-2*mu], [-1,mu,-1+2*mu], [mu,-1,-1-2*mu], [-mu,-1,-1+2*mu], [-mu,1,-1-2*mu], [mu,1,-1+2*mu]]
    plot_wrench(G)

    #  stablePoints = sample_stablepoint()

# ==========================================================================
# @Brief : 线性截取 colormap
# @Param : cmapIn - colormap 名称, [minval, maxval] - 色值区间, n - 截取点数
# @Ref   : truncate_colormap: https://stackoverflow.com/a/18926541
# ==========================================================================
def truncate_colormap(cmapIn='jet', minval=0.0, maxval=1.0, n=100):
    '''truncate_colormap(cmapIn='jet', minval=0.0, maxval=1.0, n=100)'''    
    cmapIn = plt.get_cmap(cmapIn)

    new_cmap = mpl.colors.LinearSegmentedColormap.from_list(
        'trunc({n},{a:.2f},{b:.2f})'.format(n=cmapIn.name, a=minval, b=maxval),
        cmapIn(np.linspace(minval, maxval, n)))
    return new_cmap

def stablePoints_plot_in3d():
    data = load_from_file("./data/stablePoints.txt")
    xx = np.array([data[i][0] for i in range(len(data))])
    yy = np.array([data[i][1] for i in range(len(data))])
    zz = np.array([data[i][2] for i in range(len(data))])

    #  fig = plt.figure(tight_layout=True)
    cm = 1/2.54
    fig = plt.figure(figsize=(17.6*cm, 10*cm), constrained_layout=True)
    gs = gridspec.GridSpec(1, 2)
    ax = []

    # 3d-scatter-colorbar: https://stackoverflow.com/a/66026854
    mycmap = truncate_colormap("GnBu", 0.3, 1, 90)
    ax.append(fig.add_subplot(gs[0,0],projection='3d'))
    ax.append(fig.add_subplot(gs[0,1],projection='3d'))

    # https://stackoverflow.com/a/13784887
    ax[0].scatter(xx, yy, zz, c=zz, cmap=mycmap, marker='o', s=50)
    p = ax[1].scatter(xx, yy, zz, c=zz, cmap=mycmap, marker='o', s=50)
    #  fig.colorbar(p, ax=ax[1], fraction=0.030, pad=0.22)
    fig.subplots_adjust(right=0.8)
    cbar_ax = fig.add_axes([0.88, 0.15, 0.02, 0.7])
    fig.colorbar(p, cax=cbar_ax)

    ax[0].set_xticks(np.arange(min(xx), max(xx)+2, 14))
    ax[0].set_yticks(np.arange(min(yy), max(yy)+2, 14))
    ax[1].set_xticks(np.arange(min(xx), max(xx)+2, 14))
    ax[1].set_yticks(np.arange(min(yy), max(yy)+2, 14))
    # https://stackoverflow.com/a/43083811
    xt = ax[0].get_xticks() 
    yt = ax[0].get_yticks() 
    #  xt = np.append(xt,65)
    #  xtl = xt.tolist()
    xtl = ["B", " ", " ", " ", "C"]
    ytl = ["C", " ", " ", " ", "D"]
    ax[0].set_xticklabels(xtl)
    ax[0].set_yticklabels(ytl)
    ax[1].set_xticklabels(xtl)
    ax[1].set_yticklabels(ytl)
    #  ax[0].set_title("稳定状态空间的三维视角")

    # set properties
    for ii in range(len(ax)):
        ax[ii].set_xlabel(r'手指#1位置 $c_1$(mm)')
        ax[ii].set_ylabel(r'手指#2位置 $c_2$(mm)')
        ax[ii].set_zlabel(r'物体倾斜角度 $\theta$($^o$)')
        ax[ii].set_xlim(56, 112)
        ax[ii].set_ylim(112, 168)
        ax[ii].set_zlim(0, 90)

    plt.savefig('./data/sample_stablepoint.svg')
    plt.show()


if __name__ == "__main__":
    stablePoints_plot_in3d()

