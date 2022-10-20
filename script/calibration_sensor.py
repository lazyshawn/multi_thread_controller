####################################################################
# 导入库
####################################################################
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import sympy as sym
from sympy import sin, cos, sqrt, pprint


####################################################################
# 设置字体
####################################################################
# 正常显示中文标签
plt.rcParams['font.sans-serif'] = ['SimHei']
# 正常显示负号
plt.rcParams['axes.unicode_minus'] = False

def read2Mat(fname):
    mat = np.mat(np.loadtxt(fname, dtype=np.float32))
    return mat

def lsm(y,A):
    ks = np.linalg.inv(A.transpose()*A)*A.transpose()*f
    return ks


if __name__ == "__main__":
    mat = read2Mat('../build/data/calib_f1.txt')
    f = mat[:,0]
    A = mat[:,1:]
    ks = lsm(f,A)
    print(ks.transpose())
    mat = read2Mat('../build/data/calib_f2.txt')
    f = mat[:,0]
    A = mat[:,1:]
    ks = lsm(f,A)
    print(ks.transpose())

