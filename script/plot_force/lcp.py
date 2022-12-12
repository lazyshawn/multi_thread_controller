'''
导入库
'''
import sys; sys.path.append(".")  # 将项目路径添加到环境变量中
import numpy as np
from numpy.linalg import inv, pinv
import matplotlib as mpl
import matplotlib.pyplot as plt
import sympy as sym
from sympy import sin, cos, sqrt, pprint, floor
from calibration_sensor import read2Mat


def solve_lcp(A, b):
    # enumerative LCP solver
    n = len(A)
    N = 2**n
    errormin = 1e99
    for i in range(1,N+1):
        # 构建序列n维 01 排列
        v = [(floor((i-1)*(2**j) % 2)) for j in range(1-n,1)]
        isone = np.nonzero(v)
        V = np.diag(v)
        As = V*A*V.T
        bs = V*b
        x = np.zeros((n,1))
        if (As.sum()):
            print(pinv(A))
            #  x[isone] = xs
        else:
            xs = []
        y = A*x+b

if __name__ == "__main__":
    m = 5
    n = 10
    d = 10
    xo = 1
    yo = 2
    qo = 3.2
    xf = 3
    yf = 2
    qf = 1.2

    M = np.diag((0,0,10,0,0,0,0))
    h = np.matrix([[0],[0],[10],[0],[0],[0],[0]])
    mu = np.diag((0.1, 0.1, 0.1))

    dgn1 = np.matrix([sin(qo), -cos(qo), ( -xf*cos(qo)-yf*sin(qo)+xo*cos(qo)+yo*sin(qo)+d*cos(qf)*cos(qo)+d*sin(qf)*sin(qo) ),\
           -sin(qo), cos(qo), ( -d*sin(qf)*sin(qo)-d*cos(qf)*cos(qo) ), cos(qf)*sin(qo)-sin(qf)*cos(qo)])
    dgn2 = np.matrix([-cos(qo), -sin(qo), ( -xf*sin(qo)+yf*cos(qo)+xo*sin(qo)-yo*cos(qo)-d*cos(qf)*sin(qo)+d*sin(qf)*cos(qo) ),\
            cos(qo), sin(qo), -d*sin(qf)*cos(qo)+d*cos(qf)*sin(qo), cos(qf)*cos(qo)+sin(qf)*sin(qo)])
    dgn3 = np.matrix([0, 1, -m*cos(qo)+m*sin(qo), 0, 0, 0, 0])
    wn = np.vstack((dgn1,dgn2,dgn3)).T

    dgt1 = np.matrix([-cos(qo), -sin(qo), ( xo*sin(qo)-yo*cos(qo)-xf*sin(qo)+yf*cos(qo)+d*cos(qf)*sin(qo)-d*sin(qf)*cos(qo) ),\
            cos(qo), sin(qo), d*sin(qf)*cos(qo)-d*cos(qf)*sin(qo), -cos(qf)*cos(qo)-sin(qf*sin(qo))])
    dgt2 = np.matrix([-sin(qo), -cos(qo),\
            ( xf*cos(qo)-xo*cos(qo)+d*cos(qf)*cos(qo)+2*m*sin(qo)*sin(qo)-2*m*cos(qo)*cos(qo)-4*n*sin(qo)*cos(qo)-yf*sin(qo)+yo*sin(qo)-d*sin(qf)*sin(qo) ),\
            sin(qo), cos(qo), -d*sin(qf)*sin(qo)+d*cos(qf)*cos(qo), cos(qf)*sin(qo)+sin(qf)*cos(qo)])
    dgt3 = np.matrix([1, 0, m*sin(qo)+m*cos(qo), 0, 0, 0, 0])
    wt = np.vstack((dgt1,dgt2,dgt3)).T

    a1 = np.hstack((wn.T*pinv(M)*(wn-wt*mu), wn.T*pinv(M)*wt, np.zeros((3,3))))
    a2 = np.hstack((wt.T*pinv(M)*(wn-wt*mu), wt.T*pinv(M)*wt, np.identity(3)))
    a3 = np.hstack((mu, -1*np.identity(3), np.zeros((3,3))))
    A = np.vstack((a1,a2,a3))
    A = np.asarray(A)
    print(A.shape)
    print(pinv(A))

    b1 = wn.T*pinv(M)*h*0.03
    b2 = wn.T*pinv(M)*h*0.03
    b = np.vstack((b1,b2,np.zeros((3,1))))
    #  solve_lcp(A,b)


