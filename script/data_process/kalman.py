import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

####################################################################
# 设置字体
####################################################################
# 正常显示中文标签
plt.rcParams['font.sans-serif'] = ['SimHei']
# 正常显示负号
plt.rcParams['axes.unicode_minus'] = False

class KalmanFilter:
    '''
    Kalman Filter:
    :iter: 迭代一次，返回当前最优估计值
    '''

    def __init__(self, dim, A, H, Q, R):
        '''
        :dim: 系统状态维度
        :A: 系统矩阵
        :H: 测量矩阵
        '''
        self.dim = dim  # 系统状态维度
        self.A = A      # 系统矩阵
        self.H = H      # 测量矩阵
        self.Q = Q      # 过程噪声方差
        self.R = R      # 观测噪声方差

        self.x_opt = np.zeros((dim,dim))       # 最优估计值 (初始状态值)
        self.p_opt = np.eye(dim)
        self.K = np.eye(dim)

    def iter(self, obsVal):
        '''
        利用上次保存的参数迭代计算并返回当前最优估计值
        '''
        # 1. 状态转移方程
        x_est = self.A * self.x_opt
        # 2. 观测方程
        zk = obsVal
        # 3. 先验估计 x_est 的协方差
        p_pre = self.A * self.p_opt * self.A.T + self.Q
        # 4. 更新卡尔曼增益
        self.K = p_pre*self.H.T * (self.H*p_pre*self.H.T + self.R)**-1
        # 5. 修正估计
        self.x_opt = x_est + self.K*(zk - self.H*x_est)
        # 6. 后验估计的协方差
        self.p_opt = (np.mat(np.identity(self.dim))-self.K*self.H)*p_pre
        return self.x_opt

if __name__ == "__main__":
    fig = plt.figure(figsize=(20/2.54, 20/2.54))
    ax1 = fig.add_subplot()

    time = np.linspace(0,100,101,endpoint=True)
    # 生成随机正态分布的观测噪声
    obs = np.random.normal(loc =0.0 , scale= 22.0,size = (101,1))
    # 实际观测值
    x_obs = np.array([obs[ii]+time[ii]*10 for ii in range(101)])
    # 绘制观测值散点图
    ax1.scatter(time, x_obs)

    A = np.matrix([[1, 1], [0, 1]])         # 系统矩阵
    H = np.matrix([[1.0, 0.0], [0.0, 1.0]]) # 测量矩阵
    Q = np.matrix([[0.5,0], [0,0.5]])       # 过程噪声方差
    R = np.matrix([[22,0], [0,22]])       # 观测噪声方差
    # 声明二维卡尔曼滤波器
    kf = KalmanFilter(2, A, H, Q, R)

    # 针对每次测量值进行滤波
    x_opt = np.array([kf.iter(np.matrix([x_obs[ii], [10]])) for ii in range(101)])
    x = [ii[0] for ii in x_opt]
    ax1.plot(time, x, 'r--')

    plt.show()

