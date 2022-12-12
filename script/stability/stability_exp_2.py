####################################################################
# 导入库
####################################################################
import matplotlib.pyplot as plt
import sympy as sym
from sympy import sin, cos, sqrt, pprint
import numpy as np
import matplotlib.gridspec as gridspec

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
d2r = sym.pi/180
r2d = 180/sym.pi

# ==========================================================
# @Brief : 检验支撑平面法向量乘积
# @Param : vG - sym.Matrix(3,n) - 支撑平面法向量与抓握矩阵的内积
# @Retrun: 0 - 不是支撑平面; 1 - 向量在支撑平面正面; -1 - 向量在支撑平面反面;
# ==========================================================
def verify_supP_norm(vG):
    vG = vG.evalf(3)
    findFirst = False
    for ii in vG:
        if (abs(ii) < 0.001):
            continue
        if (not findFirst):
            findFirst = True
            result = ii
        else:
            result *= ii
            if result < 0:
                return 0
            result = ii
    return result/abs(result)

# ==========================================================
# @Brief : 判断力旋是否在凸多棱锥内
# @Param : sym.Matrix(3,n) G - 凸多棱锥的棱边向量组成的矩阵;
#          sym.Matrix(3,1) Fe- 需要平衡的外力;
# @Return: True - 能平衡外力; False - 不能平衡外力;
# ==========================================================
def wrench_in_cone(G, Fe):
    # 合法性检测
    if not (G.shape[0] == 3):
        print("Shape of \'G\' must be: 3xn")
        return
    # 输入矩阵的宽度
    width = G.shape[1]
    Gi = [sym.Matrix(3,1,range(3)) for _ in range(width)]
    for ii in range(width):
        Gi[ii] = G[:,ii]
    # 遍历寻找支撑超平面
    for ii in range(width):
        for jj in range(ii+1, width):
            # vi,vj向量构成的超平面的法向量
            vij = Gi[ii].cross(Gi[jj])
            # 判断是否为支撑超平面
            vG = vij.T*G
            direc = verify_supP_norm(vG)
            if (direc != 0):
                # 判断外力与抓握力是否在支撑平面同侧
                vF = (vij.T*Fe).det().evalf(3)
                if ((vF/abs(vF)*direc) != 1): # 不在同侧,力旋不在锥内
                    return False
    return True

def dtdr(q):
    dt = 23.38*sin(q) - 22.74*cos(q) - 10*sin(q)
    dr = 13.38*cos(q) + 22.74*sin(q) - 50
    return dt, dr

# ==========================================================
# @Brief : 根据抓取点位置和物体倾斜角度生成抓握映射矩阵
# @Param : c_bc - BC 边上抓取点距离B点的距离
#          c_cd - CD 边上抓取点距离B点的距离
#          q    - 物体倾斜角度，逆时针为正
# @Retrun: G - 抓握映射矩阵; Fe - 重力的反作用力力旋;
# ==========================================================
def gen_grap_map(a1, a2, q_, mu1=0.3, mu2=0.3, mu3=0.2):
    # 模型的基本参数
    #  mu1, mu2, mu3, a, c1, c2, q = sym.symbols("mu1, mu2, mu3, a, c1, c2, theta")
    q = q_*d2r

    c1p = sym.Matrix([1,-mu1,a1-23.38+22.74*mu1])
    c1n = sym.Matrix([1,mu1,a1-23.38-22.74*mu1])
    c2p = sym.Matrix([-mu2,-1,23.38*mu2+a2-26.25])
    c2n = sym.Matrix([mu2,-1,-23.38*mu2+a2-26.25])
    qr = 1.039 - q
    dt, dr = dtdr(qr)
    c3p = sym.Matrix([mu3*cos(qr)-sin(qr), mu3*sin(qr)+cos(qr), dt-mu3*dr])
    c3n = sym.Matrix([-mu3*cos(qr)-sin(qr), -mu3*sin(qr)+cos(qr), dt+mu3*dr])
    G = sym.Matrix([[c1p,c1n,c2p,c2n,c3p,c3n]])
    qo = sym.pi - 1.039 + q
    Fe = sym.Matrix([-1*cos(qo),sin(qo),0])
    return G,Fe

def gen_grap_map_2(a1, a2, q_, mu1=0.3, mu2=0.3, mu3=0.2):
    # 模型的基本参数
    #  mu1, mu2, mu3, a, c1, c2, q = sym.symbols("mu1, mu2, mu3, a, c1, c2, theta")
    q = q_*d2r

    c1p = sym.Matrix([-mu1,-1,23.38*mu1+a1-26.25])
    c1n = sym.Matrix([mu1,-1,-23.38*mu1+a1-26.25])
    qr = 0.6435 + 0.02*a2
    dt, dr = dtdr(qr)
    c2p = sym.Matrix([mu2*cos(qr)-sin(qr), mu2*sin(qr)+cos(qr), dt-mu2*dr])
    c2n = sym.Matrix([-mu2*cos(qr)-sin(qr), -mu2*sin(qr)+cos(qr), dt+mu2*dr])
    c3p = sym.Matrix([mu3*cos(q)+sin(q), -mu3*sin(q)+cos(q), 26.62*(mu3*cos(q) +sin(q)) - 22.74*(-mu3*sin(q)+cos(q))])
    c3n = sym.Matrix([-mu3*cos(q)+sin(q), mu3*sin(q)+cos(q), 26.62*(-mu3*cos(q) +sin(q)) - 22.74*(mu3*sin(q)+cos(q))])
    G = sym.Matrix([[c1p,c1n,c2p,c2n,c3p,c3n]])

    Fe = sym.Matrix([sin(q),cos(q),0])
    return G,Fe
# ==========================================================
# @Brief : 绘制三维力旋量
# @Param : G - 3xn维力旋量组成的矩阵
# ==========================================================
def plot_wrench(G):
    ax = plt.figure().add_subplot(projection='3d')
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    ax.set_zlim(-1.5, 1.5)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.scatter(0,0,0, marker='o')
    ii = 0
    for vec in G:
        ii = ii + 1
        ax.quiver(0,0,0, vec[0], vec[1], vec[2])
        ax.text(vec[0], vec[1], vec[2], str(ii))


# ==========================================================
# @Brief : 均匀采样，获取系统的稳定状态空间
# @Param : dd - 手指在物体表面位置的采样间隔; dq - 物体倾斜角度的采样间隔
# ==========================================================
def sample_stablepoint(dd=5, dq=4):
    # 采样间隔
    stablePoints = []
    # 开始采样
    for qq in range(-42, 22, dq):
        for ii in range(0, 50, dd):
            for jj in range(0,50,dd):
                G, Fe = gen_grap_map(ii, jj, qq)
                flag = wrench_in_cone(G,Fe)
                if flag:
                    stablePoints.append([ii, jj, qq])
    # 保存数据
    save_list(stablePoints, "./data/points_exp2_1.txt")
    return stablePoints


def sample_stablepoint_2(dd=5, dq=4):
    # 采样间隔
    stablePoints = []
    # 开始采样
    for qq in range(0, 90, dq):
        for ii in range(0, 50, dd):
            for jj in range(0,55,dd):
                G, Fe = gen_grap_map(ii, jj, qq)
                flag = wrench_in_cone(G,Fe)
                if flag:
                    stablePoints.append([ii, jj, qq])
    # 保存数据
    save_list(stablePoints, "./data/points_exp2.txt")
    return stablePoints
# ==========================================================
# @Brief : 将稳定状态点绘制到三维图中
# @Param : stablePoints - 稳定的状态点
# ==========================================================
def plot_stablePoints(stablePoints):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    xx = [stablePoints[i][0] for i in range(len(stablePoints))]
    yy = [stablePoints[i][1] for i in range(len(stablePoints))]
    zz = [stablePoints[i][2] for i in range(len(stablePoints))]

    ax.scatter(xx, yy, zz, marker='o')

    ax.set_xlim(0, 168)
    ax.set_ylim(0, 168)
    ax.set_xticks([0, 56, 112, 168], labels=['A', 'B', 'C', 'D'])
    ax.set_yticks([0, 56, 112, 168], labels=['A', 'B', 'C', 'D'])
    ax.grid(True)
    ax.set_xlabel('Hand #2')
    ax.set_ylabel('Hand #1')
    ax.set_zlabel('Rotation angle')

    plt.show()


# ==========================================================
# @Brief : 将二维列表保存到指定文件
# @Param : data - 需要保存的 list 数据; fname - 保存文件路径
# ==========================================================
def save_list(data, fname):
    with open(fname, 'w') as f:
        for ii in range(len(data)):
            for jj in range(len(data[0])-1):
                f.write(str(data[ii][jj]) + ',')
            f.write(str(data[ii][-1]) + '\n')


# ==========================================================
# @Brief : 从指定文件读取二维列表
# @Param : data - 需要保存的 list 数据; fname - 保存文件路径
# ==========================================================
def load_from_file(fname):
    data = []
    with open(fname, 'r') as f:
        for line in f.readlines():
            content = line.rstrip().split(",")
            data.append([int(content[ii]) for ii in range(len(content))])
    return data


def stablePoints_plot_in3d():
    data = load_from_file("./data/points_exp2_1.txt")
    xx = np.array([data[i][0] for i in range(len(data))])
    yy = np.array([data[i][1] for i in range(len(data))])
    zz = np.array([data[i][2] for i in range(len(data))])

    #  fig = plt.figure(tight_layout=True)
    cm = 1/2.54
    #  fig = plt.figure(figsize=(17.6*cm, 10*cm), constrained_layout=True)
    fig = plt.figure(figsize=(15*cm, 8*cm), constrained_layout=True)
    ax = []

    # 3d-scatter-colorbar: https://stackoverflow.com/a/66026854
    ax.append(fig.add_subplot(projection='3d'))
    #  ax.append(fig.add_subplot(gs[0,1],projection='3d'))

    # https://stackoverflow.com/a/13784887
    ax[0].scatter(xx, yy, zz, c=zz, marker='o', s=50)
    ax[0].set_xlabel(r'手指#1位置 $c_1$(mm)')
    ax[0].set_ylabel(r'手指#2位置 $c_2$(mm)')
    ax[0].set_zlabel(r'物体倾斜角度 $\theta$($^o$)')
    plt.show()


if __name__ == "__main__":
    #  Gab1 = sym.Matrix([-mu3*cos(q)+sin(q), mu3*sin(q)+cos(q), (-mu3*cos(q)+sin(q)-mu3*sin(q)-cos(q))*a])
    #  G, F = gen_grap_map(35, 50, 30, mu1=0.3, mu2=0.3, mu3=0.2)
    #  print(wrench_in_cone(G, F))
    #  sample_stablepoint(dd=5, dq=4)
    stablePoints_plot_in3d()

