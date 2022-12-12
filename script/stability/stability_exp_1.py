####################################################################
# 导入库
####################################################################
import matplotlib.pyplot as plt
import sympy as sym
from sympy import sin, cos, sqrt

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

# ==========================================================
# @Brief : 根据抓取点位置和物体倾斜角度生成抓握映射矩阵
# @Param : c_bc - BC 边上抓取点距离B点的距离
#          c_cd - CD 边上抓取点距离B点的距离
#          q    - 物体倾斜角度，逆时针为正
# @Retrun: G - 抓握映射矩阵; Fe - 重力的反作用力力旋;
# ==========================================================
def gen_grap_map(c_bc, c_cd, q_, mu1=0.3, mu2=0.3, mu3=0.2):
    # 模型的基本参数
    #  mu1, mu2, mu3, a, c1, c2, q = sym.symbols("mu1, mu2, mu3, a, c1, c2, theta")
    mu1 = mu2  = 0.3
    mu3 = 0.2
    a = 28; cbc = c_bc; ccd = c_cd; q = q_*d2r

    Gab1 = sym.Matrix([-mu3*cos(q)+sin(q), mu3*sin(q)+cos(q), (-mu3*cos(q)+sin(q)-mu3*sin(q)-cos(q))*a])
    Gab2 = sym.Matrix([mu3*cos(q)+sin(q), -mu3*sin(q)+cos(q), (mu3*cos(q)+sin(q)+mu3*sin(q)-cos(q))*a])
    Gbc1 = sym.Matrix([-1, -mu2, (cbc-a)-mu2*a])
    Gbc2 = sym.Matrix([-1, mu2, (cbc-a)+mu2*a])
    Gcd1 = sym.Matrix([-mu1, -1, -(a-ccd)+mu1*a])
    Gcd2 = sym.Matrix([mu1, -1, -(a-ccd)-mu1*a])
    G = sym.Matrix([[Gab1, Gab2, Gbc1, Gbc2, Gcd1, Gcd2]])

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
    for qq in range(0, 90, dq):
        for ii in range(56, 112, dd):
            for jj in range(112,168,dd):
                G, Fe = gen_grap_map(ii-56, jj-112, qq)
                flag = wrench_in_cone(G,Fe)
                if flag:
                    stablePoints.append([ii, jj, qq])
    # 保存数据
    save_list(stablePoints, "../data/stablePoints.txt")
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


if __name__ == "__main__":
    # 色块宽度
    dd = 4
    width = height = 168

    # 绘制结果图
    fig, ax = plt.subplots(figsize=(8,8))

    # BC, CD 两边同时接触
    for ii in range(56, 112, dd):
        for jj in range(112,168,dd):
            G, Fe = gen_grap_map(ii-56, jj-112, 30)
            flag = wrench_in_cone(G,Fe)
            if flag:
                ax.broken_barh([(ii-dd/2,dd)],(jj-dd/2,dd), facecolors='#0000FF')

    # BC 边单独接触
    for ii in range(56, 112, dd):
        jj = 112
        G, Fe = gen_grap_map(ii-56, jj-112, 30)
        flag = wrench_in_cone(G,Fe)
        if flag:
            ax.broken_barh([(ii-dd/2,dd)],(jj-dd/2,dd), facecolors='#FF0000')

    # CD 边单独接触
    for jj in range(112,168,dd):
        ii = 56
        G, Fe = gen_grap_map(ii-56, jj-112, 30)
        flag = wrench_in_cone(G,Fe)
        if flag:
            ax.broken_barh([(ii-dd/2,dd)],(jj-dd/2,dd), facecolors='#FF0000')

    # 对角线
    for ii in range(0, 224, dd):
        ax.broken_barh([(ii-dd/2,dd)],(ii-dd/2,dd), facecolors='#000000')
    # 水平
    ax.broken_barh([(-dd/2,width)],(-dd/2,dd), facecolors='#000000')
    # 竖直
    ax.broken_barh([(-dd/2,dd)],(-dd/2,height), facecolors='#000000')

    ax.set_xlim(-dd*3/2, width +dd/2)
    ax.set_ylim(-dd*3/2, height+dd/2)
    ax.set_xlabel('Hand #2')
    ax.set_ylabel('Hand #1')
    ax.set_xticks([0,56,112,168], labels=['A', 'B', 'C', 'D'])
    ax.set_yticks([0,56,112,168], labels=['A', 'B', 'C', 'D'])
    ax.grid(True)

    plt.show()

