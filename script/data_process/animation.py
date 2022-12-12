####################################################################
# 导入库
####################################################################
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import sympy as sym
from sympy import sin, cos, sqrt, pprint, atan2
import cv2
from calibration_sensor import read2Mat, writeMat
from dynamics import plane_kinematics, plane_inv_kinematics
from data_process.data_process import merge_sys_state
import pyrealsense2 as rs
from cv2 import aruco


####################################################################
# 设置字体
####################################################################
# 正常显示中文标签
plt.rcParams['font.sans-serif'] = ['SimHei']
# 正常显示负号
plt.rcParams['axes.unicode_minus'] = False


def draw_arraw(pic, pBeg, pEnd, length, alpha, color, thickness):
    '''
    绘制箭头
    :pic: Opencv 图片
    :pBeg, pEnd: 起点坐标、终点坐标
    :length: 箭头长度(>)
    :alpha: 箭头角度(度)
    :color: 颜色(r,g,b)
    :thickness: 线条宽度
    '''
    pi = 3.1415926
    angle = atan2(pBeg[1]-pEnd[1], pBeg[0]-pEnd[0])
    cv2.line(pic,(int(pBeg[0]),int(pBeg[1])),(int(pEnd[0]),int(pEnd[1])),color,thickness)
    # 箭头端点位置
    px = int(pEnd[0]+length*cos(angle+alpha*pi/180))
    py = int(pEnd[1]+length*sin(angle+alpha*pi/180))
    cv2.line(pic,(px,py),(int(pEnd[0]),int(pEnd[1])),color,thickness,2)
    px = int(pEnd[0]+length*cos(angle-alpha*pi/180))
    py = int(pEnd[1]+length*sin(angle-alpha*pi/180))
    cv2.line(pic,(px,py),(int(pEnd[0]),int(pEnd[1])),color,thickness,2)
    return pic


def calc_finger_on_pixel(state, dis=72):
    # 相机内参
    intr = rs.pyrealsense2.intrinsics()
    intr.height = 480; intr.width = 640; intr.fx = 621.368; intr.fy = 620.836
    intr.ppx = 308.964; intr.ppy = 234.005
    intr.model = rs.pyrealsense2.distortion.inverse_brown_conrady
    #  point = rs.rs2_project_point_to_pixel(intr, [x,y,z])
    x0 = 376.36 - state[2]*27    # x0 = 348
    y0 = 47.5; z0 = 485

    # 计算手指在机械臂坐标系中的位置
    lx = state[0]-dis/2*sin(state[2])
    lz = state[1]+dis/2*cos(state[2])
    rx = state[0]+dis/2*sin(state[2])
    rz = state[1]-dis/2*cos(state[2])

    lx = lx - x0
    ly = - lz - y0
    rx = rx - x0
    ry = - rz - y0

    plx,plz = rs.rs2_project_point_to_pixel(intr, [lx,ly,z0])
    prx,prz = rs.rs2_project_point_to_pixel(intr, [rx,ry,z0])

    return plx,plz,prx,prz


def plot_finger(pic, state, dis = 72):
    '''
    在图片中手指位置绘制实心圆
    :pic: cv2 图片
    :state: [x,z,q] TCP 状态
    :dis: 夹爪开合宽度
    '''
    plx,plz,prx,prz = calc_finger_on_pixel(state)

    # img, (x,y), radius, (b,g,r), thickness
    # thickness: 负数表示实心圆
    pic = cv2.circle(pic, (int(plx),int(plz)), 4, (0,255,0), -1)
    pic = cv2.circle(pic, (int(prx),int(prz)), 4, (0,255,0), -1)
    return pic


def plot_force_on_finger(pic, pFing, force, theta):
    '''
    :pic: Opencv 图片
    :pFing: 手指的像素位置
    :force: 传感器的力值
    '''
    length = 30
    pfing1 = np.array([int(pFing[0]), int(pFing[1])])
    pfing2 = np.array([int(pFing[2]), int(pFing[3])])
    # 计算力在世界坐标系下的表示
    fx1 = sin(theta)*force[0] - cos(theta)*force[1]
    fy1 = -cos(theta)*force[0] - sin(theta)*force[1]
    # 力在像素坐标系下的表示
    norm = sqrt(fx1*fx1+fy1*fy1)
    if norm > 15:
        pfx1 = -fx1/norm*length; pfy1 = -fy1/norm*length
        pf1 = np.array([pfx1,pfy1])
        pic = cv2.circle(pic, pfing1, 4, (0,255,0), -1)
        pic = draw_arraw(pic, pfing1, pfing1+pf1, 10, 30, (0,0,255), 2)
    # 手指2
    fx2 = sin(theta)*force[2] + cos(theta)*force[3]
    fy2 = -cos(theta)*force[2] + sin(theta)*force[3]
    norm = sqrt(fx2*fx2+fy2*fy2)
    if norm > 10:
        pfx2 = -fx2/norm*length
        pfy2 = fy2/norm*length
        pf2 = np.array([pfx2,pfy2])
        pic = cv2.circle(pic, pfing2, 4, (0,255,0), -1)
        pic = draw_arraw(pic, pfing2, pfing2+pf2, 10, 30, (0,0,255), 2)

    return pic


def get_camera_intrinsics():
    '''
    获取相机内参
    '''
    pipe = rs.pipeline()
    cfg = rs.config()
    #  config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    selec = pipe.start(cfg)
    dev = selec.get_device()
    name = dev.get_info(rs.camera_info.name)
    print(name)

    prof = selec.get_stream(rs.stream.color)
    intr = prof.as_video_stream_profile().get_intrinsics()
    print(intr)


def plot_obj_frame(pic):
    pi = 3.1415926
    dq = pi/180*36
    # 识别 Marker
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(pic, aruco_dict)
    # 如果没有识别到 Marker 则跳过
    if corners:
        #  frame_markers = aruco.drawDetectedMarkers(pic.copy(), corners, ids)
        # 从第一个点顺时针计数
        lh = corners[0][0][0]; rh = corners[0][0][1]
        rl = corners[0][0][2]; ll = corners[0][0][3]
        center = np.array([(lh[0]+rl[0])/2, (lh[1]+rl[1])/2])
        xdir = np.array([rh[0]-lh[0],rh[1]-lh[1]])
        # 绘制目标坐标系
        theta = atan2(-xdir[1],xdir[0])
        theta = (np.floor(theta/dq)+1)*dq
        xdir = np.array([cos(theta), -1*sin(theta)])
        ydir = np.array([cos(theta+3.1415926/2), -1*sin(theta+3.1415926/2)])
        pic = draw_arraw(pic, center, center+xdir*30, 10,30,(255,255,0),2)
        pic = draw_arraw(pic, center, center+ydir*30, 10,30,(255,255,0),2)
        # 绘制实际坐标系
        xdir = np.array([rh[0]-lh[0],rh[1]-lh[1]])
        ydir = np.array([lh[0]-ll[0],lh[1]-ll[1]])
        pic = draw_arraw(pic, center, center+xdir*1.2, 10,30,(0,255,0),2)
        pic = draw_arraw(pic, center, center+ydir*1.2, 10,30,(0,255,0),2)
    return pic


def main():

    # 加载系统状态信息
    mat = read2Mat('./rotate_and_grasp_cube_2/data/sysConfig.txt')
    # 加载视频流
    #  pic = cv2.imread('./rotate_and_grasp_cube_2/tmp/000000.jpg')
    #  size = pic.shape[:2]

    video = cv2.VideoWriter('./rotate_and_grasp_cube_2/output.avi', cv2.VideoWriter_fourcc('I', '4', '2', '0'), 24, (640, 480))
    for ii in range(950, 1800):
    #  for ii in range(100, 1800):
        pic = cv2.imread('./rotate_and_grasp_cube_2/tmp/' + str('%06d' % ii)+'.jpg')
        pic = plot_finger(pic, [mat[ii, 5], mat[ii, 6], mat[ii, 7]])

        pic = plot_obj_frame(pic)

        video.write(pic)
    video.release()
    cv2.destroyAllWindows()


def calc_finger_velocity(pFing, preFing):
    vx1 = pFing[0] - preFing[0]
    vy1 = pFing[1] - preFing[1]
    
    vx2 = pFing[2] - preFing[2]
    vy2 = pFing[3] - preFing[3]

    vel = np.array([vx1,vy1,vx2,vy2])

    return vel

def plot_force_and_vel_on_finger(pic,pFing,force,vel,theta):
    pfing1 = np.array([int(pFing[0]), int(pFing[1])])
    pfing2 = np.array([int(pFing[2]), int(pFing[3])])
    pic = plot_force_on_finger(pic,pFing,force,theta)

    norm = sqrt(vel[0]*vel[0] + vel[1]*vel[1])
    if norm > 0:
        vx1 = vel[0]/norm*30
        vy1 = vel[1]/norm*30
        pv1 = np.array([vx1,vy1])
        pic = draw_arraw(pic, pfing1, pfing1+pv1, 10, 30, (255,153,51), 2)
    norm = sqrt(vel[2]*vel[2] + vel[3]*vel[3])
    if norm > 0:
        vx2 = vel[2]/norm*30
        vy2 = vel[3]/norm*30
        pv2 = np.array([vx2,vy2])
        pic = draw_arraw(pic, pfing2, pfing2+pv2, 10, 30, (255,153,51), 2)
    return pic

if __name__ == "__main__":
    #  main()
    #  idx = 1000
    video = cv2.VideoWriter('./rotate_and_grasp_cube_2/output.avi', cv2.VideoWriter_fourcc('I', '4', '2', '0'), 24, (640, 480))
    for idx in range(0,100):
        pic = cv2.imread('./rotate_and_grasp_cube_2/tmp/' + str('%06d' %idx)+'.jpg')
        video.write(pic)
    for idx in range(100, 1800):
        mat = read2Mat('./rotate_and_grasp_cube_2/data/sysConfig.txt')
        pic = cv2.imread('./rotate_and_grasp_cube_2/tmp/' + str('%06d' %idx)+'.jpg')
        state = mat[idx,5:].A.ravel()
        force = mat[idx,1:5].A.ravel()
        theta = mat[idx,-1]
        # 计算手指位置
        pFing = calc_finger_on_pixel(state)
        # 计算速度
        preState = mat[idx-55,5:].A.ravel()
        preFing = calc_finger_on_pixel(preState)
        vel = calc_finger_velocity(pFing, preFing)
        # 绘制力和速度方向
        pic = plot_force_and_vel_on_finger(pic,pFing,force,vel,theta)
        # 绘制力方向
        #  pic = plot_force_on_finger(pic,pFing,force,theta)
        # 绘制 Marker
        pic = plot_obj_frame(pic)
        video.write(pic)
    for idx in range(1800, 2264):
        pic = cv2.imread('./rotate_and_grasp_cube_2/tmp/' + str('%06d' %idx)+'.jpg')
        video.write(pic)
    video.release()

        #  cv2.imshow("sdf", pic)
        #  if cv2.waitKey(0) == 27:
        #      cv2.destroyAllWindows()


