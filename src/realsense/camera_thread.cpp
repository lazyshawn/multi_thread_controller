#include "realsense/camera_thread.h"

ObjState objState;
Camera camera;
Recorder cameraRecord(7);

/*************************************************************************
* @func : sensor_thread_function;
* @brief: 传感器线程;
*************************************************************************/
void camera_controller() {
  /* 线程初始化 */
  struct timespec ts;
  long double time, startTime;
  Eigen::Matrix<double, 4, 4> cam2elk;
  ObjState::Data objStateData;
  cam2elk << -1, 0, 0, 31, 0, -1, 0, 80, 0, 0, 1, 26, 0, 0, 0, 1;
  // T_cam2elk << -0.9987748,  0.0360119, 0.0339432, 31,
  //   -0.0359439, -0.9993594, 0.0026104, 82,
  //   0.0340151,  0.0013871, 0.9994293, 26,
  //   0, 0, 0, 1;
  std::string cameraOnHand = "817612070676";
  new (&camera) Camera(cameraOnHand.c_str());
  camera.set_extrMat(cam2elk);
  cv::VideoWriter outputVideo = camera.create_recorder();
  // 完成初始化
  time = get_current_time();
  pid_t cameraPid = get_tid();
  ROS_INFO("T[%Lf] P[%d] Realsense thread is Ready!", time, cameraPid);

  /* 等待线程同步 */
  ts = threadmanager.wait_for_syc();

  /* 开始伺服周期 */
  startTime = timespec2time(ts);
  while (miniROS::OK()) {
    /* Wait until next shot | 休眠到下个周期开始 */
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);

    /* 伺服线程主程序 */
    // 每个伺服周期开始的时间
    time = get_current_time() - startTime;
    cameraRecord.push_item(std::vector<double>(1, time));
    // 从读取相机图片
    cv::Mat color = camera.get_color_frame();
    outputVideo << color;
    // 检测 Marker 在 elk 下的位姿
    camera.detect_marker(color, 14, 120, objStateData.obj2elk);
    objState.update(&objStateData);
    /* 记录数据 */
    cameraRecord.data_record();
    // crmeraRecord.flag_reset();

    /* calculate next shot | 设置下一个线程恢复的时间 */
    ts.tv_nsec += CAMERA_PERIOD;
    // 时间进位
    time_wrap(ts);
    // imshow("Display color Image", color);
    // cv::waitKey(10);
  } // while (1)
  cameraRecord.data_export("objState.txt");
}

