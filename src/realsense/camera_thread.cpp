#include "realsense/camera_thread.h"

ObjState objState;
Camera camera;
Recorder cameraRecord(7);

/*************************************************************************
* @func : camera_controller;
* @brief: Realsense 线程;
*************************************************************************/
void camera_controller() {
  /* 线程初始化 */
  struct timespec ts;
  long double time, startTime;
  cv::Mat handView, sideView;
  std::vector<std::string> devSerials;
  std::string cameraOnHand = "817612070676";
  devSerials.emplace_back(cameraOnHand);
  camera.active(devSerials);
  cv::VideoWriter handVideo = camera.create_recorder("./handVideo.avi");
  cv::VideoWriter sideVideo = camera.create_recorder("./sideVideo.avi");
  // 完成初始化
  time = get_current_time();
  pid_t cameraPid = get_tid();
  ROS_INFO("P[%d] T[%Lf] Realsense thread is Ready!", time, cameraPid);

  /* 等待线程同步 */
  ts = threadmanager.wait_for_syc();

  /* 开始伺服周期 */
  startTime = timespec2time(ts);
  while (miniROS::OK()) {
    /* Wait until next shot | 休眠到下个周期开始 */
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);

    /* 伺服线程主程序 */
    camera.get_frameset();
    handView = camera.get_frame(camera.get_dev_index(cameraOnHand));
    handVideo << handView;

    /* calculate next shot | 设置下一个线程恢复的时间 */
    ts.tv_nsec += CAMERA_PERIOD;
    // 时间进位
    time_wrap(ts);
    // imshow("Display color Image", handView);
    // cv::waitKey(10);
  } // while (1)
}

// Camera On Hand
// T_cam2elk << -0.9987748,  0.0360119, 0.0339432, 31,
//   -0.0359439, -0.9993594, 0.0026104, 82,
//   0.0340151,  0.0013871, 0.9994293, 26,
//   0, 0, 0, 1;

