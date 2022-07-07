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
  ObjState::Data objStateData;
  // 初始化相机
  std::vector<std::string> devSerials;
  std::string cameraOnHand = "817612070676";
  std::string cameraOnSide = "821312061951";
  // std::string cameraOnSide = "826212070613";
  devSerials.emplace_back(cameraOnHand);
  devSerials.emplace_back(cameraOnSide);
  std::vector<rs2::pipeline> pipelines;
  camera.active(devSerials, pipelines);
  cv::Mat handView, sideView;
  cv::VideoWriter handVideo = camera.create_recorder("./handVideo.avi");
  cv::VideoWriter sideVideo = camera.create_recorder("./sideVideo.avi");
  // 设定外参
  Mat4d cam2tcp, cam2base, obj2hand, obj2base;
  cam2tcp << -1, 0, 0, 28, 0, -1, 0, 80, 0, 0, 1, -148, 0, 0, 0, 1;
  camera.set_extrMat(0, cam2tcp);
  // 完成初始化
  ROS_INFO("P[%d] T[%Lf] Realsense thread is Ready!",
           get_tid(), get_current_time());

  /* 等待线程同步 */
  ts = threadmanager.wait_for_syc();

  /* 开始伺服周期 */
  startTime = timespec2time(ts);
  while (miniROS::OK()) {
    /* Wait until next shot | 休眠到下个周期开始 */
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);

    /* 伺服线程主程序 */
    time = get_current_time() - startTime;
    cameraRecord.push_item(std::vector<double>(1, time));
    camera.get_frameset(pipelines);
    handView = camera.get_frame(camera.get_dev_index(cameraOnHand));
    sideView = camera.get_frame(camera.get_dev_index(cameraOnSide));
    handVideo << handView;
    sideVideo << sideView;
    if(objState.check_marker()) {
      objStateData = objState.get_data();
      if(camera.detect_marker(handView, 14, 298, 0, objStateData.obj2elk)) {
        objState.update(&objStateData);
        objState.reset_check_marker();
      }
    }
    /* 记录数据 */
    cameraRecord.data_record();

    /* calculate next shot | 设置下一个线程恢复的时间 */
    timer_incre(ts, CAMERA_PERIOD);
    // imshow("Display color Image", handView); cv::waitKey(10);
  } // while (miniROS::OK())
  cameraRecord.data_export("objState.txt");
}

// Camera On Hand
// T_cam2elk << -0.9987748,  0.0360119, 0.0339432, 31,
//   -0.0359439, -0.9993594, 0.0026104, 82,
//   0.0340151,  0.0013871, 0.9994293, 26,
//   0, 0, 0, 1;

