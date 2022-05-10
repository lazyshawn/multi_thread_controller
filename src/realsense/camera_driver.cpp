#include "realsense/camera_driver.h"

/*************************************************************************
 * @class: Camera
*************************************************************************/
/* **************** 构造函数 **************** */
Camera::Camera() {
  devNum = 0;
}

// 按序列号激活相机
Camera::Camera(std::string serialNum){
  // cfg.enable_device(serialNum);
  // cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
  //
  // // 启动设备的管道配置文件, 开始传送数据流
  // selection = pipe.start(cfg);
  //
  // // 读取内参
  // auto stream = selection.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
  // intr = stream.get_intrinsics(); // Calibration data
  //
  // // 初始化默认外参
  // extrMat << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  // set_extrMat(extrMat);
}

/* **************** 析构函数 **************** */
Camera::~Camera(){
}

void Camera::active(std::vector<std::string> devSerials) {
  intrPara = std::vector<rs2_intrinsics>(0);
  for (int i=0; i<devSerials.size(); ++i) {
    // 建立设备号的索引
    std::string serial = devSerials[i];
    devMap[serial] = i;
    // 初始化视频流
    rs2::config cfg;
    cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    rs2::pipeline_profile selection = pipe.start(cfg);
    pipelines.emplace_back(pipe);
    // 获取相机内参
    auto stream =
        selection.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    intrPara.emplace_back(stream.get_intrinsics());
  }
  newFrames = std::vector<rs2::frame>(pipelines.size());
}

void Camera::auto_active() {
  for (auto &&dev : ctx.query_devices()) {
    std::string devSerial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    serials.emplace_back(devSerial);
    std::cout << "Detect realsense device with num: " << devSerial
              << std::endl;
  }
  new (&pipe) rs2::pipeline(ctx);
  for (auto &&serial : serials) {
    rs2::config cfg;
    cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    pipe.start(cfg);
    pipelines.emplace_back(pipe);
  }
  newFrames = std::vector<rs2::frame>(pipelines.size());
}

int Camera::get_dev_index(std::string serial) {
  return devMap[serial];
}

/* **************** 所有相机统一读取数据 **************** */
void Camera::get_frameset() {
  std::vector<rs2::frame> new_frames;
  for (auto &&pipe : pipelines) {
    rs2::frameset fs;
    if (pipe.poll_for_frames(&fs)) {
      for (int i=0; i<fs.size(); ++i) {
        new_frames.emplace_back(fs[i]);
      }
    }
  }
  // Convert the newly-arrived frames to render-friendly format
  for (const auto &frame : new_frames) {
    // Get the serial number of the current frame's device
    auto serial =
        rs2::sensor_from_frame(frame)->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    newFrames[devMap[serial]] = frame;
  }
}

/* **************** 获取特定相机读取的视频帧 **************** */
cv::Mat Camera::get_frame(int devIndex) {
  cv::Mat color;
  if(devIndex > pipelines.size()) return color;
  auto frame = newFrames[devIndex];
  color = cv::Mat(cv::Size(640, 480), CV_8UC3, (void *)frame.get_data(),
                cv::Mat::AUTO_STEP);
  return color;
}

/* **************** 生成一个视频记录对象 **************** */
cv::VideoWriter Camera::create_recorder(std::string videoSavePath) {
  // cv::VideoWriter outputVideo("test.avi",
  //     cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, cv::Size(640, 480));
  cv::VideoWriter outputVideo;
  int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  double fps = 30;
  cv::Size size = cv::Size(640, 480);
  
  outputVideo.open(videoSavePath, fourcc, fps, size);
  if (!outputVideo.isOpened()) {
    std::cout << "fail to open!" << std::endl;
  }
  return outputVideo;
}

void Camera::set_extrMat(int devIndex, Eigen::Matrix<double, 4, 4> extr) {
  extrMat[devIndex] = extr;
}

