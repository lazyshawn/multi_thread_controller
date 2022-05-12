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
}

/* **************** 析构函数 **************** */
Camera::~Camera(){
}

void Camera::active(std::vector<std::string> devSerials,
    std::vector<rs2::pipeline>& pipelines) {
  devNum = devSerials.size();
  intrPara = std::vector<rs2_intrinsics>(devNum);
  for (int i=0; i<devNum; ++i) {
    rs2::pipeline pipe;
    // 建立设备号的索引
    std::string serial = devSerials[i];
    devMap[serial] = i;
    // 初始化视频流
    rs2::config cfg;
    cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    // cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    rs2::pipeline_profile selection = pipe.start(cfg);
    // 读取内参
    auto stream =
        selection.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    intrPara[i] = stream.get_intrinsics();
    pipelines.emplace_back(pipe);
  }
  extrMat = std::vector<Eigen::Matrix<double,4,4>>(devNum);
  newFrames = std::vector<rs2::frame>(devNum);
}

void Camera::auto_active() {
  for (auto &&dev : ctx.query_devices()) {
    std::string devSerial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    serials.emplace_back(devSerial);
    std::cout << "Detect realsense device with num: " << devSerial
              << std::endl;
  }
}

int Camera::get_dev_index(std::string serial) {
  return devMap[serial];
}

/* **************** 所有相机统一读取数据 **************** */
void Camera::get_frameset(std::vector<rs2::pipeline>& pipelines) {
  // newFrames[0] = pipelines[0].wait_for_frames().get_color_frame();
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
  if(devIndex > devNum) return color;
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

/* **************** 获取像素点在相机坐标系下的 3D 坐标 **************** */
void Camera::pixel_to_point(float* point3d, float* pixel, float depth, int devIndex) {
  rs2_deproject_pixel_to_point(point3d, &intrPara[devIndex], pixel, depth);
}

/* **************** 检测 Marker 位姿 **************** */
// markerPose: Marker 在 TCP 坐标系下的位姿
// Ref: https://blog.csdn.net/qq_33446100/article/details/89115983
int Camera::detect_marker(cv::Mat frame, int id, float depth, int devIndex,
                          Eigen::Matrix<double, 4, 4> &markerPose) {
  Eigen::Matrix<double,4,4> obj2cam = Eigen::Matrix<double,4,4>::Zero();
  // 创建字典，这里注意使用Ptr<>，不然无法显示结果
  cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> marker;
  float origPixel[2], cornPixel[4][2], orig[3], corn[4][3], dx, dy, ds;

  // 检测该帧是否有标记
  // @param: 待检测的图像, 预先定义的字典对象, 检测出的图像的角点的列表,
  //         检测出的所有maker的ID列表
  cv::aruco::detectMarkers(frame, dictionary, marker, ids);

  if (ids.size() == 0) return 0;
  for (int i=0; i<ids.size(); ++i) {
    // 只处理检测到的第一个 Marker
    if (ids[i] == id) break;
    if (i == ids.size()-1) return 0;
  }

  // cv::Mat frame_show;
  // frame.copyTo(frame_show); //复制一份
  // // 如果有，则标记出来，放入另一个Mat
  // cv::aruco::drawDetectedMarkers(frame_show, marker, ids);
  // imshow("detected", frame_show);
  // 角点的空间坐标
  for (int i=0; i<3; ++i) {
    cornPixel[i][0] = marker[0][i].x;
    cornPixel[i][1] = marker[0][i].y;
    pixel_to_point(corn[i], cornPixel[i], depth, devIndex);
  }
  origPixel[0] = (marker[0][0].x + marker[0][2].x) / 2;
  origPixel[1] = (marker[0][0].y + marker[0][2].y) / 2;
  pixel_to_point(orig, origPixel, depth, devIndex);
  // 横坐标 x (P0 - P1)
  dx = corn[0][0] - corn[1][0];
  dy = corn[0][1] - corn[1][1];
  ds = sqrt(dx*dx + dy*dy);
  obj2cam(0,0) = dx/ds;
  obj2cam(1,0) = dy/ds;
  // 纵坐标 y (P1 - P2)
  dx = corn[1][0] - corn[2][0];
  dy = corn[1][1] - corn[2][1];
  ds = sqrt(dx*dx + dy*dy);
  obj2cam(0,1) = dx/ds;
  obj2cam(1,1) = dy/ds;
  obj2cam(0,3) = orig[0];
  obj2cam(1,3) = orig[1];
  obj2cam(2,3) = orig[2];
  obj2cam(2,2) = obj2cam(3,3) = 1;
  
  markerPose = extrMat[devIndex]*obj2cam;
  // std::cout << obj2cam << std::endl;
  return 1;
}
