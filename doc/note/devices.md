## ✨ Realsense

### Official Docs
1. API: https://intelrealsense.github.io/librealsense/doxygen/namespacers2.html

### Troubleshoot
* `wait_for_frames` and `poll_for_frames`

[Issues #2422](https://github.com/IntelRealSense/librealsense/issues/2422)

* High CPU usage with rs2::pipeline

[Issues #2037](https://github.com/IntelRealSense/librealsense/issues/2037)

Using pre-build packages [librealsense](
https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md).
Alternatively, build from source with CMake flag `BUILD_WITH_OPENMP=false`.

* segmentation fault (core dumped)

[Issues #6865](https://github.com/IntelRealSense/librealsense/issues/6865), 
[Issues #2107](https://github.com/isl-org/Open3D/issues/2107)

`pipeline`的声明和初始化需要在同一个作用域内(原因未知)。

