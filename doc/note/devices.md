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

可能是由于 `pipe` 开启的线程导致的。在子线程中申明 `pipe` 并通过参数的方式传递给
其他类或函数后不会出现这种错误。

