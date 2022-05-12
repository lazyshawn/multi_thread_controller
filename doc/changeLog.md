## ✨ ChangeLog
All notable changes to this project will be documented in this file (Since 2022.05.06).

Notation:
* `fix`: 修改 Bug。
* `doc`: 更新文档。
* `perf`: 功能完善。对项目或模块进行了性能优化。
* `chore`: 其他改动。修改注释或文件清理，不影响src和test代码文件的改动。
* `ci`: 持续集成。分支合并等。
* `revert`: 回滚了一些前面的代码。

<!-- The format is based on Keep a [Changelog](https://keepachangelog.com/en/1.0.0/). -->

## ✨ Logs
> commit [](
)
1. `perf`: 将 TCP 从 ELK 中心点转移到夹爪手指连线的中点上。
1. `perf`: 新增了 `timer_incre` 函数，线程计时器累加。
1. `fix`: 新增了逆解中角度是否为 `nan` 的判断。

### 220511
> commit [d3d3adc](
), commit [](
)
1. `fix`: 解决了相机线程中的`zsh: segmentation fault (core dumped)` 的问题。重写
   了相机管理类，`rs2::pipeline`不再是其成员变量。

### 220509
> commit [55573f0](
)
1. `fix`: 解决了 Realsense 程序中 `pipe.start()` 后 CPU 占用率高的问题。这是
   Realsense 库的安装问题，[Issues #2037](
   https://github.com/IntelRealSense/librealsense/issues/2037)。

### 220508
> commit [9cec07b](
)
1. `perf`: 优化了线程同步函数。普通线程也可以与伺服线程同步启动。

> commit [0710e1a](
)
1. `fix`: 修复了不连接相机时程序卡在预加载的问题。将相机初始化放入线程内。

### 220507
> commit [7b1fb68](
)
1. `fix`: 修改了目前程序中出现的`zsh: segmentation fault (core dumped)`问题。该
   问题大部分原因是由于数组越界导致的，本次是因为将`THETA`改为`std::vector`后在
   声明时未初始化数组大小。
1. `perf`: 将包的接口函数封装在对应的命名空间下。
1. `perf`: 将线程文件从主控包`master`放入对应的功能包。
1. `perf`: 根据包含关系重新整理头文件。

### 220506
> commit [e04fa3d](
)
1. `perf`: 保存数据时不保存空白数据。
1. `feat`: 新增 realsense 包。

> commit [1f2a7b9](
)
* 重新整理 [urController](https://github.com/lazyshawn/urController) 仓库。修改
  多线程代码结构，搭建 miniROS 框架。

