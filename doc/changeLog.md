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
### 220507
> commit [](
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

