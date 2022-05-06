<!-- TOC GFM -->

* [✨ miniROS](#-miniros)
* [✨ How to create miniROS packages](#-how-to-create-miniros-packages)
  - [Basic Idea](#basic-idea)
  - [Primary files](#primary-files)
  - [Devices](#devices)
* [✨ Todo](#-todo)

<!-- /TOC -->

## ✨ miniROS
The miniROS is a ROS-like multi-threads programming framework.

## ✨ How to create miniROS packages
### Basic Idea
miniROS 的基本思想是将不同线程封装为一个单独的包，在编译时将包编译为链接库，最后
在主线程中调用包的接口，从而实现线程管理。这样做的好处主要是能实现外层控制与不同
线程的分离，方便调试和修改。

### Primary files
一个包主要包含以下四个文件。
1. `driver`: 设备的驱动文件。最好写成一个类，用于在其伺服线程中控制设备。
1. `thread`: 包的伺服线程文件。
1. `shared_variable`: 包的接口文件。主要是用于线程交互的数据结构，通过调用该文件
   修改交互数据即能与伺服线程交互。封装数据结构时需要注意设置互斥量。
1. `interface`: 包与其他包之间的交互文件。
1. `main`: 包功能的调试文件。测试包的基本功能，不用于其他包。

### Devices
Yo can find all the manual of the devices used in these project at
[release page]().

## ✨ Todo
实验完成后需要整理的工作内容。
* [ ] 实验室 Git 文档;

