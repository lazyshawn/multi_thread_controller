#ifndef DYDW_DRIVER_H
#define DYDW_DRIVER_H

#include <string>
#include <iostream>
/* *** 串口通讯头文件 *** */
#include <termios.h>    // POSIX 终端控制
#include <fcntl.h>      // 文件控制
#include <stdio.h>      // 标准输入输出
#include <stdlib.h>     // 标准函数库
#include <sys/stat.h>   // 定义状态相关的数据类型
#include <sys/types.h>  // 定义数据类型
#include <unistd.h>     // Unix 标准函数
#include <stdint.h>

class SerialPort {
private:
  // 打开串口获得的文件描述符
  int nFd;
  // 串口名称: /dev/ttyUSB0
  char* device;
  // 波特率
  int baudRate;
  // 传输延时(us): 发送指令到读取串口返回值的间隔
  int udelay;
  // Linux串口配置的结构体
  struct termios serialSetting;
  struct termios settingBackup;
  // 传输数据
  uint8_t rxData[20], txData[20];

public:
  SerialPort(std::string device_);
  // 串口初始化
  int port_init();
  // 串口通讯
  int write_port(uint8_t* tx, int txLen);
  int read_port(uint8_t* rx, int rxLen);
};

class RS485Device {
private:
  // 从机地址
  int slaveID;
  // 传输数据
  uint8_t rxData[11], txData[17];

public:
  RS485Device();
  // CRC-16 校验
  uint16_t get_crc16(uint8_t *data, int len);
};

#endif
