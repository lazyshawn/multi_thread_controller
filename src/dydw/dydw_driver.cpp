#include "dydw/dydw_driver.h"

SerialPort::SerialPort(std::string device_) {
  port_init();
}

// 串口初始化
int SerialPort::port_init() {
  /* *** 打开串口 *** */
  // TODO: 遍历串口，直到打开
  // O_RDWR: 读写方式打开; O_NOCTTY: 不允许进程管理串口; O_NDELAY: 非阻塞
  nFd = open(device, O_RDWR|O_NOCTTY|O_NDELAY);
  if(-1 == nFd) {
    perror("Open Serial Port Error!\n");
    return -1;
  }
  // 恢复串口为阻塞状态
  if ((fcntl(nFd, F_SETFL, 0)) < 0) {
    perror("Fcntl F_SETFL Error!\n");
    return -1;
  }
  // 测试是否为终端设备
  if(isatty(STDIN_FILENO)==0) {
    printf("standard input is not a terminal device\n");
    return -1;
  }
  // 保存原先串口的配置
  if (tcgetattr(nFd, &settingBackup) != 0) {
    perror("tcgetattr error!\n");
    return -1;
  }

  /* *** 串口参数设置 *** */
  // 获取串口原来的参数设置
  tcgetattr(nFd, &serialSetting);
  // 设置终端为原始模式，该模式下全部的输入数据以字节为单位被处理
  cfmakeraw(&serialSetting);
  // 设置波特率，用户不能直接通过位掩码来操作
  cfsetispeed(&serialSetting, B115200);
  cfsetospeed(&serialSetting, B115200);

  // 本地连接 | 接收使能
  serialSetting.c_cflag |= (CLOCAL|CREAD);
  // 用数据位掩码清空数据位设置
  serialSetting.c_cflag &= ~CSIZE;
  // 数据位为8位
  serialSetting.c_cflag |= CS8;
  // 无校验位
  serialSetting.c_cflag &= ~PARENB;
  // 无奇偶校验位
  serialSetting.c_iflag &= ~INPCK; 
  // 清除CSTOPB，设置停止位为1 bits
  serialSetting.c_cflag &= ~CSTOPB;

  // 设置read()函数的调用方式
  // 指定读取每个字符之间的超时时间
  serialSetting.c_cc[VTIME]=2;
  // 指定所要读取字符的最小数量
  serialSetting.c_cc[VMIN]=1;

  // 清空终端未完毕的输入/输出请求及数据
  tcflush(nFd,TCIFLUSH);
  // 立即激活新配置
  if (tcsetattr(nFd, TCSANOW, &serialSetting) != 0) {
    perror("tcsetattr Error!\n");
    return -1;
  }
  return nFd;
}

// 串口通讯
int SerialPort::write_port(uint8_t* tx, int txLen) {
  /* *** 向串口发送 *** */
  int writeRet = write(nFd, tx, txLen);
  if (writeRet == txLen) {
    return writeRet;
  } else {
    tcflush(nFd, TCOFLUSH);
    return -1;
  }
}

int SerialPort::read_port(uint8_t* rx, int rxLen) {
  /* *** 从串口接收 *** */
  fd_set fs_read;
  // 超时时间10ms
  struct timeval timeout = {0, 10};
  
  // 将指定的文件描述符集清空，每次循环都要清空集合，否则不能检测描述符变化
  FD_ZERO(&fs_read);
  // 在文件描述符集合中增加新的文件描述符
  FD_SET(nFd,&fs_read);
  
  // 使用select实现串口的多路通信
  int retSel = select(nFd+1,&fs_read,NULL,NULL,&timeout);
  if(retSel < 0){
    // 错误
    printf("\033[1;31;40m Error: \033[0m Read port failed!\n");
    return -1;
  }else if(retSel == 0) {
    // 等待超时，没有可读写文件
    return 0;
  }else {
    // 返回读到的字节数
    int rRet = read(nFd,rxData,rxLen);    
    return rRet;
  }
}

// CRC-16 校验
// @param : *data-数据帧首地址(最高位); len-数据Bit位数;
// @return: uint16_t-两字节校验码(高位在左)。
uint16_t RS485Device::get_crc16(uint8_t *data, int len) {
  // 预置 CRC 寄存器
  uint16_t crc = 0xFFFF, lsb;

  for (int i = 0; i < len; ++i) {
    // 高8位与CRC寄存器异或
    crc ^= data[i];
    // 逐位移出并检测
    for (int j = 0; j < 8; ++j) {
      lsb = crc & 0x0001;
      crc = crc >> 1;
      // 最低位检测
      if (lsb != 0) crc ^= 0xA001;
    }
  }
  return crc;
}

