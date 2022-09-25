#include "dydw/dydw_driver.h"

DYDWDriver::DYDWDriver() {
}

DYDWDriver::~DYDWDriver() {
}

DYDWDriver::DYDWDriver(std::string tty, uint8_t ID) : ttyName(tty), slaveID(ID) {
  init(ttyName, slaveID);
}

int DYDWDriver::init(std::string tty, uint8_t ID) {
  ttyName = tty;
  rs485.connect(ttyName, 19200, 8, 0, 1);
  zero();
  return 1;
}

int DYDWDriver::zero() {
  rs485.writeMulReg(0x0320, {0x00, 0x00, 0x00, 0x0A});
  return 1;
}

std::vector<float> DYDWDriver::read() {
  std::vector<uint8_t> regVal;
  std::vector<float> val;
  regVal = rs485.readMulReg(0x0100, 8);
  // 将每一路的寄存器数据依次转化为 flaot 类型
  for (int i=0; i<regVal.size(); i=i+4) {
    // 读到的原始数据
    // for (int j=0; j<4; ++j) {
    //   printf("%02X ", regVal[i+j]);
    // }
    val.emplace_back(hex2float(&regVal[i]));
  }
  // 转换为浮点数后的数据
  // std::cout << std::endl;
  // for (int i=0; i<regVal.size()/4; ++i) {
  //   printf("%f ", val[i]);
  // }
  // std::cout << std::endl;
  return val;
}


