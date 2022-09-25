#ifndef DYDW_DRIVER_H
#define DYDW_DRIVER_H

#include "dydw/rs485_driver.h"

class DYDWDriver {
private:
  std::string ttyName;
  uint8_t slaveID = 0x01;
  RS485Device rs485;

public:
  DYDWDriver();
  DYDWDriver(std::string tty, uint8_t ID = 0x01);
  ~DYDWDriver();

  int init(std::string tty, uint8_t ID = 0x01);
  int zero();
  std::vector<float> read();
};

#endif
