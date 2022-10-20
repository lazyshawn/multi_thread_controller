#include "dydw/dydw_thread.h"

void writePort(SerialPort serial) {
  // 读六路数据--float型
  uint8_t txData[] = {0x01, 0x03, 0x01, 0x00, 0x00, 0x0C, 0x44, 0x33};
  // threadmanager.wait_for_syc();
  while(miniROS::OK()) {
    serial.write_port(txData, sizeof(txData));
    sleep(1);
  }
  std::cout << "write out" << std::endl;
}

void readPort(SerialPort serial) {
  uint8_t rxData[29];
  threadmanager.wait_for_syc();
  while(miniROS::OK()) {
    int retLen = serial.read_port(rxData, sizeof(rxData));
    if (retLen > 0) {
      std::cout << "retLen = " << retLen << std::endl;
      for (int i=0; i<retLen; ++i) {
        printf("%02X  ", rxData[i]);
      }
      std::cout << std::endl;
    }
    usleep(1000);
  }
  std::cout << "read out" << std::endl;
}

int main(int argc, char** argv) {
  RS485Device sensor;
  DYDWDriver dydw;
  std::vector<float> data;
  dydw.init("/dev/ttyUSB0");

  char command;
  while (miniROS::OK()) {
    data = dydw.read(4);
    for (int i=0; i<4; ++i) {
      std::cout << data[i] << " ";
    }
    std::cout << std::endl;
  }
  std::cout << threadPool.size() << std::endl;
  miniROS::joinall();

  return 0;
}

