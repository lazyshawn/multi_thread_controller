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

  char command;
  while (miniROS::OK()) {
    command = scanKeyboard();
    switch(command) {
      case 'r':
        dydw.read();
        break;
      case 'w':
        break;
      break;
      // 清零
      case 's':
        dydw.init("/dev/ttyUSB0");
        // sensor.connect("/dev/ttyUSB0", 19200, 8, 0, 1);
        break;
      case 27:
        std::cout << "hello" << std::endl;
        miniROS::shutdown();
        break;
      default:
        std::cout << "Unknow command" << std::endl;
        break;
    }

  }
  std::cout << threadPool.size() << std::endl;
  miniROS::joinall();

  return 0;
}

