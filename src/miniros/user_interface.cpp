#include "miniros/user_interface.h"

int scanKeyboard() {
  // 通过tcsetattr函数设置terminal的属性来控制需不需要回车来结束输入
  struct termios new_settings;
  struct termios stored_settings;
  // 备份 termios 设置
  tcgetattr(0, &stored_settings);
  new_settings = stored_settings;
  new_settings.c_lflag &= (~ICANON);
  new_settings.c_cc[VTIME] = 0;
  tcgetattr(0, &stored_settings);
  new_settings.c_cc[VMIN] = 1;
  tcsetattr(0, TCSANOW, &new_settings);
  // 监听键盘事件
  int input = getchar();
  // 还原设置
  tcsetattr(0, TCSANOW, &stored_settings);
  return input;
}

