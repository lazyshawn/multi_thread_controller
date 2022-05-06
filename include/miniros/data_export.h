#pragma once

/* 文件读写 */
#include <fstream>
#include <iostream>
#include <iomanip>
/* 检测并创建文件夹 */
// Ref: https://blog.csdn.net/mao_hui_fei/article/details/122670188
#include <unistd.h>
#include <sys/stat.h>

#include <vector>

#define MAX_DATA_LENTH 10000

class Recorder{
private:
  std::vector<std::vector<double>> dataMat;
  int itemIndex, colIndex, cols, maxLen;
  bool flag;

public:
  std::vector<double> itemVec;

  Recorder(int cols_, int maxLen_=MAX_DATA_LENTH);
  void flag_set();
  void flag_reset();
  void data_record(std::vector<double> data);
  void data_record();
  void data_export(std::string fname);
  void push_item(std::vector<double> item);
};

