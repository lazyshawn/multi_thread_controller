#include "miniros/data_export.h"

Recorder::Recorder(int cols_, int maxLen_) : cols(cols_), maxLen(maxLen_){
  dataMat = std::vector(maxLen, std::vector<double>(cols, 0));
  itemVec = std::vector<double>(cols, 0);
  itemIndex = colIndex = 0;
  flag = true;
}

/* 数据记录的标志位 */
void Recorder::flag_set() {
  flag = true;
}
void Recorder::flag_reset() {
  flag = false;
}

void Recorder::data_record(std::vector<double> data) {
  if(!flag) {
    colIndex = 0;
    return;
  }
  if (itemIndex > maxLen) {
    dataMat.emplace_back(data);
  } else {
    dataMat[itemIndex] = data;
  }
  itemIndex++;
  colIndex = 0;
  return;
}

void Recorder::data_record() {
  data_record(itemVec);
}

void Recorder::data_export(std::string fname) {
  std::ofstream file;
  std::string folder = "./data", path = folder + "/" + fname;
  int len = 16;
  // 检测并创建文件夹
  if (access(folder.c_str(), 0) != F_OK) {
    printf("File don't exist, create new folder: %s\n", folder.c_str());
    mkdir(folder.c_str(), S_IRWXU);
  };

  file.open(path);
  if(!file.is_open()){printf("Open file failed. -- %s\n", fname.c_str());}
  // 写入数据
  for (int i=0; i<dataMat.size(); ++i) {
    file << std::left;
    for (int j=0; j<dataMat[0].size(); ++j) {
      file << std::setw(len) << dataMat[i][j];
    }
    file << std::endl;
  }

  file.close();
  printf("Data saved: %s.\n", fname.c_str());
}

void Recorder::push_item(std::vector<double> item) {
  copy(item.begin(), item.end(), itemVec.begin()+colIndex);
  colIndex += item.size();
}

