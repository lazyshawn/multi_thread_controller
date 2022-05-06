#include "miniros/miniros.h"
#include "uskin/sensor_driver.h"
#include "uskin/shared_variable.h"

Uskin forceSensor;
extern Force force;

/*************************************************************************
* @func : sensor_thread_function;
* @brief: 传感器线程;
*************************************************************************/
void sensor_thread_function(void) {
  Force::Data forceData;

  forceSensor.init_dev();

  while (miniROS::OK()) {
    forceSensor.read_force(forceData.forceMat, forceData.timestamp);
    force.update(&forceData);
  }
  std::cout << "Thread terminated: sensor_thread" << std::endl;
}

int main(int argc, char** argv) {
  Uskin forceSensorLocal;
  Force::Data forceData;

  if (!forceSensorLocal.init_dev()) {
    ROS_ERROR("Can't find the sensor: uskin");
    return 0;
  }

  while(true) {
    std::cout << forceData.forceMat[0][0][0] << "  " 
      << forceData.forceMat[0][0][1] << "  " 
      << forceData.forceMat[0][0][2] << std::endl;
  }

  return 0;
}

