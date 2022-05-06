############################################################################
# ==>> Basic Setting
############################################################################
cmake_minimum_required(VERSION 3.5)
# Project name | ${PROJECT_NAME}
project(miniros)
set(CMAKE_CXX_STANDARD 17)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
# Link directories
link_directories(${CMAKE_SOURCE_DIR}/lib)

############################################################################
# ==>> Generate a library
############################################################################
# Output of the library
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)
# Generate the library from the library sources
# miniros_lib
add_library(miniros_lib SHARED
  src/miniros/miniros.cpp
  src/miniros/thread_manager.cpp
  src/miniros/time_manager.cpp
  src/miniros/user_interface.cpp
  src/miniros/data_export.cpp
)
# Specify the dependencies for a lib
target_link_libraries(miniros_lib -lpthread)
# Specify include directories for a lib
target_include_directories(miniros_lib PUBLIC
  ${PROJECT_SOURCE_DIR}/include/miniros
)
# ur_driver
add_library(ur_driver SHARED
  src/ur_driver/robot_state.cpp
  src/ur_driver/robot_state_RT.cpp
  src/ur_driver/ur_communication.cpp
  src/ur_driver/ur_communication_RT.cpp
  src/ur_driver/ur_driver.cpp
)
target_include_directories(ur_driver PUBLIC
  ${PROJECT_SOURCE_DIR}/include/ur_driver
)
# Ur5e
add_library(ur5e_driver SHARED
  src/ur5e_controller/ur5e_kinematics.cpp
  src/ur5e_controller/shared_variable.cpp
  src/ur5e_controller/ur5e_thread.cpp
  src/ur5e_controller/ur5e_interface.cpp
)
target_link_libraries(ur5e_driver -lur_driver)
target_include_directories(ur5e_driver PUBLIC
  ${PROJECT_SOURCE_DIR}/include/ur5e_controller
)
# wsg_controller_lib
add_library(wsg_driver SHARED
  src/wsg_controller/wsg_driver.cpp
  src/wsg_controller/shared_variable.cpp
)
target_include_directories(wsg_driver PUBLIC
  ${PROJECT_SOURCE_DIR}/include/wsg_controller
)
# Force Sensor
add_library(uskin_driver SHARED
  src/uskin/sensor_driver.cpp
  src/uskin/shared_variable.cpp
)
target_link_libraries(uskin_driver -lGinkgo_Driver)
target_include_directories(uskin_driver PUBLIC
  ${PROJECT_SOURCE_DIR}/include/uskin
)
# realsense
add_library(camera_driver SHARED
  src/realsense/camera_driver.cpp
  src/realsense/shared_variable.cpp
  src/realsense/camera_thread.cpp
)
find_package(OpenCV REQUIRED)
target_link_libraries(camera_driver ${OpenCV_LIBS} -lopencv_aruco -lrealsense2)
target_include_directories(camera_driver PUBLIC
  ${PROJECT_SOURCE_DIR}/include/realsense
)

############################################################################
# ==>> Create an executable
############################################################################
# Add an executable with the above sources
# miniros_bin
add_executable(miniros_bin
  src/miniros/main.cpp
)
target_link_libraries(miniros_bin miniros_lib)
# wsg_bin
add_executable(wsg_bin
  src/wsg_controller/wsg_main.cpp
)
target_link_libraries(wsg_bin miniros_lib wsg_driver)
# uskin_bin
add_executable(uskin_bin
  src/uskin/sensor_main.cpp
)
target_link_libraries(uskin_bin miniros_lib uskin_driver)
# ur5e_bin
add_executable(ur5e_bin
  src/ur5e_controller/ur5e_main.cpp
)
target_link_libraries(ur5e_bin miniros_lib ur5e_driver)
# camera_bin
add_executable(camera_bin
  src/realsense/camera_main.cpp
)
target_link_libraries(camera_bin miniros_lib camera_driver)

# master
add_executable(main
  src/master/main.cpp
  src/master/wsg_thread.cpp
  src/master/uskin_thread.cpp
)
target_link_libraries(main
  miniros_lib wsg_driver uskin_driver ur5e_driver camera_driver
)


