cmake_minimum_required(VERSION 2.8.3)
project(kinect_doa)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslaunch
)

roslaunch_add_file_check(launch)


###################################
## catkin specific configuration ##
###################################

catkin_package()


###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(minidsp
  src/minidsp.c
)

add_executable(kinect_doa_node src/kinect_doa_node.cpp src/KinectDOA.cpp src/ofxKinectExtras.cpp)
add_executable(servo_node src/servo_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(kinect_doa_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(servo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(kinect_doa_node
  ${catkin_LIBRARIES}
  minidsp
  freenect
  fftw3
)
target_link_libraries(servo_node
  ${catkin_LIBRARIES}
  wiringPi
)
