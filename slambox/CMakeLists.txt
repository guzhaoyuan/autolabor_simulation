cmake_minimum_required(VERSION 2.8.3)
project(slambox)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  tf
)

## System dependencies are found with CMake's conventions
find_package (Eigen3 REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES slambox
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(slambox_odom_filter src/slambox_odom_filter.cpp)
target_link_libraries(slambox_odom_filter ${catkin_LIBRARIES})

add_executable(IMU_filter src/IMU.cpp)
target_link_libraries(IMU_filter ${catkin_LIBRARIES})