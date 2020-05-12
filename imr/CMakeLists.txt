cmake_minimum_required(VERSION 2.8.3)
project(imr)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  tf
  tf_conversions
  apriltag_ros
  rviz_visual_tools
  message_filters
)
find_package (Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imr
#  CATKIN_DEPENDS nav_msgs roscpp rospy tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(odom_filter src/odom_filter.cpp)
target_link_libraries(odom_filter ${catkin_LIBRARIES})

add_executable(viz_path src/path.cpp)
target_link_libraries(viz_path ${catkin_LIBRARIES})