cmake_minimum_required(VERSION 2.8.3)
project(simulation_launch)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs tf)

## System dependencies are found with CMake's conventions
find_package (Eigen3 REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simulation_launch
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS Eigen3:Eigen
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
include_directories(${EIGEN3_INCLUDE_DIR})

###########
## Build ##
###########

add_executable(odom_filter src/odom_filter.cpp)
target_link_libraries(odom_filter ${catkin_LIBRARIES})

add_executable(slambox_odom_filter src/slambox_odom_filter.cpp)
target_link_libraries(slambox_odom_filter ${catkin_LIBRARIES})

add_executable(viz_path src/path.cpp)
target_link_libraries(viz_path ${catkin_LIBRARIES})

add_executable(slerp test/slerp.cpp)
target_link_libraries(slerp ${catkin_LIBRARIES})