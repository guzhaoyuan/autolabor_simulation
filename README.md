# autolabor simulation

A simulation of differential drive vehicle.

## Structure

High level application folders:

- imr folder
- slambox folder
- local_planner folder

## Install

```shell script
sudo apt install ros-melodic-costmap-2d ros-melodic-random-numbers ros-melodic-nav-core ros-melodic-move-base ros-melodic-map-server ros-melodic-robot-localization ros-melodic-global-planner
cd $WORK_SPACE/src
git clone git@github.com:guzhaoyuan/autolabor_simulation.git
cd ../..
catkin_make
```

## Demos

1. IMR localization using UWB, camera and IMU.

```shell script
roslaunch imr perceptionbox_localization.launch
```

2. slam box localization using lidar and IMU.

```shell script
roslaunch slambox slambox_localization.launch
```

