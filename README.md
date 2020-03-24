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
cd $WORKSPACE/src
git clone git@github.com:guzhaoyuan/autolabor_simulation.git
cd ../..
catkin_make
```

## Demos

### 1.  IMR localization using UWB, camera and IMU.

```shell script
roslaunch imr perceptionbox_localization.launch
```

### 2. slam box localization using lidar and IMU.

```shell script
roslaunch slambox slambox_localization.launch
```

### 3. Simulation of differential drive vehicle, move to specified goal using navigation stack.

```shell script
roslaunch simulation_launch move_base_joy_sim.launch
```

I have implemented another local planner called cs_local_planner, in order to replace the dwa_local_planner in
 navigation stack. To run the same simulation with cs_local_planner. I assume the robot does not know the environment
  and require the laser scan sensor to update its environment and plan.

```shell script
roslaunch simulation_launch local_planner.launch
```
 
