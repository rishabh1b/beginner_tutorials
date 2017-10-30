# beginner_tutorials
Introduction to ROS packages, catkin_workspace, nodes, Publisher and Subscriber

## Standard install via command-line
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/rishabh1b/beginner_tutorials/
catkin_init_workspace
cd ~/catkin_ws
catkin_make
```
## Running Instructions
Terminal 1-
```
cd ~/catkin_ws
source devel/setup.bash
roscore

```
Terminal 2-
```
cd ~/catkin_ws
source devel/setup.bash
rosrun beginner_tutorials talker

```
Terminal 3-
```
cd ~/catkin_ws
source devel/setup.bash
rosrun beginner_tutorials listener

```

## Dependencies
1. ROS Kinetic. This can be downloaded by following the steps [here](http://wiki.ros.org/kinetic/Installation).
2. CMake. This can be downloaded from [here](https://cmake.org/download/)
