# beginner_tutorials
Introduction to ROS packages, catkin_workspace, nodes, Publisher and Subscriber, Service and Clients

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
For running the talker and subcriber node together, launch the ```demo.launch``` file in Terminal 1 -
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch beginner_tutorials demo.launch

```
or with optional arguments
```
roslaunch beginner_tutorials demo.launch publish_rate:=10 topic_name:=chatter

```
For calling the service run the following command in Terminal 2-
```
cd ~/catkin_ws
source devel/setup.bash
rosservice call /changeString "808XCourse"
```
For running rqt_console and rqt_logger, run the following commands in separate terminals
```
rosrun rqt_logger_level rqt_logger_level

```
```
rosrun rqt_console rqt_console

```

## Dependencies
1. ROS Kinetic. This can be downloaded by following the steps [here](http://wiki.ros.org/kinetic/Installation).
2. CMake. This can be downloaded from [here](https://cmake.org/download/)
