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
## Running rostest/gtest 
Make sure you have followed the installing instructions first using as described in the last section
```
cd ~/catkin_ws/build
make run_tests
```
This will run a sample unit test called ```test_string_service.cpp``` in ```src/test/``` folder

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
For optionally recording the bag files with all the topics with given duration
```
roslaunch beginner_tutorials demo.launch rec_bag:=true bag_dur:=20
```
The bag file get saved in ```~/.ros``` folder 

For running rqt_console and rqt_logger, run the following commands in separate terminals
```
rosrun rqt_logger_level rqt_logger_level

```
```
rosrun rqt_console rqt_console

```
### Instructions for tf
For testing relation between frames
```
roslaunch beginner_tutorials demo.launch
rosrun tf tf_echo world talk
```
For printing the tree structure from the tf
```
rosrun tf view_frames
```
This will save a file called frames.pdf in the current directory.

### Instructions for using the bag file
For recording all the topics use the following command with optionally setting the duration of recording
```
rosbag record -a --duration=15
```
For Inspecting the containts use
```
rosbag info bagfile.bag
```
This will enlist all the topics recorded.

For testing the bag file has been recorded properly run the following commands in separate terminals- 
```
roscore
rosrun beginner_tutorials listener
rosbag play bagFileName.bag
```
This will cross-check whether the string is being published properly on the ```/chatter``` topic by the bag file.

## Dependencies
1. ROS Kinetic. This can be downloaded by following the steps [here](http://wiki.ros.org/kinetic/Installation).
2. CMake. This can be downloaded from [here](https://cmake.org/download/)
