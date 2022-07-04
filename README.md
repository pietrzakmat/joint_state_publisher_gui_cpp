# joint_state_publisher_gui_cpp

## General info
* This is **ROS2** joint_state_publisher_gui for CPP fans. It is built with Qt5.

## Setup
* In your workspace (i.e. cd ~/dev_ws/src): 
```
git clone https://github.com/pietrzakmat/joint_state_publisher_gui_cpp
colcon build --packages-select joint_state_publisher_gui_cpp
```
##  Example usage: UR10 model from Universal Robots
* Build in your workspace (i.e. cd ~/dev_ws/src): 

 ```
 git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description

 mv ~/dev_ws/src/joint_state_publisher_gui_cpp/launch/view_ur_no_pub.launch.py ~/dev_ws/src/Universal_Robots_ROS2_Description/launch/view_ur_no_pub.launch.py 
 colcon build --packages-select ur_description
 ```
* Run:
 ```
 . install/setup.bash && ros2 launch joint_state_publisher_gui_cpp ur10_and_pkg.launch.py 

 ```
