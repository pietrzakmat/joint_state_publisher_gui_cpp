# joint_state_publisher_gui_cpp

## General info
This is **ROS2** joint_state_publisher_gui for CPP fans. It is built with Qt5.

## Setup
In your workspace (i.e. cd ~/dev_ws/src): 

#### Clone
```
git clone https://github.com/pietrzakmat/joint_state_publisher_gui_cpp
```

#### Build
```
colcon build --packages-select joint_state_publisher_gui_cpp
```
##  Example usage: UR10 model from Universal Robots
Proceed in your workspace (i.e. cd ~/dev_ws/src): 
### Clone xacro descriptions of UR robot from official repository
 ```
 git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description
 ```
### Add modified launch script to Universal Robot package
The modified launch script is necessary due to the fact that original one launches joint_state_publisher written in Python which constantly publishes its state and effectively modifies the setting from "joint_state_publisher_gui_cpp". Therefore it needs to be supressed.
 ```
 mv ~/dev_ws/src/joint_state_publisher_gui_cpp/launch/view_ur_no_pub.launch.py ~/dev_ws/src/Universal_Robots_ROS2_Description/launch/view_ur_no_pub.launch.py 
 ```
### Build ur_description package
 ```
 colcon build --packages-select ur_description
 ```

### Install & Launch
 ```
 . install/setup.bash && ros2 launch joint_state_publisher_gui_cpp ur10_and_pkg.launch.py 

 ```
