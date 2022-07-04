# joint_state_publisher_gui_cpp

## General info
This is **ROS2** joint_state_publisher_gui for CPP fans. It is built with Qt5.

## Setup
In your workspace (i.e. cd ~/dev_ws/src): 
```
git clone https://github.com/pietrzakmat/joint_state_publisher_gui_cpp
colcon build --packages-select joint_state_publisher_gui_cpp
```
##  Example usage: UR10 model from Universal Robots
Proceed in your workspace (i.e. cd ~/dev_ws/src): 
### Clone xacro description of UR robot from official repository:
 ```
 git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description
 ```
### Swap Original UR10 package launch script with modified one.
The swap of the launch scripts is necessary due to the fact that original one launches joint_state_publisher written in python wchich constantly publishes its state and effectively modifying the setting from "joint_state_publisher_gui_cpp". Therefore it needs to be supressed.
 ```
 mv ~/dev_ws/src/joint_state_publisher_gui_cpp/launch/view_ur_no_pub.launch.py ~/dev_ws/src/Universal_Robots_ROS2_Description/launch/view_ur_no_pub.launch.py 
 colcon build --packages-select ur_description
 ```

### Launch:
 ```
 . install/setup.bash && ros2 launch joint_state_publisher_gui_cpp ur10_and_pkg.launch.py 

 ```
