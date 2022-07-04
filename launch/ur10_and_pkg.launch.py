import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction


def generate_launch_description():

    joint_state_publisher_gui_cpp_node = Node(
        package="joint_state_publisher_gui_cpp",
        executable="joint_state_publisher_gui_cpp",
    )

    ur10_robot = IncludeLaunchDescription(
        #PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ur_description'), 'launch'),'/view_ur.launch.py']),
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ur_description'), 'launch'),'/view_ur_no_pub.launch.py']),
        launch_arguments={'ur_type': 'ur10'}.items(),
      )

    return LaunchDescription([
        joint_state_publisher_gui_cpp_node,
        ur10_robot
    ])
