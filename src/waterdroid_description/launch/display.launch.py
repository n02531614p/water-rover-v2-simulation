"""Inspect the Waterdroid model in RViz (no Gazebo).

    ros2 launch waterdroid_description display.launch.py

Move the wheels and the delivery-arm servo with the joint_state_publisher_gui sliders.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('waterdroid_description')
    xacro_file = os.path.join(pkg, 'urdf', 'waterdroid.urdf.xacro')
    rviz_cfg = os.path.join(pkg, 'rviz', 'waterdroid.rviz')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Show joint_state_publisher_gui sliders'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg], output='screen'),
    ])
