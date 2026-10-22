import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_waterdroid_description = get_package_share_directory('waterdroid_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Path to URDF / Xacro file
    xacro_file = os.path.join(pkg_waterdroid_description, 'urdf', 'waterdroid.urdf.xacro')
    if not os.path.exists(xacro_file):
        xacro_file = os.path.join(pkg_waterdroid_description, 'urdf', 'waterdroid.urdf')

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]) if xacro_file.endswith('.xacro') else open(xacro_file).read(),
            'use_sim_time': True
        }]
    )

    # Launch Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Spawn Robot Node
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'waterdroid', '-z', '0.1'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot
    ])
