"""Waterdroid full simulation: Gazebo Harmonic + ROS 2 Jazzy.

    ros2 launch waterdroid_bringup simulation.launch.py
    ros2 launch waterdroid_bringup simulation.launch.py mission:=odom rviz:=true
    ros2 launch waterdroid_bringup simulation.launch.py mqtt:=true      # drive the Node-RED dashboard
    ros2 launch waterdroid_bringup simulation.launch.py world:=village.sdf mission:=none

Arguments
  world    world file inside waterdroid_gazebo/worlds  (default v2_village.sdf)
  mission  gt   = mission_navigator_gt (ground truth, full pump/dose/arm cycle)  [default]
           route = same fixed route by dead reckoning from /cmd_vel only, like the real rover
                   without GPS/compass; logs the drift against ground truth at every stop
           odom = mission_navigator (wheel odometry, waypoint pauses only)
           none = no autonomous mission (teleop / manual testing)
  mqtt     true = status_monitor mirrors telemetry onto the Node-RED MQTT topics
  rviz     true = open RViz
  gui      false = run Gazebo headless (server only, e.g. on a laptop without a GPU)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, EqualsSubstitution, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_desc = get_package_share_directory('waterdroid_description')
    pkg_gazebo = get_package_share_directory('waterdroid_gazebo')
    pkg_bringup = get_package_share_directory('waterdroid_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg_desc, 'urdf', 'waterdroid.urdf.xacro')
    params = os.path.join(pkg_bringup, 'config', 'params.yaml')
    rviz_cfg = os.path.join(pkg_desc, 'rviz', 'waterdroid.rviz')

    world = LaunchConfiguration('world')
    mqtt = LaunchConfiguration('mqtt')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    args = [
        DeclareLaunchArgument('world', default_value='v2_village.sdf'),
        DeclareLaunchArgument('mission', default_value='gt', choices=['gt', 'route', 'odom', 'none']),
        DeclareLaunchArgument('mqtt', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true', description='false = headless Gazebo server only'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r ',
                        PythonExpression(["'' if '", LaunchConfiguration('gui'), "' == 'true' else '-s --headless-rendering '"]),
                        PathJoinSubstitution([pkg_gazebo, 'worlds', world])],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'waterdroid',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'), '-z', '0.03'],
    )

    # [ = Gazebo -> ROS,  ] = ROS -> Gazebo
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/arm_servo/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        ],
    )

    water_processor = Node(package='waterdroid_control', executable='water_processor',
                           output='screen', parameters=[params])
    arm_controller = Node(package='waterdroid_control', executable='arm_controller',
                          output='screen', parameters=[params])
    status_monitor = Node(package='waterdroid_control', executable='status_monitor',
                          output='screen',
                          parameters=[params, {'mqtt_enabled': ParameterValue(mqtt, value_type=bool)}])

    mission_gt = Node(package='waterdroid_control', executable='mission_navigator_gt',
                      name='mission_navigator', output='screen', parameters=[params],
                      condition=IfCondition(EqualsSubstitution(LaunchConfiguration('mission'), 'gt')))
    # 'route' = same fixed route, but dead reckoning like the real rover (no GPS / compass)
    mission_route = Node(package='waterdroid_control', executable='mission_navigator_gt',
                         name='mission_navigator', output='screen',
                         parameters=[params, {'pose_source': 'dead_reckoning'}],
                         condition=IfCondition(EqualsSubstitution(LaunchConfiguration('mission'), 'route')))
    mission_odom = Node(package='waterdroid_control', executable='mission_navigator',
                        name='mission_navigator', output='screen',
                        parameters=[params, {'use_sim_time': True}],
                        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('mission'), 'odom')))

    rviz = Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg],
                parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription(args + [
        gazebo, robot_state_publisher, spawn_robot, bridge,
        water_processor, arm_controller, status_monitor,
        mission_gt, mission_route, mission_odom, rviz,
    ])
