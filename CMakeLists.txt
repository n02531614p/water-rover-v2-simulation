cmake_minimum_required(VERSION 3.8)
project(water_rover_v2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(ros_gz_bridge REQUIRED)

install(DIRECTORY urdf worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/v2_fsm.py DESTINATION lib/${PROJECT_NAME})

ament_package()
