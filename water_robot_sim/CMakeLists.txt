cmake_minimum_required(VERSION 3.8)
project(water_robot_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(ros_gz_bridge REQUIRED)

install(DIRECTORY urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/water_fsm.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
