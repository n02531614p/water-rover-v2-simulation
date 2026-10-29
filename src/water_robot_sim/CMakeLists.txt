cmake_minimum_required(VERSION 3.8)
project(water_robot_sim)

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

ament_package()
