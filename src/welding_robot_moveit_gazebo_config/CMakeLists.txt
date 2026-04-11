cmake_minimum_required(VERSION 3.10.2)
project(welding_robot_moveit_gazebo_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
