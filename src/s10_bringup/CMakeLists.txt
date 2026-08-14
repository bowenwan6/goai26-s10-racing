cmake_minimum_required(VERSION 3.16)
project(s10_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
