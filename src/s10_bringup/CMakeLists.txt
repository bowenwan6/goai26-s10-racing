cmake_minimum_required(VERSION 3.16)
project(s10_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

# Keep one authoritative source copy while installing the profile beside the router config.
install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../models/deployed/gate16/front_tuck_command_profiles.json
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
