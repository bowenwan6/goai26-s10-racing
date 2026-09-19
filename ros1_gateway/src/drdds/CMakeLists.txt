cmake_minimum_required(VERSION 3.16)
project(drdds)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/MetaType.msg
  msg/GaitValue.msg
  msg/Gait.msg
  msg/NavCmdValue.msg
  msg/NavCmd.msg
  msg/MotionStateValue.msg
  msg/MotionState.msg
  msg/MotionInfoValue.msg
  msg/MotionInfo.msg
  DEPENDENCIES builtin_interfaces)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
