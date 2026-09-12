cmake_minimum_required(VERSION 3.8)
project(drdds)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ExecStatusValue.msg"
  "msg/Gait.msg"
  "msg/GaitValue.msg"
  "msg/InputStatusValue.msg"
  "msg/JointData.msg"
  "msg/JointDataCmd.msg"
  "msg/JointsData.msg"
  "msg/JointsDataCmd.msg"
  "msg/JointsDataCmdValue.msg"
  "msg/JointsDataValue.msg"
  "msg/LocationStatus.msg"
  "msg/LocationStatusValue.msg"
  "msg/LossStatusValue.msg"
  "msg/MetaType.msg"
  "msg/MotionInfo.msg"
  "msg/MotionInfoValue.msg"
  "msg/MotionStateValue.msg"
  "msg/Steer.msg"
  "msg/SteerValue.msg"
  DEPENDENCIES builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
