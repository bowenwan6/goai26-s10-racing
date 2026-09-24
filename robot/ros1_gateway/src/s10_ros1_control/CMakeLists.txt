cmake_minimum_required(VERSION 3.16)
project(s10_ros1_control CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(drdds REQUIRED)
find_package(yaml-cpp REQUIRED)
# Only for the find_ros1_package() pkg-config helper; nothing is linked from it.
find_package(ros1_bridge REQUIRED)
find_ros1_package(roscpp REQUIRED)
find_ros1_package(geometry_msgs REQUIRED)
find_ros1_package(std_msgs REQUIRED)

add_executable(s10_ros1_control src/control.cpp)
target_include_directories(s10_ros1_control SYSTEM PRIVATE
  ${ros1_roscpp_INCLUDE_DIRS} ${ros1_geometry_msgs_INCLUDE_DIRS} ${ros1_std_msgs_INCLUDE_DIRS})
target_link_libraries(s10_ros1_control ${ros1_roscpp_LIBRARIES} yaml-cpp)
ament_target_dependencies(s10_ros1_control rclcpp drdds)

install(TARGETS s10_ros1_control DESTINATION lib/${PROJECT_NAME})
ament_package()
