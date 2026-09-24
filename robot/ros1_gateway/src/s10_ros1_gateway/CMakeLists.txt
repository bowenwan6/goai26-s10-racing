cmake_minimum_required(VERSION 3.16)
project(s10_ros1_gateway CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# ros1_bridge exports find_ros1_package() (pkg-config lookup of ROS 1 packages).
find_package(ros1_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)

find_ros1_package(roscpp REQUIRED)
find_ros1_package(topic_tools REQUIRED)
find_ros1_package(std_msgs REQUIRED)

add_executable(s10_ros1_gateway src/gateway.cpp)
target_include_directories(s10_ros1_gateway SYSTEM PRIVATE
  ${ros1_roscpp_INCLUDE_DIRS} ${ros1_topic_tools_INCLUDE_DIRS} ${ros1_std_msgs_INCLUDE_DIRS})
target_link_libraries(s10_ros1_gateway
  ${ros1_roscpp_LIBRARIES} ${ros1_topic_tools_LIBRARIES} yaml-cpp)
ament_target_dependencies(s10_ros1_gateway rclcpp ros1_bridge)

install(TARGETS s10_ros1_gateway DESTINATION lib/${PROJECT_NAME})
ament_package()
