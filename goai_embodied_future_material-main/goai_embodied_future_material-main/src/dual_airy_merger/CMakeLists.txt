cmake_minimum_required(VERSION 3.8)
project(dual_airy_merger)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(dual_airy_merger_node src/dual_airy_merger.cpp)
ament_target_dependencies(dual_airy_merger_node rclcpp sensor_msgs)
target_compile_features(dual_airy_merger_node PUBLIC cxx_std_17)
install(TARGETS dual_airy_merger_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config systemd DESTINATION share/${PROJECT_NAME})
ament_package()
