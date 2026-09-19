cmake_minimum_required(VERSION 3.16)
project(s10_him_contract)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
get_filename_component(ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(SDK "${ROOT}/upstream/goai_embodied_future_material/src/S10_sdk_deploy" CACHE PATH "Actual SDK source")
set(BUILD_PLATFORM "x86" CACHE STRING "x86 or arm")
set(ORT "${SDK}/third_party/onnxruntime/${BUILD_PLATFORM}")
add_executable(him_contract him_contract.cpp)
target_include_directories(him_contract PRIVATE "${ROOT}/integration" "${SDK}/run_policy"
  "${SDK}/include/types" "${SDK}/include/utils" "${SDK}/third_party/eigen" "${ORT}/include")
ament_target_dependencies(him_contract rclcpp std_msgs)
target_link_libraries(him_contract "${ORT}/lib/libonnxruntime.so")
target_compile_options(him_contract PRIVATE -UNDEBUG)

add_executable(him_handover him_handover.cpp "${SDK}/state_machine/parameters/s10_control_parameters.cpp")
target_include_directories(him_handover PRIVATE "${SDK}/state_machine/quadruped_wheel"
  "${SDK}/run_policy" "${SDK}/state_machine" "${SDK}/state_machine/parameters"
  "${SDK}/interface/robot" "${SDK}/interface/user_command" "${SDK}/include/types"
  "${SDK}/include/utils" "${SDK}/third_party/eigen" "${ORT}/include")
ament_target_dependencies(him_handover rclcpp std_msgs)
target_link_libraries(him_handover "${ORT}/lib/libonnxruntime.so")
target_compile_options(him_handover PRIVATE -UNDEBUG)
