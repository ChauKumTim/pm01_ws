cmake_minimum_required(VERSION 3.8)
project(joint_state_mapping)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interface_protocol REQUIRED)

add_executable(joint_state_mapper
  src/joint_state_mapper.cpp
)

target_compile_features(joint_state_mapper PUBLIC cxx_std_17)

ament_target_dependencies(joint_state_mapper
  rclcpp
  sensor_msgs
  interface_protocol
)

install(
  TARGETS joint_state_mapper
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()