cmake_minimum_required(VERSION 3.5)
project(unitree_ros2_example)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include include/common include/nlohmann)

find_package(ament_cmake REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(g1_loco_client_example
  src/g1/high_level/loco_client_example.cpp)
target_compile_features(g1_loco_client_example PRIVATE cxx_std_20)
ament_target_dependencies(g1_loco_client_example
  unitree_api
  rclcpp)

add_executable(g1_cmd_vel_bridge
  src/g1/high_level/cmd_vel_bridge.cpp)
target_compile_features(g1_cmd_vel_bridge PRIVATE cxx_std_20)
ament_target_dependencies(g1_cmd_vel_bridge
  unitree_api
  rclcpp
  geometry_msgs)

install(TARGETS
  g1_loco_client_example
  g1_cmd_vel_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
