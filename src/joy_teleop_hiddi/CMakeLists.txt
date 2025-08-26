cmake_minimum_required(VERSION 3.5)
project(joy_teleop_hiddi)
 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
 
include_directories(include)
 
add_executable(joy_teleop_node
  src/joy_teleop_node.cpp
  src/parser.c
)
 
# Link against the QNX HIDDI library
target_link_libraries(joy_teleop_node
  hiddi
  spdlog
)
 
ament_target_dependencies(joy_teleop_node
  "rclcpp"
  "sensor_msgs"
)
 
install(TARGETS
  joy_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)
 
ament_package()
 