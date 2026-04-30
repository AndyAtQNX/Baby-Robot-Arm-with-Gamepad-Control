#
# Copyright (c) 2025, BlackBerry Limited. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
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
 