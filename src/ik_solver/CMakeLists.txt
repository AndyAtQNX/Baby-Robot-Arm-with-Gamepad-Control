cmake_minimum_required(VERSION 3.5)
project(ik_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(std_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(ik_solver_node
  src/ik_solver_node.cpp
)

target_include_directories(ik_solver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(ik_solver_node
  spdlog
)

ament_target_dependencies(ik_solver_node
  "rclcpp"
  "sensor_msgs"
  "kdl_parser"
  "std_msgs"
  "urdf"
  "orocos_kdl"
)

# ===================================================================
# Install
# ===================================================================

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS ik_solver_node DESTINATION lib/${PROJECT_NAME})

ament_package()
