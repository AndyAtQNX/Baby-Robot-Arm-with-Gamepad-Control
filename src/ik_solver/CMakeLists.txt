cmake_minimum_required(VERSION 3.5)
project(ik_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(EIGEN3_INCLUDE_DIR "$ENV{QNX_TARGET}/aarch64le/opt/ros/humble/include/eigen3")

# Mark Eigen3 as "found" so orocos_kdl doesn't search for it
set(EIGEN3_FOUND TRUE)
set(Eigen3_FOUND TRUE)

# Also set the CMake standard Eigen3 variables
set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

message(STATUS "Pre-configured Eigen3: ${EIGEN3_INCLUDE_DIR}")
# ===================================================================
# Find ROS packages
# ===================================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(std_msgs REQUIRED)
# Now find orocos_kdl (will use our pre-set EIGEN3_INCLUDE_DIR)
find_package(orocos_kdl REQUIRED)

# Find kdl_parser
find_package(kdl_parser REQUIRED)

# ===================================================================
# Build executable
# ===================================================================

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
