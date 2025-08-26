cmake_minimum_required(VERSION 3.5)
project(arm_controller)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

install(PROGRAMS
        src/arm_controller_node.py
        src/PCA9685.py
        DESTINATION lib/${PROJECT_NAME})

# Install launch files, if needed
if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
    install(DIRECTORY launch
            DESTINATION share/${PROJECT_NAME})
endif()

# Install config files, if needed
if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
    install(DIRECTORY config
            DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
