cmake_minimum_required(VERSION 3.8)
project(simple_move)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(rcl_interfaces REQUIRED)

# C++ section
add_executable(simple_move_node 
    src/simple_move_node.cpp
)

ament_target_dependencies(simple_move_node
    rclcpp

    std_msgs
    sensor_msgs 
    geometry_msgs
    nav_msgs
    action_msgs
    actionlib_msgs

    tf2
    tf2_ros

    rcl_interfaces
)

# Python section
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
#  ${PROJECT_NAME}/my_python.py
  DESTINATION lib/${PROJECT_NAME}
)

# Ensure both are included
install(TARGETS
simple_move_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
