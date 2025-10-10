cmake_minimum_required(VERSION 3.8)
project(path_planner)

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

find_package(rcl_interfaces REQUIRED)

# C++ section
# Create library from PathPlanner.cpp
add_library(path_planner_lib
  src/PathPlanner.cpp
)

target_include_directories(path_planner_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(path_planner_lib
  rclcpp
  nav_msgs
  geometry_msgs
)

# Create executable
add_executable(path_planner_node 
    src/path_planner_node.cpp
)

target_link_libraries(path_planner_node
    path_planner_lib
)

ament_target_dependencies(path_planner_node
    rclcpp 

    std_msgs
    sensor_msgs 
    geometry_msgs
    nav_msgs
)


# Python section
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
#  ${PROJECT_NAME}/my_python.py
  DESTINATION lib/${PROJECT_NAME}
)

# Ensure both are included
install(TARGETS
  path_planner_lib
  path_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
