cmake_minimum_required(VERSION 3.16)
project(ppowalk)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# getup exports OnnxPolicy / ObservationBuilder (getup_core) and its bundled
# ONNX Runtime.
find_package(getup REQUIRED)

# The bundled libonnxruntime.so lives in getup's lib directory.
set(CMAKE_INSTALL_RPATH "${getup_DIR}/../../../lib")

add_executable(ppowalk_policy_node src/ppowalk_policy_node.cpp)
target_include_directories(ppowalk_policy_node PRIVATE include)
ament_target_dependencies(ppowalk_policy_node
  rclcpp geometry_msgs sensor_msgs std_msgs std_srvs getup)

install(TARGETS ppowalk_policy_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch models DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_motion test/test_motion.cpp)
  target_include_directories(test_motion PRIVATE include)
  ament_target_dependencies(test_motion getup)
endif()

ament_package()
