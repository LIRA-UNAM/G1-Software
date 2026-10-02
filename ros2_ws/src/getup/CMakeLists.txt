cmake_minimum_required(VERSION 3.16)
project(getup)

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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(unitree_hg REQUIRED)

include(cmake/FetchOnnxRuntime.cmake)

# Executables live in lib/getup, the bundled onnxruntime in lib.
set(CMAKE_INSTALL_RPATH "$ORIGIN/..;$ORIGIN")

add_library(getup_core
  src/onnx_policy.cpp
  src/observation.cpp
  src/motor_crc.cpp
)
target_include_directories(getup_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(getup_core PUBLIC onnxruntime::onnxruntime)
ament_target_dependencies(getup_core PUBLIC unitree_hg)

add_executable(getup_policy_node src/getup_policy_node.cpp)
target_link_libraries(getup_policy_node getup_core)
ament_target_dependencies(getup_policy_node rclcpp sensor_msgs std_msgs std_srvs)

add_executable(g1_lowlevel_bridge src/g1_lowlevel_bridge.cpp)
target_link_libraries(g1_lowlevel_bridge getup_core)
ament_target_dependencies(g1_lowlevel_bridge rclcpp sensor_msgs std_msgs std_srvs)

install(TARGETS getup_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(TARGETS getup_policy_node g1_lowlevel_bridge
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch models
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_observation test/test_observation.cpp)
  target_link_libraries(test_observation getup_core)
  ament_add_gtest(test_motor_crc test/test_motor_crc.cpp)
  target_link_libraries(test_motor_crc getup_core)
  ament_add_gtest(test_onnx_policy test/test_onnx_policy.cpp)
  target_link_libraries(test_onnx_policy getup_core)
  target_compile_definitions(test_onnx_policy PRIVATE
    GETUP_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
endif()

ament_package()
