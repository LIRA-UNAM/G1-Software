cmake_minimum_required(VERSION 3.5)
project(unitree_hg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message definitions copied verbatim from unitreerobotics/unitree_ros2
# (cyclonedds_ws/src/unitree/unitree_hg). Used by the G1/H1-2 low-level
# interface (rt/lowstate, rt/lowcmd).
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IMUState.msg"
  "msg/LowCmd.msg"
  "msg/LowState.msg"
  "msg/MotorCmd.msg"
  "msg/MotorState.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
