cmake_minimum_required(VERSION 3.8)
project(robofest_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionMap.msg"
  "msg/MineDetection.msg"
  "msg/MineMap.msg"
  "msg/SwarmState.msg"
  "action/MissionCommand.action"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
