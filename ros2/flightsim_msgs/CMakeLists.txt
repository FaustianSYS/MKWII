cmake_minimum_required(VERSION 3.24)
project(flightsim_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlInputs.msg"
  "msg/AircraftState.msg"
  "msg/SimStatus.msg"
  "msg/TargetState.msg"
  "msg/MissileState.msg"
  "msg/EngagementStatus.msg"
  "msg/SceneEntity.msg"
  "msg/SceneState.msg"
  "msg/SeekerTrack.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
