cmake_minimum_required(VERSION 3.8)
project(atlas_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/RobotState.msg"
  "msg/BalanceStatus.msg"
  "msg/GaitCommand.msg"
  "msg/PerceptionResult.msg"
  "msg/TaskGoal.msg"
  "msg/SupervisorHeartbeat.msg"
  "msg/BrainQuery.msg"
  # Services
  "srv/SetGait.srv"
  "srv/PlanTask.srv"
  "srv/QueryBrain.srv"
  "srv/SystemDiagnostic.srv"
  # Actions
  "action/ExecuteTask.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_package()
