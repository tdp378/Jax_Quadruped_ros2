cmake_minimum_required(VERSION 3.8)
project(jax_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotionCommand.msg"
  "msg/GaitCommand.msg"
  "msg/Estop.msg"
  "msg/RobotState.msg"
  "srv/SetGait.srv"
  "action/Walk.action"
)

ament_package()
