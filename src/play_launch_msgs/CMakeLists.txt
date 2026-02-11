cmake_minimum_required(VERSION 3.14)
project(play_launch_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ComponentEvent.msg"
  DEPENDENCIES builtin_interfaces
)

ament_package()
