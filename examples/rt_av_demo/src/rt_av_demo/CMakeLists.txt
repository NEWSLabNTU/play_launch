cmake_minimum_required(VERSION 3.8)
project(rt_av_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Stamped.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

# -O0 on purpose. burn_ms() must cost what it says it costs; at -O2 the
# optimiser reshapes the spin loop and the calibration drifts between
# machines, which would make the deadline numbers meaningless.
set(CMAKE_BUILD_TYPE Debug)

foreach(node lidar_driver obstacle_detector brake_controller load_node)
  add_executable(${node} src/${node}.cpp)
  target_include_directories(${node} PRIVATE src)
  ament_target_dependencies(${node} rclcpp std_msgs)
  rosidl_target_interfaces(${node} ${PROJECT_NAME} "rosidl_typesupport_cpp")
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
