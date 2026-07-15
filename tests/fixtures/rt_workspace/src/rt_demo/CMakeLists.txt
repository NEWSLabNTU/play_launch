cmake_minimum_required(VERSION 3.8)
project(rt_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

# sensor_node - standalone 100 Hz publisher on points_raw
add_executable(sensor_node src/sensor_node.cpp)
ament_target_dependencies(sensor_node rclcpp std_msgs)

# control_node - standalone: sub points_raw -> pub cmd (the FIFO-20 node)
add_executable(control_node src/control_node.cpp)
ament_target_dependencies(control_node rclcpp std_msgs)

# filter_component - rclcpp_components composable: sub points_raw -> pub points_filtered
add_library(filter_component SHARED src/filter_component.cpp)
ament_target_dependencies(filter_component rclcpp rclcpp_components std_msgs)
rclcpp_components_register_node(filter_component
  PLUGIN "rt_demo::FilterComponent"
  EXECUTABLE filter_component_node
)

install(TARGETS filter_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  sensor_node
  control_node
  DESTINATION lib/${PROJECT_NAME}
)

# The launch/ + contract sidecar live at the fixture root (a sibling of
# src/), not inside this package's own source tree, so both the runnable
# example and the integration fixture share one copy.
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/../../launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
