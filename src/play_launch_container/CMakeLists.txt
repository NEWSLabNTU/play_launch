cmake_minimum_required(VERSION 3.14)
project(play_launch_container)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(composition_interfaces REQUIRED)
find_package(play_launch_msgs REQUIRED)
find_package(class_loader REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Shared library (both component manager classes)
add_library(observable_component_manager SHARED
  src/observable_component_manager.cpp
  src/clone_isolated_component_manager.cpp)
target_include_directories(observable_component_manager PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(observable_component_manager
  rclcpp rclcpp_components composition_interfaces play_launch_msgs)

# SlowLoader test component (shared library for composition)
add_library(slow_loader SHARED src/slow_loader.cpp)
ament_target_dependencies(slow_loader rclcpp rclcpp_components)
rclcpp_components_register_node(slow_loader
  PLUGIN "play_launch_container::SlowLoader"
  EXECUTABLE slow_loader_node)

# Container executable (single binary, configurable via CLI flags)
add_executable(component_container src/component_container.cpp)
target_link_libraries(component_container observable_component_manager)
ament_target_dependencies(component_container rclcpp)

# Standalone node binary (fork+exec'd by CloneIsolatedComponentManager)
add_executable(component_node src/component_node.cpp)
ament_target_dependencies(component_node
  rclcpp rclcpp_components class_loader ament_index_cpp)

install(TARGETS observable_component_manager slow_loader
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS component_container component_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_package()
