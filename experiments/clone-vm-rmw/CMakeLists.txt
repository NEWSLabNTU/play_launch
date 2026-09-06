cmake_minimum_required(VERSION 3.14)
project(clone_vm_rmw_probe)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(clone_vm_rmw_probe src/clone_vm_rmw_probe.cpp)
ament_target_dependencies(clone_vm_rmw_probe rclcpp std_msgs)
# _dl_allocate_tls / __ctype_init are resolved through dlsym at runtime.
target_link_libraries(clone_vm_rmw_probe ${CMAKE_DL_LIBS})
# -O0: the child bootstrap writes into struct pthread by discovered offset and
# must not be reordered around the clone boundary.
target_compile_options(clone_vm_rmw_probe PRIVATE -O0 -g)

install(TARGETS clone_vm_rmw_probe DESTINATION lib/${PROJECT_NAME})
ament_package()
