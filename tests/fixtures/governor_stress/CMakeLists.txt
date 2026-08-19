cmake_minimum_required(VERSION 3.8)
project(governor_stress)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(mem_hog src/mem_hog.cpp)
ament_target_dependencies(mem_hog rclcpp)

add_executable(cpu_burn src/cpu_burn.cpp)
ament_target_dependencies(cpu_burn rclcpp)

install(TARGETS mem_hog cpu_burn DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
