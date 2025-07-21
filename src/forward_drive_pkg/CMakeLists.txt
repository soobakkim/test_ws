cmake_minimum_required(VERSION 3.5)
project(forward_drive_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp       REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(forward_drive_node src/forward_drive_node.cpp)
ament_target_dependencies(forward_drive_node
  rclcpp
  geometry_msgs
)

install(
  TARGETS forward_drive_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
