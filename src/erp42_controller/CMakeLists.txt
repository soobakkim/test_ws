cmake_minimum_required(VERSION 3.8)
project(erp42_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_STANDARD_REQUIRED)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(erp42_controller_node src/erp42_controller_node.cpp)

ament_target_dependencies(erp42_controller_node
  rclcpp
  geometry_msgs
)

install(
  TARGETS erp42_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.launch.py"
)

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
