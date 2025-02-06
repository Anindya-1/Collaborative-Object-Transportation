cmake_minimum_required(VERSION 3.8)
project(dijkstra)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mm_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(dijkstra_node
  src/dijkstra_node.cpp
)

target_include_directories(dijkstra_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dijkstra_node
  rclcpp geometry_msgs mm_interfaces visualization_msgs
)

install(TARGETS
  dijkstra_node 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
