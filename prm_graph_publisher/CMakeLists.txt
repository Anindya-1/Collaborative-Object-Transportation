cmake_minimum_required(VERSION 3.8)
project(prm_graph_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(mm_interfaces REQUIRED)

add_executable(prm_graph_publisher
  src/prm_graph_publisher.cpp
)

target_include_directories(prm_graph_publisher
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(prm_graph_publisher
  rclcpp geometry_msgs visualization_msgs example_interfaces mm_interfaces
)

install(TARGETS
   prm_graph_publisher
   DESTINATION lib/${PROJECT_NAME}
)


ament_package()
