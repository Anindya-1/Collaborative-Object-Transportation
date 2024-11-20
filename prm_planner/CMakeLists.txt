cmake_minimum_required(VERSION 3.8)
project(prm_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mm_interfaces REQUIRED)

add_executable(prm_planner 
  src/prm.cpp
)

target_include_directories(prm_planner
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(prm_planner
  rclcpp geometry_msgs visualization_msgs mm_interfaces
)

install(TARGETS
   prm_planner 
   DESTINATION lib/${PROJECT_NAME}
)


ament_package()
