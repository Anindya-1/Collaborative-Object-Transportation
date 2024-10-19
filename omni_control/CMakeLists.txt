cmake_minimum_required(VERSION 3.8)
project(omni_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs  
  nav_msgs
  rclcpp
  tf2
  tf2_msgs
  tf2_ros
  tf2_geometry_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(go_to_goal
  src/go_to_goal_node.cpp
  src/go_to_goal.cpp
)

target_include_directories(go_to_goal
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(go_to_goal
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


install(TARGETS
  go_to_goal
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
