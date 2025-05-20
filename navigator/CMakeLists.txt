cmake_minimum_required(VERSION 3.8)
project(navigator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs  
  nav_msgs
  rclcpp
  rclcpp_action
  tf2
  tf2_msgs
  tf2_ros
  tf2_geometry_msgs
  mm_interfaces
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(navigator
  src/navigator.cpp
)

target_include_directories(navigator
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(navigator
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(TARGETS
  navigator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
