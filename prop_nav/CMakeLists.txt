cmake_minimum_required(VERSION 3.8)
project(prop_nav)

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
  tf2
  tf2_msgs
  tf2_ros
  tf2_geometry_msgs
  mm_interfaces
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(prop_nav
  src/prop_nav_node.cpp
  src/prop_nav_guidance.cpp
)

target_include_directories(prop_nav
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(prop_nav
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(TARGETS
  prop_nav
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
