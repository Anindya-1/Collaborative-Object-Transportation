cmake_minimum_required(VERSION 3.8)
project(inv_kin_solver)

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

include_directories(
  include
  /home/csl-a/alglib-cpp/src
)

#Add the ALGLIB source files
file(GLOB ALGLIB_SOURCES "/home/csl-a/alglib-cpp/src/*.cpp")

add_executable(inv_kin_node
  src/inv_kin_node.cpp
  src/optimizer_SQP.cpp
  ${ALGLIB_SOURCES}
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

target_include_directories(inv_kin_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/inv_kin_solver>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(inv_kin_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  inv_kin_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
