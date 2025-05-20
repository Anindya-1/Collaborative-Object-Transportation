cmake_minimum_required(VERSION 3.8)
project(inv_kin_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find core dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mm_interfaces REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs  
  nav_msgs
  rclcpp
  tf2
  tf2_msgs
  tf2_ros
  tf2_geometry_msgs
  mm_interfaces
  trajectory_msgs
)

include_directories(
  include
  /home/csl-a/alglib-cpp/src
)

# Add the ALGLIB source files
file(GLOB ALGLIB_SOURCES "/home/csl-a/alglib-cpp/src/*.cpp")

# Executable 1
add_executable(inv_kin_node
  src/inv_kin_node.cpp
  src/optimizer_SQP.cpp
  ${ALGLIB_SOURCES}
)

target_include_directories(inv_kin_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/inv_kin_solver>
         $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(inv_kin_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Executable 2
add_executable(standalone_inv_kin_node
  src/inv_kin_ros_standalone.cpp
  src/optimizer_SQP.cpp  
  ${ALGLIB_SOURCES}
)

target_include_directories(standalone_inv_kin_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/inv_kin_solver>
         $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(standalone_inv_kin_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Executable 3
add_executable(test_inv_kin
  src/test_inv_kin.cpp
  src/optimizer_SQP.cpp
  ${ALGLIB_SOURCES}
)

target_include_directories(test_inv_kin
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/inv_kin_solver>
         $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(test_inv_kin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install executables
install(TARGETS
  inv_kin_node
  standalone_inv_kin_node
  test_inv_kin
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

