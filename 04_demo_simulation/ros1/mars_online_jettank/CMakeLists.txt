cmake_minimum_required(VERSION 3.16)
project(mars_online_jettank)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  visualization_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_msgs std_msgs tf2 tf2_ros visualization_msgs
)

set(BUILD_TESTING OFF CACHE BOOL "Skip sibling module tests" FORCE)
set(MARS_BUILD_TESTS OFF CACHE BOOL "Skip graph-bundle tests" FORCE)
set(MARS_DEMO_BUILD_TESTS OFF CACHE BOOL "Skip Mode 1 unit tests" FORCE)

get_filename_component(MARS_PKG_DIR "${CMAKE_CURRENT_SOURCE_DIR}" REALPATH)
get_filename_component(MARS_DEMO_DIR "${MARS_PKG_DIR}/../.." ABSOLUTE)

add_subdirectory(
  ${MARS_DEMO_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/mars_demo)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(online_jettank_node
  src/laser_geometry.cpp
  src/online_jettank_node.cpp
)
target_compile_features(online_jettank_node PRIVATE cxx_std_17)
target_include_directories(online_jettank_node PRIVATE
  include
  ${MARS_DEMO_DIR}/include
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(online_jettank_node PRIVATE
  mars::demo_simulation
  ${catkin_LIBRARIES}
)

install(TARGETS online_jettank_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch worlds rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
