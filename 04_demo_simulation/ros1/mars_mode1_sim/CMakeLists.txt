cmake_minimum_required(VERSION 3.16)
project(mars_mode1_sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  visualization_msgs)

catkin_package()

set(BUILD_TESTING OFF CACHE BOOL "Skip sibling module tests" FORCE)
set(MARS_BUILD_TESTS OFF CACHE BOOL "Skip graph-bundle tests" FORCE)
set(MARS_DEMO_BUILD_TESTS OFF CACHE BOOL "Skip Mode 1 unit tests" FORCE)

# Resolve the catkin symlink so ../.. is 04_demo_simulation, not ~/mars_ws.
get_filename_component(MARS_MODE1_PKG_DIR "${CMAKE_CURRENT_SOURCE_DIR}" REALPATH)
get_filename_component(MARS_DEMO_DIR "${MARS_MODE1_PKG_DIR}/../.." ABSOLUTE)

add_subdirectory(
  ${MARS_DEMO_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/mars_demo)

add_executable(mode1_rviz_node src/mode1_rviz_node.cpp)
target_compile_features(mode1_rviz_node PRIVATE cxx_std_17)
target_compile_definitions(mode1_rviz_node PRIVATE
  MARS_MODE1_MAP_DIR="${MARS_DEMO_DIR}/maps")
target_include_directories(mode1_rviz_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(mode1_rviz_node PRIVATE mars::demo_simulation ${catkin_LIBRARIES})

install(TARGETS mode1_rviz_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
