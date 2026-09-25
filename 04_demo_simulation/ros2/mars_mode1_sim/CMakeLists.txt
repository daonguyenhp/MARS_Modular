cmake_minimum_required(VERSION 3.16)
project(mars_mode1_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(BUILD_TESTING OFF CACHE BOOL "Skip sibling module tests in the RViz package" FORCE)
set(MARS_BUILD_TESTS OFF CACHE BOOL "Skip graph-bundle tests in the RViz package" FORCE)
set(MARS_DEMO_BUILD_TESTS OFF CACHE BOOL "Skip Mode 1 unit tests in the RViz package" FORCE)

add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../..
  ${CMAKE_CURRENT_BINARY_DIR}/mars_modular)

add_executable(mode1_rviz_node src/mode1_rviz_node.cpp)
target_link_libraries(mode1_rviz_node mars::demo_simulation)
ament_target_dependencies(mode1_rviz_node
  ament_index_cpp
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  visualization_msgs)

install(TARGETS mode1_rviz_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../maps/
  DESTINATION share/${PROJECT_NAME}/maps
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.pgm")

ament_package()
