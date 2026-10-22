cmake_minimum_required(VERSION 3.16)
project(mars_online_gazebo)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

set(BUILD_TESTING OFF CACHE BOOL "Skip sibling module tests" FORCE)
set(MARS_BUILD_TESTS OFF CACHE BOOL "Skip graph-bundle tests" FORCE)
set(MARS_DEMO_BUILD_TESTS OFF CACHE BOOL "Skip Mode 1 tests" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../.. ${CMAKE_CURRENT_BINARY_DIR}/mars_modular)

add_executable(online_navigation_node src/online_navigation_node.cpp src/laser_geometry.cpp)
target_include_directories(online_navigation_node PRIVATE include)
target_link_libraries(online_navigation_node PRIVATE
  mars::demo_simulation
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  tf2_ros::tf2_ros
  visualization_msgs::visualization_msgs)
install(TARGETS online_navigation_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz worlds DESTINATION share/${PROJECT_NAME})
ament_package()
