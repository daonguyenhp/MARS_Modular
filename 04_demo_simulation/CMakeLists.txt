cmake_minimum_required(VERSION 3.16)
project(mars_demo_simulation VERSION 0.1.0 LANGUAGES CXX)

# Module 1 tests require GTest. Keep that switch off unless the caller set it.
if(NOT DEFINED BUILD_TESTING)
  set(BUILD_TESTING OFF CACHE BOOL "Build module tests" FORCE)
endif()

if(NOT TARGET mars::navigation_decision)
  if(NOT TARGET mars::common)
    add_library(mars_common INTERFACE)
    add_library(mars::common ALIAS mars_common)
    target_include_directories(mars_common INTERFACE
      ${CMAKE_CURRENT_SOURCE_DIR}/../common/cpp/include)
  endif()
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../01_perception_geometry
                   ${CMAKE_CURRENT_BINARY_DIR}/01_perception_geometry)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../02_graph_bundle_management
                   ${CMAKE_CURRENT_BINARY_DIR}/02_graph_bundle_management)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../03_navigation_decision
                   ${CMAKE_CURRENT_BINARY_DIR}/03_navigation_decision)
endif()

add_library(mars_demo_simulation
  maps/mode1_maps.cpp
  launch/mode1_shell.cpp
  launch/path_follower.cpp
  launch/mode1_mission.cpp
  visualization/mode1_trace.cpp)
add_library(mars::demo_simulation ALIAS mars_demo_simulation)

target_compile_features(mars_demo_simulation PUBLIC cxx_std_17)
set_target_properties(mars_demo_simulation PROPERTIES
  CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
target_link_libraries(mars_demo_simulation
  PUBLIC
    mars::perception_geometry
    mars::graph_bundle_management
    mars::navigation_decision)
target_include_directories(mars_demo_simulation
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../03_navigation_decision/include>
    $<INSTALL_INTERFACE:include>)

if(MSVC)
  target_compile_options(mars_demo_simulation PRIVATE /W4 /permissive-)
else()
  target_compile_options(mars_demo_simulation PRIVATE
    -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
endif()

add_executable(polygon_explore_sim launch/polygon_explore_sim.cpp)
target_link_libraries(polygon_explore_sim PRIVATE mars::demo_simulation)
if(MSVC)
  target_compile_options(polygon_explore_sim PRIVATE /W4 /permissive-)
else()
  target_compile_options(polygon_explore_sim PRIVATE
    -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
endif()

file(TO_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/maps" MARS_MODE1_MAP_DIR)
target_compile_definitions(polygon_explore_sim PRIVATE
  MARS_MODE1_MAP_DIR="${MARS_MODE1_MAP_DIR}")

option(MARS_DEMO_BUILD_TESTS "Build Mode 1 map and launch tests" ON)
if(MARS_DEMO_BUILD_TESTS)
  enable_testing()
  add_executable(mars_mode1_launch_tests tests/test_mode1_launch.cpp)
  target_link_libraries(mars_mode1_launch_tests PRIVATE mars::demo_simulation)
  if(MSVC)
    target_compile_options(mars_mode1_launch_tests PRIVATE /W4 /permissive-)
  else()
    target_compile_options(mars_mode1_launch_tests PRIVATE
      -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
  endif()
  target_compile_definitions(mars_mode1_launch_tests PRIVATE
    MARS_MODE1_MAP_DIR="${MARS_MODE1_MAP_DIR}")
  add_test(NAME mars_mode1_launch_tests COMMAND mars_mode1_launch_tests)
  add_test(NAME polygon_explore_sim_hard_alley
    COMMAND polygon_explore_sim --map hard_alley)
  add_test(NAME polygon_explore_sim_geogebra
    COMMAND polygon_explore_sim --map geogebra)
endif()
