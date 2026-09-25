cmake_minimum_required(VERSION 3.16)
project(mars_navigation_decision VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET mars::common)
  add_library(mars_common INTERFACE)
  add_library(mars::common ALIAS mars_common)
  target_include_directories(mars_common INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/cpp/include)
endif()

add_library(mars_navigation_decision
  explore/explore.cpp
  bar_escape/bar_escape.cpp
  dap_funnel/dap_funnel.cpp
  pipeline/path_utils.cpp
  pipeline/navigator.cpp)
add_library(mars::navigation_decision ALIAS mars_navigation_decision)

target_compile_features(mars_navigation_decision PUBLIC cxx_std_17)
set_target_properties(mars_navigation_decision PROPERTIES
  CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
target_link_libraries(mars_navigation_decision PUBLIC mars::common)
target_include_directories(mars_navigation_decision
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../01_perception_geometry/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../02_graph_bundle_management/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR})

if(MSVC)
  target_compile_options(mars_navigation_decision PRIVATE /W4 /permissive-)
else()
  target_compile_options(mars_navigation_decision PRIVATE
    -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(mars_navigation_decision_tests
    tests/test_navigation_decision.cpp)
  target_link_libraries(mars_navigation_decision_tests
    PRIVATE mars::navigation_decision)
  add_test(NAME mars_navigation_decision_tests
    COMMAND mars_navigation_decision_tests)

  add_executable(mars_navigation_decision_example
    examples/synthetic_decide.cpp)
  target_link_libraries(mars_navigation_decision_example
    PRIVATE mars::navigation_decision)
endif()
