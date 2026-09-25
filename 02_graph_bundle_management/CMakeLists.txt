cmake_minimum_required(VERSION 3.16)
project(mars_graph_bundle_management VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET mars::common)
  add_library(mars_common INTERFACE)
  add_library(mars::common ALIAS mars_common)
  target_include_directories(mars_common INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/cpp/include)
endif()

add_library(mars_graph_bundle_management
  ranking_memory/ranking.cpp
  ranking_memory/open_point_memory.cpp
  visibility_graph/visibility_graph.cpp
  visibility_graph/graph_search.cpp
  visibility_graph/shared_sight_connector.cpp
  bundle_sequences/bundle_builder.cpp
  bundle_sequences/bundle_history.cpp
  bundle_sequences/sequence_extractor.cpp
  bundle_sequences/gate_preparation.cpp
  pipeline/graph_bundle_manager.cpp)
add_library(mars::graph_bundle_management ALIAS mars_graph_bundle_management)

target_compile_features(mars_graph_bundle_management PUBLIC cxx_std_17)
target_link_libraries(mars_graph_bundle_management
  PUBLIC mars::common mars::perception_geometry)
target_include_directories(mars_graph_bundle_management
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

if(MSVC)
  target_compile_options(mars_graph_bundle_management PRIVATE /W4 /permissive-)
else()
  target_compile_options(mars_graph_bundle_management PRIVATE
    -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
endif()

if(MARS_BUILD_TESTS)
  add_executable(mars_graph_bundle_management_tests
    tests/test_graph_bundle_management.cpp)
  target_link_libraries(mars_graph_bundle_management_tests
    PRIVATE mars::graph_bundle_management)
  add_test(NAME mars_graph_bundle_management_tests
    COMMAND mars_graph_bundle_management_tests)

  add_executable(mars_graph_bundle_management_example
    examples/synthetic_update.cpp)
  target_link_libraries(mars_graph_bundle_management_example
    PRIVATE mars::graph_bundle_management)
endif()
