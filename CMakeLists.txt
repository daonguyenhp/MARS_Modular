cmake_minimum_required(VERSION 3.16)
project(mars_modular LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Keep clangd/cmake configure light; turn tests on when you want them.
option(BUILD_TESTING "Build module tests" OFF)
option(MARS_BUILD_TESTS "Build graph-bundle tests/examples" OFF)

if(NOT TARGET mars::common)
  add_library(mars_common INTERFACE)
  add_library(mars::common ALIAS mars_common)
  target_include_directories(mars_common INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/common/cpp/include)
endif()

add_subdirectory(01_perception_geometry)
add_subdirectory(02_graph_bundle_management)
add_subdirectory(03_navigation_decision)
add_subdirectory(04_demo_simulation)
