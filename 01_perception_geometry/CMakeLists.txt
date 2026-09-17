cmake_minimum_required(VERSION 3.16)
project(mars_perception_geometry VERSION 0.1.0 LANGUAGES CXX)

add_library(mars_perception_geometry geometry/geometry.cpp neighbor_sight/neighbor_sight.cpp
    closed_sights/closed_sights.cpp open_sights/open_sights.cpp open_sights/open_points.cpp pipeline/perception_pipeline.cpp)
add_library(mars::perception_geometry ALIAS mars_perception_geometry)
target_compile_features(mars_perception_geometry PUBLIC cxx_std_17)
set_target_properties(mars_perception_geometry PROPERTIES
    CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
target_include_directories(mars_perception_geometry PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/cpp/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/internal)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mars_perception_geometry PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(CTest)
if(BUILD_TESTING)
    find_package(GTest CONFIG REQUIRED)
    add_executable(perception_tests geometry/test_geometry.cpp neighbor_sight/test_neighbor_sight.cpp
        closed_sights/test_closed_sights.cpp open_sights/test_open_sights.cpp open_sights/test_open_points.cpp
        pipeline/test_pipeline.cpp)
    target_link_libraries(perception_tests PRIVATE mars::perception_geometry GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(perception_tests)
endif()
