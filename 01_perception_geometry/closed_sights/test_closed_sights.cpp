#include <mars/perception_geometry/closed_sights.hpp>
#include <mars/perception_geometry/neighbor_sight.hpp>
#include <mars/perception_geometry/geometry.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <stdexcept>
using namespace mars::perception_geometry;

TEST(ClosedSights, WallCrossingZero) {
    const auto sights=compute_closed_sights(compute_neighbor_sight({0,0},5,{{{{2,-2},{2,2}}}}));
    ASSERT_EQ(sights.size(),1u);
    EXPECT_NEAR(sights[0].interval.start,7*pi/4,1e-12);
    EXPECT_NEAR(sights[0].interval.sweep,pi/2,1e-12);
    EXPECT_EQ(sights[0].visible_boundaries.size(),1u);
}
TEST(ClosedSights, SeparatedWallsAreOrdered) {
    const auto sights=compute_closed_sights(compute_neighbor_sight({0,0},5,
        {{{{2,-1},{2,1}}},{{{-2,-1},{-2,1}}}}));
    ASSERT_EQ(sights.size(),2u);
    EXPECT_LT(sights[0].interval.start,sights[1].interval.start);
}
TEST(ClosedSights, OverlappingProjectionsMergeVisibleFragments) {
    const auto sights=compute_closed_sights(compute_neighbor_sight({0,0},10,
        {{{{2,-0.5},{2,0.5}}},{{{4,-3},{4,3}}}}));
    ASSERT_EQ(sights.size(),1u);
    EXPECT_NEAR(sights[0].interval.sweep,2*std::atan2(3.0,4.0),1e-12);
    EXPECT_EQ(sights[0].visible_boundaries.size(),3u);
}
TEST(ClosedSights, HiddenWallNotIncluded) {
    const auto sights=compute_closed_sights(compute_neighbor_sight({0,0},10,
        {{{{2,-1},{2,1}}},{{{4,-2},{4,2}}}}));
    ASSERT_EQ(sights.size(),1u);
    ASSERT_EQ(sights[0].visible_boundaries.size(),1u);
    EXPECT_NEAR(sights[0].visible_boundaries[0].start.x,2,1e-12);
}
TEST(ClosedSights, EmptyDegenerateAndTiny) {
    EXPECT_TRUE(compute_closed_sights({{0,0},5,{}}).empty());
    EXPECT_TRUE(compute_closed_sights({{0,0},5,{{{1,0},{1,0}}}}).empty());
    EXPECT_TRUE(compute_closed_sights({{0,0},100,{{{50,0},{50,2e-9}}}}).empty());
}
TEST(ClosedSights, RejectsInvalidBoundaries) {
    EXPECT_THROW(compute_closed_sights({{0,0},1,{{{2,-1},{2,1}}}}),std::invalid_argument);
    EXPECT_THROW(compute_closed_sights({{0,0},5,{{{-1,0},{1,0}}}}),std::invalid_argument);
}
