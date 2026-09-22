#include <mars/perception_geometry/neighbor_sight.hpp>
#include <mars/perception_geometry/geometry.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <stdexcept>
using namespace mars::perception_geometry;

TEST(NeighborSight, EmptyAndOutside) {
    EXPECT_TRUE(compute_neighbor_sight({0,0},5,{}).visible_boundaries.empty());
    EXPECT_TRUE(compute_neighbor_sight({0,0},5,{{{{6,-1},{6,1}}}}).visible_boundaries.empty());
}
TEST(NeighborSight, WallAndNearFarOcclusion) {
    const auto s=compute_neighbor_sight({0,0},10,{{{{2,-1},{2,1}}},{{{4,-2},{4,2}}}});
    ASSERT_EQ(s.visible_boundaries.size(),1u);
    EXPECT_NEAR(s.visible_boundaries[0].start.x,2,1e-12);
    EXPECT_NEAR(s.visible_boundaries[0].start.y,-1,1e-12);
    EXPECT_NEAR(s.visible_boundaries[0].end.y,1,1e-12);
}
TEST(NeighborSight, PartialOcclusionPreservesFarFragments) {
    const auto s=compute_neighbor_sight({0,0},10,{{{{2,-0.5},{2,0.5}}},{{{4,-3},{4,3}}}});
    ASSERT_EQ(s.visible_boundaries.size(),3u);
    double far_length=0;
    for (const auto& edge:s.visible_boundaries)
        if (std::abs(edge.start.x-4)<1e-8) far_length+=distance(edge.start,edge.end);
    EXPECT_NEAR(far_length,4,1e-9);
}
TEST(NeighborSight, FilledRectangleShowsOnlyFrontFace) {
    const auto s=compute_neighbor_sight({0,0},10,{{{{2,-1},{4,-1},{4,1},{2,1}}}});
    ASSERT_EQ(s.visible_boundaries.size(),1u);
    EXPECT_NEAR(s.visible_boundaries[0].start.x,2,1e-12);
    EXPECT_NEAR(s.visible_boundaries[0].end.x,2,1e-12);
}
TEST(NeighborSight, CircleClipsLongWall) {
    const auto s=compute_neighbor_sight({0,0},5,{{{{3,-10},{3,10}}}});
    ASSERT_EQ(s.visible_boundaries.size(),1u);
    EXPECT_NEAR(s.visible_boundaries[0].start.y,-4,1e-9);
    EXPECT_NEAR(s.visible_boundaries[0].end.y,4,1e-9);
}
TEST(NeighborSight, TangentRadialAndDegenerateHaveZeroAngularMeasure) {
    const auto s=compute_neighbor_sight({0,0},5,
        {{{{5,-1},{5,1}}},{{{1,0},{2,0}}},{{{1,1},{1,1}}}});
    EXPECT_TRUE(s.visible_boundaries.empty());
}
TEST(NeighborSight, NearWallAndNearTangent) {
    EXPECT_FALSE(compute_neighbor_sight({0,0},2,{{{{1e-6,-1},{1e-6,1}}}}).visible_boundaries.empty());
    EXPECT_FALSE(compute_neighbor_sight({0,0},1,{{{{1-1e-8,-1},{1-1e-8,1}}}}).visible_boundaries.empty());
}
TEST(NeighborSight, CrossingWallsExchangeVisibility) {
    const auto s=compute_neighbor_sight({0,0},10,{{{{2,-2},{4,2}}},{{{4,-2},{2,2}}}});
    ASSERT_EQ(s.visible_boundaries.size(),2u);
    double length=0;
    for (const auto& edge:s.visible_boundaries) length+=distance(edge.start,edge.end);
    EXPECT_NEAR(length,2*std::sqrt(5.0),1e-9);
}
TEST(NeighborSight, DuplicateVerticesAndOverlappingWalls) {
    const auto s=compute_neighbor_sight({0,0},10,
        {{{{2,-1},{2,-1},{4,-1},{4,1},{2,1},{2,-1}}},{{{2,-1},{2,1}}}});
    double length=0;
    for (const auto& edge:s.visible_boundaries) length+=distance(edge.start,edge.end);
    EXPECT_NEAR(length,2,1e-9);
}
TEST(NeighborSight, RejectsInvalidObserverAndPolygon) {
    EXPECT_THROW(compute_neighbor_sight({0,0},5,{{{{-1,-1},{1,-1},{1,1},{-1,1}}}}),std::invalid_argument);
    EXPECT_THROW(compute_neighbor_sight({0,0},5,{{{{0,-1},{0,1}}}}),std::invalid_argument);
    EXPECT_THROW(compute_neighbor_sight({0,0},5,{{{{2,-1},{4,1},{2,1},{4,-1}}}}),std::invalid_argument);
    EXPECT_THROW(compute_neighbor_sight({0,0},0,{}),std::invalid_argument);
}
