#include <mars/perception_geometry/open_points.hpp>
#include <mars/perception_geometry/geometry.hpp>
#include <gtest/gtest.h>
#include <stdexcept>
using namespace mars::perception_geometry;

TEST(OpenPoints, SingleSector) {
    const auto p=compute_open_point({1,2},3,{{0,pi}});
    EXPECT_NEAR(p.angle,pi/2,1e-12);
    EXPECT_NEAR(p.point.x,1,1e-12); EXPECT_NEAR(p.point.y,5,1e-12);
    EXPECT_FALSE(p.sight_index);
}
TEST(OpenPoints, MultipleAndWrappingSectors) {
    const auto points=compute_open_points({0,0},2,{{{7*pi/4,pi/2}},{{pi/2,pi}}});
    ASSERT_EQ(points.size(),2u);
    EXPECT_NEAR(points[0].angle,0,1e-12); EXPECT_NEAR(points[0].point.x,2,1e-12);
    EXPECT_NEAR(points[1].angle,pi,1e-12); EXPECT_NEAR(points[1].point.x,-2,1e-12);
    ASSERT_TRUE(points[0].sight_index); EXPECT_EQ(*points[0].sight_index,0u);
    ASSERT_TRUE(points[1].sight_index); EXPECT_EQ(*points[1].sight_index,1u);
}
TEST(OpenPoints, FullCircleAndDeterminism) {
    const auto first=compute_open_point({3,4},2,{{0,two_pi}},7);
    const auto second=compute_open_point({3,4},2,{{0,two_pi}},7);
    EXPECT_DOUBLE_EQ(first.angle,pi);
    EXPECT_NEAR(first.point.x,1,1e-12); EXPECT_NEAR(first.point.y,4,1e-12);
    EXPECT_DOUBLE_EQ(first.point.x,second.point.x);
    EXPECT_DOUBLE_EQ(first.point.y,second.point.y);
    EXPECT_EQ(first.sight_index,second.sight_index);
}
TEST(OpenPoints, EmptyListAndInvalidInput) {
    EXPECT_TRUE(compute_open_points({0,0},1,{}).empty());
    EXPECT_THROW(compute_open_points({0,0},-1,{}),std::invalid_argument);
    EXPECT_THROW(compute_open_point({0,0},1,{{0,0}}),std::invalid_argument);
}
