#include <mars/perception_geometry/geometry.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <stdexcept>
using namespace mars::perception_geometry;

TEST(Geometry, DistanceAndCircle) {
    EXPECT_DOUBLE_EQ(distance({0,0},{3,4}),5);
    const auto p = point_on_circle({{1,2},3},pi/2);
    EXPECT_NEAR(p.x,1,1e-12); EXPECT_NEAR(p.y,5,1e-12);
}
TEST(Geometry, AngularConventions) {
    EXPECT_DOUBLE_EQ(normalize_angle(two_pi),0);
    EXPECT_NEAR(normalize_angle(-pi/2),3*pi/2,1e-12);
    EXPECT_NEAR(angular_midpoint({350*pi/180,30*pi/180}),5*pi/180,1e-12);
    EXPECT_DOUBLE_EQ(angular_midpoint({0,two_pi}),pi);
    EXPECT_THROW(angular_midpoint({0,0}),std::invalid_argument);
    EXPECT_THROW(validate_interval({two_pi,1}),std::invalid_argument);
    EXPECT_THROW(validate_interval({0,-1}),std::invalid_argument);
    EXPECT_THROW(validate_interval({0,two_pi+1}),std::invalid_argument);
}
TEST(Geometry, InvalidValues) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    EXPECT_THROW(distance({nan,0},{0,0}),std::invalid_argument);
    EXPECT_THROW(normalize_angle(inf),std::invalid_argument);
    for (double r : {0.0,-1.0,linear_epsilon,nan,inf})
        EXPECT_THROW(validate_circle({{0,0},r}),std::invalid_argument);
    EXPECT_THROW(validate_interval({nan,1}),std::invalid_argument);
    EXPECT_THROW(distance({1e308,0},{-1e308,0}),std::overflow_error);
}
TEST(Geometry, RayHitsAndMisses) {
    auto p = ray_segment_intersection({0,0},0,{{2,-1},{2,1}});
    ASSERT_TRUE(p); EXPECT_NEAR(p->x,2,1e-12); EXPECT_NEAR(p->y,0,1e-12);
    EXPECT_FALSE(ray_segment_intersection({0,0},pi,{{2,-1},{2,1}}));
    EXPECT_FALSE(ray_segment_intersection({0,0},0,{{1,1},{3,1}}));
    p = ray_segment_intersection({0,0},pi/4,{{1,1},{1,2}});
    ASSERT_TRUE(p); EXPECT_NEAR(p->y,1,1e-12);
}
TEST(Geometry, CollinearAndDegenerateRays) {
    auto p = ray_segment_intersection({0,0},0,{{3,0},{1,0}});
    ASSERT_TRUE(p); EXPECT_DOUBLE_EQ(p->x,1);
    p = ray_segment_intersection({0,0},0,{{-1,0},{2,0}});
    ASSERT_TRUE(p); EXPECT_DOUBLE_EQ(p->x,0);
    EXPECT_FALSE(ray_segment_intersection({0,0},0,{{-3,0},{-1,0}}));
    EXPECT_TRUE(ray_segment_intersection({0,0},0,{{1,0},{1,0}}));
    EXPECT_FALSE(ray_segment_intersection({0,0},0,{{1,1},{1,1}}));
    p = ray_segment_intersection({0,0},pi,{{-3,0},{-1,0}});
    ASSERT_TRUE(p); EXPECT_NEAR(p->x,-1,1e-12);
    p = ray_segment_intersection({0,0},pi/2,{{0,3},{0,1}});
    ASSERT_TRUE(p); EXPECT_NEAR(p->y,1,1e-12);
}
TEST(Geometry, CircleIntersectionsAndClipping) {
    const Circle2D c{{0,0},2};
    auto hits = segment_circle_intersections({{-3,0},{3,0}},c);
    ASSERT_EQ(hits.size(),2u); EXPECT_NEAR(hits[0].x,-2,1e-12); EXPECT_NEAR(hits[1].x,2,1e-12);
    auto clipped = clip_segment_to_circle({{-3,0},{3,0}},c);
    ASSERT_TRUE(clipped); EXPECT_NEAR(clipped->start.x,-2,1e-12);
    EXPECT_NEAR(clipped->end.x,2,1e-12);
    EXPECT_TRUE(segment_circle_intersections({{-1,0},{1,0}},c).empty());
    clipped = clip_segment_to_circle({{-1,0},{1,0}},c);
    ASSERT_TRUE(clipped); EXPECT_DOUBLE_EQ(clipped->start.x,-1);
    EXPECT_FALSE(clip_segment_to_circle({{3,0},{4,0}},c));
}
TEST(Geometry, TangentAndNearTangent) {
    const Circle2D c{{0,0},1};
    EXPECT_EQ(segment_circle_intersections({{-2,1},{2,1}},c).size(),1u);
    EXPECT_EQ(segment_circle_intersections({{-2,1-1e-8},{2,1-1e-8}},c).size(),2u);
    EXPECT_TRUE(segment_circle_intersections({{-2,1+1e-8},{2,1+1e-8}},c).empty());
    EXPECT_EQ(segment_circle_intersections({{1,0},{1,0}},c).size(),1u);
}
