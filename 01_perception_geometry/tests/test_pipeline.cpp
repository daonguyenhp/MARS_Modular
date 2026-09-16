#include <mars/perception_geometry/perception_pipeline.hpp>
#include <mars/perception_geometry/geometry.hpp>
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
using namespace mars::perception_geometry;

namespace {
double coverage(const PerceptionResult& result) {
    double sum=0;
    for (const auto& sight:result.closed_sights) sum+=sight.interval.sweep;
    return sum;
}
void check_partition(const PerceptionResult& result) {
    double total=coverage(result);
    for (const auto& sight:result.open_sights) total+=sight.interval.sweep;
    EXPECT_NEAR(total,two_pi,20*angular_epsilon);
    ASSERT_EQ(result.open_points.size(),result.open_sights.size());
    for (std::size_t i=0;i<result.open_points.size();++i) {
        const auto& point=result.open_points[i];
        EXPECT_EQ(point.sight_index,i);
        EXPECT_NEAR(distance(point.point,result.neighbor_sight.center),result.neighbor_sight.radius,1e-8);
        EXPECT_NEAR(point.angle,angular_midpoint(result.open_sights[i].interval),1e-12);
    }
}
// Independent test oracle: solve the 2x2 ray/edge equations directly, with no
// production geometry helpers, no critical-angle subdivision and no clipping.
double oracle_hit(const Point2D& origin,double angle,const std::vector<Segment2D>& edges,double radius) {
    double nearest=std::numeric_limits<double>::infinity();
    const double dx=std::cos(angle),dy=std::sin(angle);
    for (const auto& edge:edges) {
        const double ex=edge.end.x-edge.start.x,ey=edge.end.y-edge.start.y;
        const double qx=edge.start.x-origin.x,qy=edge.start.y-origin.y;
        const double det=dx*ey-dy*ex;
        if (std::abs(det)<1e-14) continue;
        const double t=(qx*ey-qy*ex)/det,u=(qx*dy-qy*dx)/det;
        if (t>=0 && t<=radius && u>=0 && u<=1) nearest=std::min(nearest,t);
    }
    return nearest;
}
bool blocked(const PerceptionResult& result,double angle) {
    for (const auto& sight:result.closed_sights)
        if (normalize_angle(angle-sight.interval.start)<sight.interval.sweep) return true;
    return false;
}
}  // namespace

TEST(Pipeline, CaseAEmptyEnvironment) {
    const auto r=perceive({1,2},5,{});
    EXPECT_TRUE(r.neighbor_sight.visible_boundaries.empty());
    EXPECT_TRUE(r.closed_sights.empty());
    ASSERT_EQ(r.open_sights.size(),1u); ASSERT_EQ(r.open_points.size(),1u);
    EXPECT_DOUBLE_EQ(r.open_sights[0].interval.sweep,two_pi);
    EXPECT_NEAR(r.open_points[0].point.x,-4,1e-12);
    EXPECT_NEAR(r.open_points[0].point.y,2,1e-12);
    check_partition(r);
}
TEST(Pipeline, CaseBOneWall) {
    const auto r=perceive({0,0},5,{{{{2,-2},{2,2}}}});
    ASSERT_EQ(r.closed_sights.size(),1u); ASSERT_EQ(r.open_sights.size(),1u);
    EXPECT_NEAR(coverage(r),pi/2,1e-12);
    EXPECT_NEAR(r.open_points[0].angle,pi,1e-12);
    check_partition(r);
}
TEST(Pipeline, CaseCTwoSeparatedWalls) {
    const auto r=perceive({0,0},5,{{{{2,-2},{2,2}}},{{{-2,-2},{-2,2}}}});
    EXPECT_EQ(r.closed_sights.size(),2u); EXPECT_EQ(r.open_sights.size(),2u);
    EXPECT_NEAR(coverage(r),pi,1e-12); check_partition(r);
}
TEST(Pipeline, CaseDNearWallHidesFarWall) {
    const auto r=perceive({0,0},10,{{{{2,-2},{2,2}}},{{{4,-3},{4,3}}}});
    ASSERT_EQ(r.neighbor_sight.visible_boundaries.size(),1u);
    EXPECT_NEAR(r.neighbor_sight.visible_boundaries[0].start.x,2,1e-12);
    EXPECT_NEAR(coverage(r),pi/2,1e-12); check_partition(r);
}
TEST(Pipeline, CaseECorridor) {
    const auto r=perceive({0,0},5,{{{{-10,1},{10,1}}},{{{-10,-1},{10,-1}}}});
    ASSERT_EQ(r.closed_sights.size(),2u); ASSERT_EQ(r.open_points.size(),2u);
    EXPECT_NEAR(r.open_points[0].angle,pi,1e-12);
    EXPECT_NEAR(r.open_points[1].angle,0,1e-12);
    EXPECT_NEAR(coverage(r),two_pi-4*std::asin(0.2),1e-12); check_partition(r);
}
TEST(Pipeline, CaseFBlindAlleyPerceptionOnly) {
    const auto r=perceive({0,0},5,
        {{{{-2,-2},{2,-2}}},{{{2,-2},{2,2}}},{{{2,2},{-2,2}}}});
    ASSERT_EQ(r.closed_sights.size(),1u); ASSERT_EQ(r.open_points.size(),1u);
    EXPECT_NEAR(coverage(r),3*pi/2,1e-12);
    EXPECT_NEAR(r.open_sights[0].interval.sweep,pi/2,1e-12);
    EXPECT_NEAR(r.open_points[0].angle,pi,1e-12); check_partition(r);
}
TEST(Pipeline, CaseGMovedObserver) {
    const std::vector<Polygon2D> map{{{{2,-1},{2,1}}}};
    const auto a=perceive({0,0},5,map), b=perceive({1,0},5,map);
    EXPECT_GT(coverage(b),coverage(a));
    EXPECT_NEAR(coverage(a),2*std::atan(0.5),1e-12);
    EXPECT_NEAR(coverage(b),pi/2,1e-12); check_partition(b);
}
TEST(Pipeline, CaseHChangedRadius) {
    const std::vector<Polygon2D> map{{{{3,-4},{3,4}}}};
    EXPECT_TRUE(perceive({0,0},2,map).closed_sights.empty());
    const auto a=perceive({0,0},4,map), b=perceive({0,0},5,map);
    EXPECT_GT(coverage(b),coverage(a));
    EXPECT_NEAR(coverage(a),2*std::acos(0.75),1e-12); check_partition(a); check_partition(b);
}
TEST(Pipeline, FullyEnclosedBySeparateWalls) {
    const auto r=perceive({0,0},5,{{{{-1,-1},{1,-1}}},{{{1,-1},{1,1}}},
                                 {{{1,1},{-1,1}}},{{{-1,1},{-1,-1}}}});
    ASSERT_EQ(r.closed_sights.size(),1u);
    EXPECT_DOUBLE_EQ(r.closed_sights[0].interval.start,0);
    EXPECT_DOUBLE_EQ(r.closed_sights[0].interval.sweep,two_pi);
    EXPECT_TRUE(r.open_sights.empty()); EXPECT_TRUE(r.open_points.empty()); check_partition(r);
}
TEST(Pipeline, ConcaveFilledUHasAnOpenMouth) {
    const auto r=perceive({0,0},5,
        {{{{-3,-3},{3,-3},{3,3},{2,3},{2,-2},{-2,-2},{-2,3},{-3,3}}}});
    ASSERT_EQ(r.open_sights.size(),1u);
    EXPECT_NEAR(r.open_sights[0].interval.sweep,2*std::atan2(2.0,3.0),1e-12);
    EXPECT_NEAR(r.open_points[0].angle,pi/2,1e-12);
    check_partition(r);
}
TEST(Pipeline, NarrowObstacleBetweenUniformScanAngles) {
    const double angle=0.123456,half_width=1e-5;
    const Point2D a{2*std::cos(angle-half_width),2*std::sin(angle-half_width)};
    const Point2D b{2*std::cos(angle+half_width),2*std::sin(angle+half_width)};
    const auto r=perceive({0,0},5,{{{a,b}}});
    ASSERT_EQ(r.closed_sights.size(),1u);
    EXPECT_NEAR(coverage(r),2*half_width,1e-12);
}
TEST(Pipeline, TranslationRotationScaleAndInputOrder) {
    std::vector<Polygon2D> map{{{{2,-1},{4,-1},{4,1},{2,1}}},{{{-2,-2},{-2,2}}}};
    const auto base=perceive({0,0},5,map);
    const double rotation=0.37, scale=2.5;
    const Point2D offset{100,-30};
    for (auto& polygon:map) {
        for (auto& p:polygon.vertices) {
            const double x=p.x,y=p.y;
            p={offset.x+scale*(x*std::cos(rotation)-y*std::sin(rotation)),
               offset.y+scale*(x*std::sin(rotation)+y*std::cos(rotation))};
        }
        std::reverse(polygon.vertices.begin(),polygon.vertices.end());
    }
    std::reverse(map.begin(),map.end());
    const auto moved=perceive(offset,5*scale,map);
    EXPECT_EQ(base.closed_sights.size(),moved.closed_sights.size());
    EXPECT_NEAR(coverage(base),coverage(moved),1e-12);
    for (int i=0;i<300;++i) {
        const double angle=two_pi*(i+0.321)/300;
        EXPECT_EQ(blocked(base,angle),blocked(moved,normalize_angle(angle+rotation)));
    }
    check_partition(moved);
}
TEST(Pipeline, DeterministicRandomWallsMatchIndependentRayOracle) {
    std::mt19937 generator(1729);
    std::uniform_real_distribution<double> coordinate(-6,6);
    for (int sample=0;sample<30;++sample) {
        SCOPED_TRACE(sample);
        std::vector<Polygon2D> map;
        std::vector<Segment2D> edges;
        for (int i=0;i<8;++i) {
            Segment2D edge{{coordinate(generator),coordinate(generator)},
                           {coordinate(generator),coordinate(generator)}};
            edges.push_back(edge); map.push_back({{edge.start,edge.end}});
        }
        const auto r=perceive({0,0},5,map);
        check_partition(r);
        for (int i=0;i<400;++i) {
            const double angle=two_pi*(i+0.371)/400;
            const double expected=oracle_hit({0,0},angle,edges,5);
            const double actual=oracle_hit({0,0},angle,r.neighbor_sight.visible_boundaries,5+1e-9);
            ASSERT_EQ(std::isfinite(actual),std::isfinite(expected)) << angle;
            EXPECT_EQ(blocked(r,angle),std::isfinite(expected)) << angle;
            if (std::isfinite(expected)) {
                EXPECT_NEAR(actual,expected,1e-8) << angle;
            }
        }
    }
}
TEST(Pipeline, InvalidInputAndStatelessRepeatability) {
    EXPECT_THROW(perceive({0,0},-1,{}),std::invalid_argument);
    EXPECT_THROW(perceive({0,0},5,{{{{std::numeric_limits<double>::infinity(),0},{2,1}}}}),std::invalid_argument);
    const std::vector<Polygon2D> map{{{{2,-2},{2,2}}}};
    const auto a=perceive({0,0},5,map);
    perceive({1,0},2,map);
    const auto b=perceive({0,0},5,map);
    ASSERT_EQ(a.open_points.size(),b.open_points.size());
    EXPECT_DOUBLE_EQ(a.open_points[0].point.x,b.open_points[0].point.x);
    EXPECT_DOUBLE_EQ(a.open_points[0].point.y,b.open_points[0].point.y);
}
