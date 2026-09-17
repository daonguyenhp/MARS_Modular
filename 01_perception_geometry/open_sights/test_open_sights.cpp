#include <mars/perception_geometry/open_sights.hpp>
#include <mars/perception_geometry/geometry.hpp>
#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>
using namespace mars::perception_geometry;

TEST(OpenSights, EmptyAndFullCoverage) {
    const auto open=compute_open_sights({});
    ASSERT_EQ(open.size(),1u);
    EXPECT_DOUBLE_EQ(open[0].interval.start,0);
    EXPECT_DOUBLE_EQ(open[0].interval.sweep,two_pi);
    EXPECT_TRUE(compute_open_sights({{{1,two_pi},{}}}).empty());
    EXPECT_EQ(compute_open_sights({{{1,0},{}}}).size(),1u);
}
TEST(OpenSights, WrappingClosedSector) {
    const auto open=compute_open_sights({{{7*pi/4,pi/2},{}}});
    ASSERT_EQ(open.size(),1u);
    EXPECT_NEAR(open[0].interval.start,pi/4,1e-12);
    EXPECT_NEAR(open[0].interval.sweep,3*pi/2,1e-12);
}
TEST(OpenSights, WrappingOpenSector) {
    const auto open=compute_open_sights({{{pi/4,pi/2},{}}});
    ASSERT_EQ(open.size(),1u);
    EXPECT_NEAR(open[0].interval.start,3*pi/4,1e-12);
    EXPECT_NEAR(open[0].interval.sweep,3*pi/2,1e-12);
}
TEST(OpenSights, UnsortedOverlappingNestedAndDuplicateIntervals) {
    const auto open=compute_open_sights({{{2,2},{}},{{1,2},{}},{{1.5,0.1},{}},{{2,2},{}}});
    ASSERT_EQ(open.size(),1u);
    EXPECT_DOUBLE_EQ(open[0].interval.start,4);
    EXPECT_NEAR(open[0].interval.sweep,two_pi-3,1e-12);
}
TEST(OpenSights, MultipleOpenRegionsAndTouchingClosedIntervals) {
    auto open=compute_open_sights({{{0,pi/2},{}},{{pi,pi/2},{}}});
    ASSERT_EQ(open.size(),2u);
    EXPECT_NEAR(open[0].interval.start,pi/2,1e-12);
    EXPECT_NEAR(open[1].interval.start,3*pi/2,1e-12);
    EXPECT_NEAR(open[0].interval.sweep,pi/2,1e-12);
    EXPECT_NEAR(open[1].interval.sweep,pi/2,1e-12);
    EXPECT_TRUE(compute_open_sights({{{0,pi},{}},{{pi,pi},{}}}).empty());
}
TEST(OpenSights, TinyBlockedSectorsAndGaps) {
    auto open=compute_open_sights({{{1,angular_epsilon/2},{}}});
    ASSERT_EQ(open.size(),1u); EXPECT_DOUBLE_EQ(open[0].interval.sweep,two_pi);
    EXPECT_TRUE(compute_open_sights({{{0,1},{}},{{1+angular_epsilon/2,two_pi-1-angular_epsilon/2},{}}}).empty());
    open=compute_open_sights({{{0,1},{}},{{1+10*angular_epsilon,two_pi-1-10*angular_epsilon},{}}});
    ASSERT_EQ(open.size(),1u); EXPECT_NEAR(open[0].interval.sweep,10*angular_epsilon,1e-15);
}
TEST(OpenSights, InvalidIntervalsEvenAfterFullCoverage) {
    EXPECT_THROW(compute_open_sights({{{0,two_pi},{}},{{-1,1},{}}}),std::invalid_argument);
    EXPECT_THROW(compute_open_sights({{{0,std::numeric_limits<double>::quiet_NaN()},{}}}),std::invalid_argument);
}
TEST(OpenSights, SeamGapUsesCombinedWidth) {
    const double half=0.75*angular_epsilon;
    const auto open=compute_open_sights({{{half,two_pi-2*half},{}}});
    ASSERT_EQ(open.size(),1u);
    EXPECT_NEAR(open[0].interval.sweep,2*half,1e-15);
}
