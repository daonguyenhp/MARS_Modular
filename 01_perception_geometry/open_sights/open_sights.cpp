#include <mars/perception_geometry/open_sights.hpp>
#include "intervals.hpp"

namespace mars::perception_geometry {
std::vector<OpenSight> compute_open_sights(const std::vector<ClosedSight>& closed) {
    std::vector<AngularInterval> intervals;
    for (const auto& sight:closed) intervals.push_back(sight.interval);
    const auto spans=detail::linear_union(intervals);
    if (spans.empty()) return {{{0,two_pi}}};
    std::vector<OpenSight> result;
    // Evaluate the seam gap as one circular interval before applying epsilon.
    for (std::size_t i=0;i<spans.size();++i) {
        const double end=i+1<spans.size() ? spans[i+1].start : spans[0].start+two_pi;
        const double sweep=end-spans[i].end;
        if (sweep>angular_epsilon) result.push_back({{normalize_angle(spans[i].end),sweep}});
    }
    std::sort(result.begin(),result.end(),[](const OpenSight& a,const OpenSight& b) {
        return a.interval.start<b.interval.start;
    });
    return result;
}
}  // namespace mars::perception_geometry
