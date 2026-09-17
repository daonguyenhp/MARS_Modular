#pragma once
#include <mars/perception_geometry/geometry.hpp>
#include <algorithm>

namespace mars::perception_geometry::detail {
struct Span { double start; double end; };

inline std::vector<Span> linear_union(const std::vector<AngularInterval>& intervals) {
    std::vector<Span> spans;
    bool full=false;
    for (const auto& interval : intervals) {
        validate_interval(interval);
        if (interval.sweep<=angular_epsilon) continue;
        if (interval.sweep>=two_pi-angular_epsilon) { full=true; continue; }
        const double end=interval.start+interval.sweep;
        if (end<=two_pi) spans.push_back({interval.start,end});
        else { spans.push_back({interval.start,two_pi}); spans.push_back({0,end-two_pi}); }
    }
    if (full) return {{0,two_pi}};
    std::sort(spans.begin(),spans.end(),[](const Span& a,const Span& b) {
        return a.start<b.start || (a.start==b.start && a.end<b.end);
    });
    std::vector<Span> merged;
    for (const auto& span:spans) {
        if (!merged.empty() && span.start<=merged.back().end+angular_epsilon)
            merged.back().end=std::max(merged.back().end,span.end);
        else merged.push_back(span);
    }
    return merged;
}

inline std::vector<AngularInterval> circular_union(const std::vector<AngularInterval>& input) {
    auto spans=linear_union(input);
    if (spans.empty()) return {};
    if (spans.size()==1 && spans.front().end-spans.front().start>=two_pi-angular_epsilon)
        return {{0,two_pi}};
    std::vector<AngularInterval> result;
    const double seam_gap=spans.front().start+two_pi-spans.back().end;
    if (spans.size()>1 && seam_gap<=angular_epsilon) {
        result.push_back({spans.back().start,two_pi-spans.back().start+spans.front().end});
        spans.erase(spans.begin()); spans.pop_back();
    }
    for (const auto& span:spans) result.push_back({span.start,span.end-span.start});
    std::sort(result.begin(),result.end(),[](const AngularInterval& a,const AngularInterval& b) {
        return a.start<b.start;
    });
    return result;
}
inline bool contains(const AngularInterval& interval,double angle) {
    const double offset=normalize_angle(angle-interval.start);
    return interval.sweep==two_pi || offset<=interval.sweep+angular_epsilon ||
           two_pi-offset<=angular_epsilon;
}
}  // namespace mars::perception_geometry::detail
