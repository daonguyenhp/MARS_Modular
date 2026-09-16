#pragma once
#include <mars/perception_geometry/types.hpp>

namespace mars::perception_geometry {
// Circular complement of the union of blocked intervals. Input may be
// unsorted/overlapping. Empty closed coverage returns canonical {0, 2*pi}.
std::vector<OpenSight> compute_open_sights(const std::vector<ClosedSight>& closed);
}  // namespace mars::perception_geometry
