#include "mars/graph_bundle_management/open_point_memory.hpp"

#include <cmath>
#include <stdexcept>

#include "internal/geometry_helpers.hpp"

namespace mars::graph_bundle_management {

OpenPointMemory::OpenPointMemory(double merge_radius)
    : merge_radius_(merge_radius) {
  if (!std::isfinite(merge_radius_) || merge_radius_ < 0.0) {
    throw std::invalid_argument("merge_radius must be finite and non-negative");
  }
}

std::vector<OpenPointId> OpenPointMemory::ingest(
    const mars::perception_geometry::PerceptionResult& perception,
    ObservationId observation_id) {
  // Logic ported from online_memory_manager.py::make_open_point and
  // merge_open_point_into_global. Verified against paper Sections 4.3/4.6 and
  // adapted to stable integer IDs and Module 1 types.
  internal::require_finite(perception.neighbor_sight.center,
                           "neighbor_sight.center");
  for (const auto& open_point : perception.open_points) {
    internal::require_finite(open_point.point, "open_point.point");
    if (!std::isfinite(open_point.angle) || open_point.angle < 0.0 ||
        open_point.angle >= mars::common::two_pi) {
      throw std::invalid_argument(
          "open point angle must use Module 1's normalized [0, 2*pi) convention");
    }
    if (open_point.sight_index &&
        *open_point.sight_index >= perception.open_sights.size()) {
      throw std::invalid_argument("open point sight_index is out of range");
    }
  }
  std::vector<OpenPointId> ids;
  ids.reserve(perception.open_points.size());
  for (const auto& open_point : perception.open_points) {
    if (auto* existing = find_merge_candidate(open_point.point)) {
      existing->latest_observation_id = observation_id;
      ids.push_back(existing->id);
      continue;
    }

    const auto& center = perception.neighbor_sight.center;
    records_.push_back({next_id_++,
                        open_point.point,
                        open_point.angle,
                        open_point.sight_index,
                        observation_id,
                        observation_id,
                        center,
                        OpenPointStatus::Active});
    ids.push_back(records_.back().id);
  }
  return ids;
}

bool OpenPointMemory::mark_selected(OpenPointId id) {
  return set_status(id, OpenPointStatus::Selected);
}

bool OpenPointMemory::mark_reached(OpenPointId id) {
  return set_status(id, OpenPointStatus::Reached);
}

bool OpenPointMemory::mark_explored(OpenPointId id) {
  return set_status(id, OpenPointStatus::InactiveExplored);
}

bool OpenPointMemory::reactivate(OpenPointId id) {
  return set_status(id, OpenPointStatus::Active);
}

bool OpenPointMemory::invalidate(OpenPointId id) {
  return set_status(id, OpenPointStatus::Invalid);
}

std::size_t OpenPointMemory::mark_within_radius(
    const mars::common::Point2D& center,
    double radius,
    OpenPointStatus status) {
  // Implements the lifecycle part of
  // online_memory_manager.py::invalidate_open_points_in_explored. Verified
  // against paper Sections 4.3/4.6 and adapted without ROS mission state.
  internal::require_finite(center, "center");
  if (!std::isfinite(radius) || radius < 0.0) {
    throw std::invalid_argument("radius must be finite and non-negative");
  }
  std::size_t changed = 0;
  for (auto& record : records_) {
    if (record.status == OpenPointStatus::Active &&
        internal::point_near(record.point, center, radius)) {
      record.status = status;
      ++changed;
    }
  }
  return changed;
}

std::vector<OpenPointRecord> OpenPointMemory::active_records() const {
  std::vector<OpenPointRecord> active;
  for (const auto& record : records_) {
    if (record.status == OpenPointStatus::Active) {
      active.push_back(record);
    }
  }
  return active;
}

const std::vector<OpenPointRecord>& OpenPointMemory::records() const noexcept {
  return records_;
}

const OpenPointRecord* OpenPointMemory::find(OpenPointId id) const noexcept {
  for (const auto& record : records_) {
    if (record.id == id) {
      return &record;
    }
  }
  return nullptr;
}

OpenPointRecord* OpenPointMemory::find_mutable(OpenPointId id) noexcept {
  for (auto& record : records_) {
    if (record.id == id) {
      return &record;
    }
  }
  return nullptr;
}

OpenPointRecord* OpenPointMemory::find_merge_candidate(
    const mars::common::Point2D& point) noexcept {
  OpenPointRecord* best = nullptr;
  double best_distance = merge_radius_;
  for (auto& record : records_) {
    const double candidate_distance = internal::distance(record.point, point);
    if (candidate_distance <= best_distance) {
      if (best == nullptr || candidate_distance < best_distance ||
          record.id < best->id) {
        best = &record;
        best_distance = candidate_distance;
      }
    }
  }
  return best;
}

bool OpenPointMemory::set_status(OpenPointId id, OpenPointStatus status) {
  if (auto* record = find_mutable(id)) {
    record->status = status;
    return true;
  }
  return false;
}

}  // namespace mars::graph_bundle_management
