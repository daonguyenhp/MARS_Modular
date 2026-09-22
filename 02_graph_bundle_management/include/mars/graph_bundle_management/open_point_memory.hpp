#pragma once

#include <optional>
#include <vector>

#include "mars/graph_bundle_management/types.hpp"

namespace mars::graph_bundle_management {

/** Persistent open-point storage with stable IDs and explicit lifecycle state. */
class OpenPointMemory {
 public:
  /** Construct memory using the given geometric duplicate radius in metres. */
  explicit OpenPointMemory(double merge_radius = 0.05);

  /** Merge all open points from one observation and return IDs in input order. */
  std::vector<OpenPointId> ingest(
      const mars::perception_geometry::PerceptionResult& perception,
      ObservationId observation_id);

  /** Mark a known record as selected. */
  bool mark_selected(OpenPointId id);
  /** Mark a known record as reached. */
  bool mark_reached(OpenPointId id);
  /** Mark a known record as already explored. */
  bool mark_explored(OpenPointId id);
  /** Return a known record to the active candidate set. */
  bool reactivate(OpenPointId id);
  /** Mark a known record as invalid. */
  bool invalidate(OpenPointId id);
  /** Transition active records within radius and return the number changed. */
  std::size_t mark_within_radius(const mars::common::Point2D& center,
                                 double radius,
                                 OpenPointStatus status);

  /** Return copies of records eligible for ranking. */
  std::vector<OpenPointRecord> active_records() const;
  /** Return read-only access to every persistent record. */
  const std::vector<OpenPointRecord>& records() const noexcept;
  /** Find a record by stable ID, or return null. */
  const OpenPointRecord* find(OpenPointId id) const noexcept;

 private:
  OpenPointRecord* find_mutable(OpenPointId id) noexcept;
  OpenPointRecord* find_merge_candidate(const mars::common::Point2D& point) noexcept;
  bool set_status(OpenPointId id, OpenPointStatus status);

  double merge_radius_;
  OpenPointId next_id_{1};
  std::vector<OpenPointRecord> records_{};
};

}  // namespace mars::graph_bundle_management
