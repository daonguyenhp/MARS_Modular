#include <iostream>
#include <vector>

#include "mars/graph_bundle_management/graph_bundle_manager.hpp"
#include "mars/perception_geometry/perception_pipeline.hpp"

int main() {
  namespace mgbm = mars::graph_bundle_management;

  mgbm::GraphBundleUpdate update;
  update.observation_id = 1;
  update.current_pose = {{0.0, 0.0}, 0.0};
  update.goal = {3.0, 0.0};
  const std::vector<mars::common::Polygon2D> obstacles{
      {{{1.0, -0.5}, {1.0, 0.5}}}};
  update.perception = mars::perception_geometry::perceive(
      update.current_pose.position, 2.0, obstacles);

  mgbm::GraphBundleManager manager;
  const auto summary = manager.update(update);
  std::cout << "Observation " << summary.observation_id << " produced "
            << summary.active_open_point_count << " active open point(s).\n";
  return 0;
}
