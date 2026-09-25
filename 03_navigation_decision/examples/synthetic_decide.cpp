#include <iostream>
#include <vector>

#include "navigator.hpp"

int main() {
  using namespace mars::navigation_decision;

  Navigator navigator;
  NavigationObservation observation;
  observation.pose = {0.0, 0.0};
  observation.goal = {3.0, 0.0};
  observation.entry_pose = {0.0, 0.0};
  observation.entry_path = {{0.0, 0.0}, {0.4, 0.2}, {0.8, -0.2}, {1.2, 0.0}};

  RankedOpenPoint open;
  open.id = 1;
  open.point = {1.2, 0.0};
  open.score = 1.0;
  observation.ranked = {open};
  observation.routes = {{1, {{0.0, 0.0}, {1.2, 0.0}}}};

  const auto explore = navigator.decide(observation);
  std::cout << "Explore event=" << explore.event
            << " path_length=" << explore.planned_path.length << "\n";

  observation.pose = {1.2, 0.0};
  observation.ranked.clear();
  observation.routes.clear();
  observation.escape_candidates = {{1, {1.2, 0.0}, {0.0, 0.0}, 1.0}};
  const auto bar = navigator.decide(observation);
  std::cout << "BAR event=" << bar.event << " " << bar.log << "\n";

  Gate first;
  first.left = {0.8, 0.3};
  first.right = {0.8, -0.3};
  first.valid = true;
  observation.gates = {first};
  const auto escape = navigator.decide(observation);
  std::cout << "Escape event=" << escape.event
            << " L_return=" << escape.planned_path.length;
  if (escape.return_leq_entry) {
    std::cout << " leq_entry=" << (*escape.return_leq_entry ? "yes" : "no");
  }
  std::cout << "\n";
  return 0;
}
