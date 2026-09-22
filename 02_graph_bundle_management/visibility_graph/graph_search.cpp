#include "mars/graph_bundle_management/graph_search.hpp"

#include <algorithm>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <utility>

namespace mars::graph_bundle_management {
namespace {

std::optional<GraphPath> reconstruct(const VisibilityGraph& graph,
                                     VisibilityNodeId start,
                                     VisibilityNodeId goal,
                                     const std::map<VisibilityNodeId,
                                                    VisibilityNodeId>& previous) {
  std::vector<VisibilityNodeId> reversed{goal};
  while (reversed.back() != start) {
    const auto iterator = previous.find(reversed.back());
    if (iterator == previous.end()) {
      return std::nullopt;
    }
    reversed.push_back(iterator->second);
  }
  std::reverse(reversed.begin(), reversed.end());
  double cost = 0.0;
  for (std::size_t index = 1; index < reversed.size(); ++index) {
    const auto* edge = graph.find_edge(reversed[index - 1], reversed[index]);
    if (edge == nullptr) {
      return std::nullopt;
    }
    cost += edge->cost;
  }
  return GraphPath{std::move(reversed), cost};
}

}  // namespace

std::optional<GraphPath> breadth_first_path(const VisibilityGraph& graph,
                                            VisibilityNodeId start,
                                            VisibilityNodeId goal) {
  if (graph.find_node(start) == nullptr || graph.find_node(goal) == nullptr) {
    return std::nullopt;
  }
  if (start == goal) {
    return GraphPath{{start}, 0.0};
  }
  std::queue<VisibilityNodeId> queue;
  std::set<VisibilityNodeId> visited{start};
  std::map<VisibilityNodeId, VisibilityNodeId> previous;
  queue.push(start);
  while (!queue.empty()) {
    const auto current = queue.front();
    queue.pop();
    for (const auto neighbor : graph.neighbors(current)) {
      if (!visited.insert(neighbor).second) {
        continue;
      }
      previous[neighbor] = current;
      if (neighbor == goal) {
        return reconstruct(graph, start, goal, previous);
      }
      queue.push(neighbor);
    }
  }
  return std::nullopt;
}

std::optional<GraphPath> shortest_cost_path(const VisibilityGraph& graph,
                                            VisibilityNodeId start,
                                            VisibilityNodeId goal) {
  // Logic ported from dsfm_polygon_node.py::_bfs_skeleton_path.
  // Verified against paper Section 4.5. The legacy implementation is
  // cost-based, so this API accurately names Dijkstra.
  if (graph.find_node(start) == nullptr || graph.find_node(goal) == nullptr) {
    return std::nullopt;
  }
  using QueueItem = std::pair<double, VisibilityNodeId>;
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;
  std::map<VisibilityNodeId, double> distances;
  std::map<VisibilityNodeId, VisibilityNodeId> previous;
  distances[start] = 0.0;
  queue.push({0.0, start});

  while (!queue.empty()) {
    const auto [distance, current] = queue.top();
    queue.pop();
    if (distance > distances[current]) {
      continue;
    }
    if (current == goal) {
      return reconstruct(graph, start, goal, previous);
    }
    for (const auto neighbor : graph.neighbors(current)) {
      const auto* edge = graph.find_edge(current, neighbor);
      const double candidate = distance + edge->cost;
      const auto known = distances.find(neighbor);
      if (known == distances.end() || candidate < known->second ||
          (candidate == known->second && current < previous[neighbor])) {
        distances[neighbor] = candidate;
        previous[neighbor] = current;
        queue.push({candidate, neighbor});
      }
    }
  }
  return std::nullopt;
}

}  // namespace mars::graph_bundle_management
