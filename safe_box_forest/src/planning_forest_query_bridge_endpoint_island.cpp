#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>

#include <algorithm>
#include <vector>

namespace rbf {

std::vector<int> RBFPlanningForest::endpoint_main_largest_island_partition_first(
    const std::vector<int>& preferred_island) const {
    if (!preferred_island.empty()) {
        return preferred_island;
    }
    if (adaptive_partition_query_enabled_ && adaptive_partition_ && !adaptive_partition_->empty()) {
        return adaptive_partition_->largest_component_box_ids_with_overlay();
    }
    if (partition_native_mode()) {
        return {};
    }
    auto islands = find_islands(adjacency_);
    if (islands.empty()) {
        return {};
    }
    std::sort(islands.begin(), islands.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.size() > rhs.size();
    });
    return islands.front();
}

}  // namespace rbf
