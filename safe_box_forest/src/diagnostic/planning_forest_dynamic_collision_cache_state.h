#pragma once

#include <SBF/box_graph.h>

#include <cstddef>
#include <unordered_map>
#include <vector>

namespace rbf {

struct DynamicCollisionCacheEntry {
    BoxNode box;
    std::vector<int> blocking_obstacle_indices;
    bool active = true;
};

struct DynamicCollisionCacheState {
    std::vector<DynamicCollisionCacheEntry> boxes;
    std::unordered_map<int, std::vector<std::size_t>> blocker_index;
    int active_count = 0;
};

}  // namespace rbf
