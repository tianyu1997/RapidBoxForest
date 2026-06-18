#include <SBF/safe_box_forest.h>

#include <algorithm>
#include <utility>
#include <vector>

namespace rbf {

void RBFPlanningForest::populate_dynamic_collision_cache(const LeafSweepResult& result,
                                                         int obstacle_count) {
    clear_dynamic_collision_cache();
    dynamic_collision_box_cache_.reserve(result.collision_boxes.size());
    std::vector<int> all_obstacles;
    all_obstacles.reserve(static_cast<std::size_t>(std::max(0, obstacle_count)));
    for (int index = 0; index < obstacle_count; ++index) {
        all_obstacles.push_back(index);
    }
    for (std::size_t index = 0; index < result.collision_boxes.size(); ++index) {
        std::vector<int> blockers = all_obstacles;
        if (index < result.collision_box_obstacle_indices.size() &&
            !result.collision_box_obstacle_indices[index].empty()) {
            blockers = result.collision_box_obstacle_indices[index];
        }
        add_dynamic_collision_cache_box(result.collision_boxes[index], std::move(blockers));
    }
}

void RBFPlanningForest::clear_dynamic_collision_cache() {
    dynamic_collision_box_cache_.clear();
    dynamic_collision_cache_blocker_index_.clear();
    dynamic_collision_cache_active_count_ = 0;
}

void RBFPlanningForest::rebuild_dynamic_collision_cache_index() {
    dynamic_collision_cache_blocker_index_.clear();
    dynamic_collision_cache_active_count_ = 0;
    for (std::size_t index = 0; index < dynamic_collision_box_cache_.size(); ++index) {
        const auto& cached = dynamic_collision_box_cache_[index];
        if (!cached.active || cached.blocking_obstacle_indices.empty()) {
            continue;
        }
        dynamic_collision_cache_active_count_ += 1;
        for (int obstacle_index : cached.blocking_obstacle_indices) {
            dynamic_collision_cache_blocker_index_[obstacle_index].push_back(index);
        }
    }
}

void RBFPlanningForest::add_dynamic_collision_cache_box(const BoxNode& box,
                                                        std::vector<int> blocking_obstacle_indices) {
    blocking_obstacle_indices.erase(
        std::remove_if(blocking_obstacle_indices.begin(),
                       blocking_obstacle_indices.end(),
                       [](int index) { return index < 0; }),
        blocking_obstacle_indices.end());
    std::sort(blocking_obstacle_indices.begin(), blocking_obstacle_indices.end());
    blocking_obstacle_indices.erase(
        std::unique(blocking_obstacle_indices.begin(), blocking_obstacle_indices.end()),
        blocking_obstacle_indices.end());
    if (blocking_obstacle_indices.empty()) {
        return;
    }
    CachedCollisionBox cached;
    cached.box = box;
    cached.blocking_obstacle_indices = std::move(blocking_obstacle_indices);
    cached.active = true;
    const std::size_t cache_index = dynamic_collision_box_cache_.size();
    dynamic_collision_box_cache_.push_back(std::move(cached));
    dynamic_collision_cache_active_count_ += 1;
    for (int obstacle_index : dynamic_collision_box_cache_.back().blocking_obstacle_indices) {
        dynamic_collision_cache_blocker_index_[obstacle_index].push_back(cache_index);
    }
}

}  // namespace rbf
