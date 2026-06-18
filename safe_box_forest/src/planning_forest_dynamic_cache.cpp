#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <unordered_set>
#include <vector>

namespace rbf {

namespace {

const BoxNode* find_box_by_id_local(const std::vector<BoxNode>& boxes, int box_id) {
    for (const auto& box : boxes) {
        if (box.id == box_id) {
            return &box;
        }
    }
    return nullptr;
}

bool box_contains_box_exact_local(const BoxNode& outer, const BoxNode& inner) {
    if (outer.n_dims() != inner.n_dims()) {
        return false;
    }
    for (int dim = 0; dim < outer.n_dims(); ++dim) {
        if (outer.joint_intervals[dim].lo > inner.joint_intervals[dim].lo ||
            outer.joint_intervals[dim].hi < inner.joint_intervals[dim].hi) {
            return false;
        }
    }
    return outer.id != inner.id;
}

bool has_adjacency_to_existing_box(const std::vector<BoxNode>& boxes,
                                   const BoxNode& box,
                                   double tolerance,
                                   int* parent_box_id) {
    for (const auto& existing : boxes) {
        if (boxes_connected(existing, box, tolerance)) {
            if (parent_box_id != nullptr) {
                *parent_box_id = existing.id;
            }
            return true;
        }
    }
    return false;
}

} // namespace

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

int RBFPlanningForest::promote_unblocked_collision_cache(const std::unordered_set<int>& removed_obstacle_indices,
                                                         RebuildProfile& profile) {
    if (dynamic_collision_cache_active_count_ <= 0 && !dynamic_collision_box_cache_.empty()) {
        rebuild_dynamic_collision_cache_index();
    }
    profile.collision_cache_boxes_before = dynamic_collision_cache_active_count_;
    if (removed_obstacle_indices.empty() || dynamic_collision_cache_active_count_ <= 0) {
        profile.collision_cache_boxes_after = dynamic_collision_cache_active_count_;
        return 0;
    }

    std::vector<int> sorted_removed(removed_obstacle_indices.begin(), removed_obstacle_indices.end());
    std::sort(sorted_removed.begin(), sorted_removed.end());
    sorted_removed.erase(std::unique(sorted_removed.begin(), sorted_removed.end()), sorted_removed.end());
    const int min_removed = sorted_removed.empty() ? std::numeric_limits<int>::max() : sorted_removed.front();
    const int max_removed = sorted_removed.empty() ? std::numeric_limits<int>::min() : sorted_removed.back();
    auto is_removed_index = [&](int old_index) {
        return std::binary_search(sorted_removed.begin(), sorted_removed.end(), old_index);
    };
    auto removed_before_count = [&](int old_index) {
        return static_cast<int>(
            std::lower_bound(sorted_removed.begin(), sorted_removed.end(), old_index) - sorted_removed.begin());
    };
    auto remap_obstacle_index = [&](int old_index, int& new_index) {
        if (is_removed_index(old_index)) {
            return false;
        }
        const int shift = removed_before_count(old_index);
        new_index = old_index - shift;
        return new_index >= 0 && new_index < scene_.n_obstacles();
    };

    int promoted = 0;
    int next_id = next_box_id();
    const double adjacency_tolerance = config_.query.adjacency_tolerance;
    const auto cache_scan_t0 = std::chrono::steady_clock::now();

    auto deactivate_cached = [&](CachedCollisionBox& cached) {
        if (cached.active) {
            cached.active = false;
            cached.blocking_obstacle_indices.clear();
            dynamic_collision_cache_active_count_ =
                std::max(0, dynamic_collision_cache_active_count_ - 1);
        }
    };

    auto try_promote_touched_cached = [&](CachedCollisionBox& cached) {
        bool touched = false;
        std::vector<int> remaining_blockers;
        remaining_blockers.reserve(cached.blocking_obstacle_indices.size());
        for (int old_index : cached.blocking_obstacle_indices) {
            profile.diagnostics["delete.blocker_index_checks"] += 1.0;
            if (is_removed_index(old_index)) {
                touched = true;
                continue;
            }
            int new_index = -1;
            if (remap_obstacle_index(old_index, new_index)) {
                remaining_blockers.push_back(new_index);
            }
        }
        std::sort(remaining_blockers.begin(), remaining_blockers.end());
        remaining_blockers.erase(std::unique(remaining_blockers.begin(), remaining_blockers.end()),
                                 remaining_blockers.end());
        if (!touched) {
            cached.blocking_obstacle_indices = std::move(remaining_blockers);
            return;
        }
        profile.collision_cache_candidates += 1;
        if (!remaining_blockers.empty()) {
            cached.blocking_obstacle_indices = std::move(remaining_blockers);
            return;
        }

        BoxNode box = cached.box;
        box.id = next_id;
        bool contained = false;
        const auto contained_t0 = std::chrono::steady_clock::now();
        for (const auto& existing : boxes_) {
            profile.diagnostics["delete.containment_checks"] += 1.0;
            if (box_contains_box_exact_local(existing, box)) {
                contained = true;
                break;
            }
        }
        profile.diagnostics["delete.containment_check_ms"] +=
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - contained_t0).count();
        if (contained) {
            profile.collision_cache_rejected_contained += 1;
            deactivate_cached(cached);
            return;
        }
        int adjacent_parent = -1;
        const auto adjacency_t0 = std::chrono::steady_clock::now();
        if (!boxes_.empty() &&
            !has_adjacency_to_existing_box(boxes_, box, adjacency_tolerance, &adjacent_parent)) {
            profile.diagnostics["delete.adjacency_check_ms"] +=
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - adjacency_t0).count();
            profile.collision_cache_rejected_disconnected += 1;
            deactivate_cached(cached);
            return;
        }
        profile.diagnostics["delete.adjacency_check_ms"] +=
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - adjacency_t0).count();
        profile.diagnostics["delete.adjacency_checks"] += 1.0;
        box.parent_box_id = adjacent_parent;
        const BoxNode* parent = find_box_by_id_local(boxes_, adjacent_parent);
        box.root_id = parent != nullptr && parent->root_id >= 0 ? parent->root_id : box.id;
        box.safety_status = BoxSafetyStatus::CertifiedFree;
        box.strict_audit_required = false;
        box.compute_volume();
        if (oracle_ && box.tree_id >= 0) {
            oracle_->reserve_node(box.tree_id, box.id);
        }
        boxes_.push_back(box);
        raw_boxes_.push_back(box);
        next_id += 1;
        promoted += 1;
        profile.collision_cache_promoted += 1;
        profile.boxes_added += 1;
        profile.raw_boxes_added += 1;
        deactivate_cached(cached);
    };

    const bool suffix_remove = min_removed >= scene_.n_obstacles();
    if (suffix_remove && !dynamic_collision_cache_blocker_index_.empty()) {
        std::vector<std::size_t> candidate_indices;
        std::unordered_set<std::size_t> seen;
        for (int removed_index : sorted_removed) {
            const auto it = dynamic_collision_cache_blocker_index_.find(removed_index);
            if (it == dynamic_collision_cache_blocker_index_.end()) {
                continue;
            }
            for (std::size_t index : it->second) {
                if (index < dynamic_collision_box_cache_.size() && seen.insert(index).second) {
                    candidate_indices.push_back(index);
                }
            }
        }
        profile.diagnostics["delete.cache_indexed_suffix_path"] = 1.0;
        profile.diagnostics["delete.cache_index_removed_keys"] = static_cast<double>(sorted_removed.size());
        profile.diagnostics["delete.cache_index_candidate_entries"] =
            static_cast<double>(candidate_indices.size());
        for (std::size_t index : candidate_indices) {
            if (index >= dynamic_collision_box_cache_.size()) {
                profile.diagnostics["delete.cache_index_stale_out_of_range"] += 1.0;
                continue;
            }
            CachedCollisionBox& cached = dynamic_collision_box_cache_[index];
            if (!cached.active) {
                profile.diagnostics["delete.cache_index_stale_inactive"] += 1.0;
                continue;
            }
            profile.diagnostics["delete.cache_entries_scanned"] += 1.0;
            try_promote_touched_cached(cached);
        }
        profile.diagnostics["delete.cache_scan_ms"] +=
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - cache_scan_t0).count();
        profile.collision_cache_boxes_after = dynamic_collision_cache_active_count_;
        return promoted;
    }

    profile.diagnostics["delete.cache_indexed_suffix_path"] = 0.0;
    std::size_t write_index = 0;
    auto keep_cached_at = [&](std::size_t read_index) {
        if (write_index != read_index) {
            dynamic_collision_box_cache_[write_index] =
                std::move(dynamic_collision_box_cache_[read_index]);
        }
        ++write_index;
    };
    for (std::size_t read_index = 0; read_index < dynamic_collision_box_cache_.size(); ++read_index) {
        auto& cached = dynamic_collision_box_cache_[read_index];
        if (!cached.active) {
            continue;
        }
        profile.diagnostics["delete.cache_entries_scanned"] += 1.0;
        if (!cached.blocking_obstacle_indices.empty()) {
            const int first_blocker = cached.blocking_obstacle_indices.front();
            const int last_blocker = cached.blocking_obstacle_indices.back();
            if (last_blocker < min_removed) {
                // Common suffix-delete case: this cached box is blocked only by
                // obstacles that remain before the deleted suffix, so neither
                // blocker membership nor obstacle numbering changes.
                profile.diagnostics["delete.cache_fast_untouched_before_removed"] += 1.0;
                keep_cached_at(read_index);
                continue;
            }
            if (first_blocker > max_removed) {
                // No blocker is removed, but all blocker ids shift down by the
                // number of removed obstacles before them.
                profile.diagnostics["delete.cache_fast_remap_after_removed"] += 1.0;
                std::vector<int> remapped;
                remapped.reserve(cached.blocking_obstacle_indices.size());
                for (int old_index : cached.blocking_obstacle_indices) {
                    int new_index = -1;
                    if (remap_obstacle_index(old_index, new_index)) {
                        remapped.push_back(new_index);
                    }
                }
                std::sort(remapped.begin(), remapped.end());
                remapped.erase(std::unique(remapped.begin(), remapped.end()), remapped.end());
                cached.blocking_obstacle_indices = std::move(remapped);
                if (!cached.blocking_obstacle_indices.empty()) {
                    keep_cached_at(read_index);
                }
                continue;
            }
        }
        bool touched = false;
        std::vector<int> remaining_blockers;
        remaining_blockers.reserve(cached.blocking_obstacle_indices.size());
        for (int old_index : cached.blocking_obstacle_indices) {
            profile.diagnostics["delete.blocker_index_checks"] += 1.0;
            if (is_removed_index(old_index)) {
                touched = true;
                continue;
            }
            int new_index = -1;
            if (remap_obstacle_index(old_index, new_index)) {
                remaining_blockers.push_back(new_index);
            }
        }
        std::sort(remaining_blockers.begin(), remaining_blockers.end());
        remaining_blockers.erase(std::unique(remaining_blockers.begin(), remaining_blockers.end()),
                                 remaining_blockers.end());

        if (!touched) {
            cached.blocking_obstacle_indices = std::move(remaining_blockers);
            keep_cached_at(read_index);
            continue;
        }
        profile.collision_cache_candidates += 1;
        if (!remaining_blockers.empty()) {
            cached.blocking_obstacle_indices = std::move(remaining_blockers);
            keep_cached_at(read_index);
            continue;
        }

        BoxNode box = cached.box;
        box.id = next_id;
        bool contained = false;
        const auto contained_t0 = std::chrono::steady_clock::now();
        for (const auto& existing : boxes_) {
            profile.diagnostics["delete.containment_checks"] += 1.0;
            if (box_contains_box_exact_local(existing, box)) {
                contained = true;
                break;
            }
        }
        profile.diagnostics["delete.containment_check_ms"] +=
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - contained_t0).count();
        if (contained) {
            profile.collision_cache_rejected_contained += 1;
            continue;
        }
        int adjacent_parent = -1;
        const auto adjacency_t0 = std::chrono::steady_clock::now();
        if (!boxes_.empty() &&
            !has_adjacency_to_existing_box(boxes_, box, adjacency_tolerance, &adjacent_parent)) {
            profile.diagnostics["delete.adjacency_check_ms"] +=
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - adjacency_t0).count();
            profile.collision_cache_rejected_disconnected += 1;
            cached.blocking_obstacle_indices.clear();
            keep_cached_at(read_index);
            continue;
        }
        profile.diagnostics["delete.adjacency_check_ms"] +=
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - adjacency_t0).count();
        profile.diagnostics["delete.adjacency_checks"] += 1.0;
        box.parent_box_id = adjacent_parent;
        const BoxNode* parent = find_box_by_id_local(boxes_, adjacent_parent);
        box.root_id = parent != nullptr && parent->root_id >= 0 ? parent->root_id : box.id;
        box.safety_status = BoxSafetyStatus::CertifiedFree;
        box.strict_audit_required = false;
        box.compute_volume();
        if (oracle_ && box.tree_id >= 0) {
            oracle_->reserve_node(box.tree_id, box.id);
        }
        boxes_.push_back(box);
        raw_boxes_.push_back(box);
        next_id += 1;
        promoted += 1;
        profile.collision_cache_promoted += 1;
        profile.boxes_added += 1;
        profile.raw_boxes_added += 1;
    }
    profile.diagnostics["delete.cache_scan_ms"] +=
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - cache_scan_t0).count();
    dynamic_collision_box_cache_.resize(write_index);
    rebuild_dynamic_collision_cache_index();
    profile.collision_cache_boxes_after = dynamic_collision_cache_active_count_;
    return promoted;
}

} // namespace rbf
