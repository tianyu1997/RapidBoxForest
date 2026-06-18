#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>
#include <SBF/connector.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <unordered_set>
#include <vector>

#include "env_config.h"
#include "planning_forest_audit.h"

namespace rbf {

namespace {

using detail::env_int_or_default;

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

bool intervals_subset_local(const std::vector<Interval>& inner,
                            const std::vector<Interval>& outer,
                            double tolerance = 0.0) {
    if (inner.size() != outer.size()) {
        return false;
    }
    for (std::size_t dim = 0; dim < inner.size(); ++dim) {
        if (inner[dim].lo < outer[dim].lo - tolerance ||
            inner[dim].hi > outer[dim].hi + tolerance) {
            return false;
        }
    }
    return true;
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

void append_local_edge(AdjacencyGraph& graph, int lhs, int rhs) {
    if (lhs < 0 || rhs < 0 || lhs == rhs) {
        return;
    }
    auto append_one = [](std::vector<int>& neighbors, int value) {
        if (std::find(neighbors.begin(), neighbors.end(), value) == neighbors.end()) {
            neighbors.push_back(value);
        }
    };
    append_one(graph[lhs], rhs);
    append_one(graph[rhs], lhs);
}

void connect_incremental_boxes(AdjacencyGraph& graph,
                               const std::vector<BoxNode>& boxes,
                               std::size_t first_new_index,
                               double tolerance) {
    if (first_new_index >= boxes.size()) {
        return;
    }
    for (const auto& box : boxes) {
        graph[box.id];
    }
    for (std::size_t i = first_new_index; i < boxes.size(); ++i) {
        for (std::size_t j = 0; j < i; ++j) {
            if (boxes_connected(boxes[j], boxes[i], tolerance)) {
                append_local_edge(graph, boxes[j].id, boxes[i].id);
            }
        }
    }
}

bool allow_dynamic_commit(BoxOracle& oracle,
                          FindFreeBoxResult& result,
                          BoxCommitPolicy policy) {
    if (result.validation_detail.safety_status == BoxSafetyStatus::CertifiedFree) {
        return true;
    }
    if (result.validation_detail.safety_status != BoxSafetyStatus::ProvisionalFree) {
        return false;
    }
    if (policy == BoxCommitPolicy::CommitCertifiedOnly) {
        return false;
    }
    if (policy == BoxCommitPolicy::CommitProvisionalAllowed) {
        return true;
    }
    if (policy == BoxCommitPolicy::AuditBeforeCommit) {
        if (oracle.validate_intervals(result.intervals)) {
            result.validation_detail.safety_status = BoxSafetyStatus::CertifiedFree;
            result.validation_detail.strict_audit_required = false;
            return true;
        }
        return false;
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

int RBFPlanningForest::refill_removed_box_with_leaf_sweep(const BoxNode& removed_box,
                                                          int new_obstacle_index,
                                                          int max_depth,
                                                          int& next_id,
                                                          RebuildProfile& profile) {
    if (!oracle_ || removed_box.tree_id < 0 || removed_box.joint_intervals.empty()) {
        return 0;
    }
    const int effective_max_depth = std::max(0, std::min(max_depth, oracle_->max_tree_depth() - 1));
    struct Item {
        OracleNodeId node = kInvalidOracleNodeId;
        int changed_dim = -1;
        std::vector<Interval> intervals;
    };
    std::vector<Item> stack;
    stack.push_back(Item{removed_box.tree_id, -1, removed_box.joint_intervals});
    int added = 0;
    const OracleSplitOptions split_options = config_.grower.find_free_box.split;
    const int max_stack_pops = std::max(64, 4 * (1 << std::min(effective_max_depth + 1, 12)));
    int stack_pops = 0;

    auto cache_collision_leaf = [&](OracleNodeId node, const std::vector<Interval>& intervals) {
        profile.diagnostics["insert.refill_cached_collision_leaves"] += 1.0;
        BoxNode cached_box;
        cached_box.id = -1;
        cached_box.joint_intervals = intervals;
        cached_box.seed_config = cached_box.center();
        cached_box.tree_id = node;
        cached_box.parent_box_id = removed_box.parent_box_id;
        cached_box.root_id = removed_box.root_id;
        cached_box.safety_status = BoxSafetyStatus::Unknown;
        cached_box.strict_audit_required = true;
        cached_box.compute_volume();
        add_dynamic_collision_cache_box(cached_box, {new_obstacle_index});
    };

    while (!stack.empty()) {
        profile.diagnostics["insert.refill_stack_pops"] += 1.0;
        if (config_.dynamic_update.local_regrow_box_limit > 0 &&
            profile.boxes_added >= config_.dynamic_update.local_regrow_box_limit) {
            profile.diagnostics["insert.refill_local_regrow_box_cap_hits"] += 1.0;
            break;
        }
        if (++stack_pops > max_stack_pops) {
            profile.diagnostics["insert.refill_stack_pop_cap_hits"] += 1.0;
            break;
        }
        const Item item = stack.back();
        stack.pop_back();
        if (item.node < 0) {
            continue;
        }
        const std::vector<Interval> tree_intervals = oracle_->node_intervals(item.node);
        std::vector<Interval> intervals = item.intervals;
        if (intervals.empty()) {
            bool found_matching_native_copy = false;
            for (auto candidate : oracle_->native_interval_copies_for_node(item.node, tree_intervals)) {
                if (intervals_subset_local(candidate, removed_box.joint_intervals, 1e-12)) {
                    intervals = std::move(candidate);
                    found_matching_native_copy = true;
                    break;
                }
            }
            if (!found_matching_native_copy) {
                profile.diagnostics["insert.refill_native_copy_misses"] += 1.0;
                continue;
            }
        } else if (!intervals_subset_local(intervals, removed_box.joint_intervals, 1e-10)) {
            profile.diagnostics["insert.refill_interval_subset_rejects"] += 1.0;
            continue;
        }
        if (oracle_->is_reserved(item.node)) {
            profile.diagnostics["insert.refill_reserved_skips"] += 1.0;
            continue;
        }
        profile.regrow_attempts += 1;
        const auto validate_t0 = std::chrono::steady_clock::now();
        const BoxValidation validation = oracle_->validate_node(item.node, intervals, item.changed_dim);
        profile.diagnostics["insert.refill_validate_node_calls"] += 1.0;
        profile.diagnostics["insert.refill_validate_node_ms"] +=
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - validate_t0).count();
        OracleValidationDetail detail = oracle_->last_validation_detail();
        if (validation == BoxValidation::Free) {
            profile.diagnostics["insert.refill_free_leaves"] += 1.0;
            BoxNode box;
            box.id = next_id++;
            box.joint_intervals = std::move(intervals);
            box.seed_config = box.center();
            box.tree_id = item.node;
            box.parent_box_id = removed_box.parent_box_id;
            box.root_id = removed_box.root_id >= 0 ? removed_box.root_id : box.id;
            box.safety_status = detail.safety_status;
            box.strict_audit_required = detail.strict_audit_required;
            box.compute_volume();

            FindFreeBoxResult commit_probe;
            commit_probe.found = true;
            commit_probe.node = item.node;
            commit_probe.intervals = box.joint_intervals;
            commit_probe.validation_detail = detail;
            if (!allow_dynamic_commit(*oracle_, commit_probe, config_.grower.commit_policy)) {
                profile.diagnostics["insert.refill_commit_rejects"] += 1.0;
                continue;
            }
            bool contained = false;
            const auto contained_t0 = std::chrono::steady_clock::now();
            for (const auto& existing : boxes_) {
                profile.diagnostics["insert.refill_containment_checks"] += 1.0;
                if (box_contains_box_exact_local(existing, box)) {
                    contained = true;
                    break;
                }
            }
            profile.diagnostics["insert.refill_containment_check_ms"] +=
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - contained_t0).count();
            if (contained) {
                profile.collision_cache_rejected_contained += 1;
                profile.diagnostics["insert.refill_contained_rejects"] += 1.0;
                continue;
            }
            oracle_->reserve_node(box.tree_id, box.id);
            boxes_.push_back(box);
            raw_boxes_.push_back(std::move(box));
            profile.boxes_added += 1;
            profile.raw_boxes_added += 1;
            added += 1;
            continue;
        }

        if (oracle_->depth(item.node) >= effective_max_depth) {
            profile.diagnostics["insert.refill_depth_cap_hits"] += 1.0;
            cache_collision_leaf(item.node, intervals);
            continue;
        }
        const auto split_t0 = std::chrono::steady_clock::now();
        const auto split = oracle_->split_node(item.node, tree_intervals, item.changed_dim, split_options);
        profile.diagnostics["insert.refill_split_node_calls"] += 1.0;
        profile.diagnostics["insert.refill_split_node_ms"] +=
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - split_t0).count();
        if (!split.split) {
            profile.diagnostics["insert.refill_split_failures"] += 1.0;
            cache_collision_leaf(item.node, intervals);
            continue;
        }
        profile.diagnostics["insert.refill_split_success"] += 1.0;
        if (split.split_dim < 0 || split.split_dim >= static_cast<int>(intervals.size())) {
            profile.diagnostics["insert.refill_split_bad_dim"] += 1.0;
            cache_collision_leaf(item.node, intervals);
            continue;
        }
        const int dim = split.split_dim;
        const double lo = intervals[static_cast<std::size_t>(dim)].lo;
        const double hi = intervals[static_cast<std::size_t>(dim)].hi;
        double value = split.split_value;
        if (!(value > lo + 1e-14 && value < hi - 1e-14)) {
            value = 0.5 * (lo + hi);
            profile.diagnostics["insert.refill_split_native_midpoint_fallbacks"] += 1.0;
        }
        if (!(value > lo && value < hi)) {
            profile.diagnostics["insert.refill_split_degenerate_intervals"] += 1.0;
            cache_collision_leaf(item.node, intervals);
            continue;
        }
        std::vector<Interval> left_intervals = intervals;
        std::vector<Interval> right_intervals = intervals;
        left_intervals[static_cast<std::size_t>(dim)].hi = value;
        right_intervals[static_cast<std::size_t>(dim)].lo = value;
        stack.push_back(Item{split.right, split.split_dim, std::move(right_intervals)});
        stack.push_back(Item{split.left, split.split_dim, std::move(left_intervals)});
    }
    return added;
}

RebuildProfile RBFPlanningForest::connect_update_segment_fallback() {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    RebuildProfile profile;
    profile.boxes_before = static_cast<int>(boxes_.size());
    profile.raw_boxes_before = static_cast<int>(raw_boxes_.size());
    profile.obstacles_before = scene_.n_obstacles();
    profile.obstacles_after = profile.obstacles_before;
    profile.collision_cache_boxes_before = static_cast<int>(dynamic_collision_box_cache_.size());
    profile.collision_cache_boxes_after = profile.collision_cache_boxes_before;
    profile.diagnostics["segment_fallback.segment_edges_before"] = static_cast<double>(segment_edges_.size());
    const bool use_partition_backend =
        partition_native_mode() && adaptive_partition_query_enabled_ && adaptive_partition_;
    const int islands_before = use_partition_backend
        ? adaptive_partition_->component_count_with_overlay()
        : static_cast<int>(find_islands(adjacency_).size());
    profile.diagnostics["segment_fallback.islands_before"] = static_cast<double>(islands_before);

    if (!oracle_ || boxes_.empty()) {
        profile.boxes_after = profile.boxes_before;
        profile.raw_boxes_after = profile.raw_boxes_before;
        profile.adjacency_islands = islands_before;
        profile.fallback_reason = boxes_.empty() ? "empty_forest" : "missing_oracle";
        profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        return profile;
    }

    if (use_partition_backend) {
        const auto connector_t0 = Clock::now();
        if (islands_before <= 1) {
            profile.boxes_after = profile.boxes_before;
            profile.raw_boxes_after = profile.raw_boxes_before;
            profile.adjacency_islands = islands_before;
            profile.diagnostics["segment_fallback.partition_native"] = 1.0;
            profile.diagnostics["segment_fallback.connected"] = 1.0;
            profile.diagnostics["segment_fallback.segment_edges_after"] =
                static_cast<double>(segment_edges_.size());
            profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
            return profile;
        }
        CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
        int attempted_pairs = 0;
        int audit_fail = 0;
        int added = 0;
        const int pair_candidate_cap = std::max(
            8,
            env_int_or_default("RBF_PARTITION_SEGMENT_FALLBACK_PAIR_CANDIDATE_CAP", 128));
        const auto candidate_pairs =
            adaptive_partition_->nearest_component_pairs_to_largest(1, pair_candidate_cap);
        for (const auto& pair : candidate_pairs) {
            if (pair.source_box_id < 0 || pair.target_box_id < 0 ||
                pair.source_point.size() == 0 || pair.target_point.size() == 0) {
                continue;
            }
            ++attempted_pairs;
            std::vector<Eigen::VectorXd> waypoints{pair.source_point, pair.target_point};
            if (!audit_waypoint_path_passes(waypoints,
                                            checker,
                                            config_.query.audit_resolution,
                                            config_.query.audit_segment_step)) {
                ++audit_fail;
                continue;
            }
            const int edge_id = add_segment_edge_partition_first(pair.source_box_id,
                                                                 pair.target_box_id,
                                                                 std::move(waypoints),
                                                                 SegmentEdgeType::QueryBridge,
                                                                 config_.query.audit_resolution,
                                                                 SegmentEdgeValidation::CollisionChecked,
                                                                 true,
                                                                 -1,
                                                                 nullptr,
                                                                 "segment_fallback.partition_native");
            if (edge_id >= 0) {
                ++added;
            }
        }
        profile.regrow_ms = std::chrono::duration<double, std::milli>(Clock::now() - connector_t0).count();
        profile.segment_edges_added = added;
        profile.rrt_segment_edges_added = added;
        profile.point_gap_segment_edges_added = 0;
        profile.boxes_added = 0;
        profile.raw_boxes_added = 0;
        profile.boxes_after = static_cast<int>(boxes_.size());
        profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
        sync_adaptive_partition_segment_edges(nullptr, "segment_fallback.partition_native");
        profile.adjacency_islands = adaptive_partition_->component_count_with_overlay();
        profile.diagnostics["segment_fallback.partition_native"] = 1.0;
        profile.diagnostics["segment_fallback.attempted_pairs"] = static_cast<double>(attempted_pairs);
        profile.diagnostics["segment_fallback.audit_fail"] = static_cast<double>(audit_fail);
        profile.diagnostics["segment_fallback.partition_pair_candidates"] =
            static_cast<double>(candidate_pairs.size());
        profile.diagnostics["segment_fallback.connected"] = profile.adjacency_islands <= 1 ? 1.0 : 0.0;
        profile.diagnostics["segment_fallback.segment_edges_after"] =
            static_cast<double>(segment_edges_.size());
        profile.diagnostics["segment_fallback.islands_after"] =
            static_cast<double>(profile.adjacency_islands);
        profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        const auto& partition_stats = adaptive_partition_->stats();
        profile.diagnostics["adaptive.partition_cells"] = static_cast<double>(partition_stats.cells);
        profile.diagnostics["adaptive.partition_islands"] = static_cast<double>(partition_stats.islands);
        profile.diagnostics["adaptive.partition_overlay_edges"] =
            static_cast<double>(partition_stats.overlay_edges);
        return profile;
    }

    const auto connector_t0 = Clock::now();
    StageContext context = StageContext::from_runtime(config_.runtime);
    CollisionChecker checker(robot_, scene_);
    IslandConnectorConfig connector_config = config_.connector;
    connector_config.segment_edges_enabled = true;
    connector_config.rrt_segment_edges = true;
    connector_config.point_gap_segment_edges = true;
    if (connector_config.n_threads <= 1 && config_.runtime.n_threads > 1) {
        connector_config.n_threads = config_.runtime.n_threads;
    }
    IslandConnector connector(*oracle_, robot_, checker, connector_config);
    int connector_next_id = next_box_id();
    const auto connector_result =
        connector.connect_all(boxes_, adjacency_, segment_edges_, connector_next_id, context);
    profile.regrow_ms = std::chrono::duration<double, std::milli>(Clock::now() - connector_t0).count();

    profile.bridge_boxes_added = connector_result.bridge_boxes_added;
    profile.segment_edges_added = connector_result.segment_edges_added;
    profile.rrt_segment_edges_added = connector_result.rrt_segment_edges_added;
    profile.point_gap_segment_edges_added = connector_result.point_gap_segment_edges_added;
    profile.boxes_added = std::max(0, static_cast<int>(boxes_.size()) - profile.boxes_before);
    profile.raw_boxes_added = std::max(0, static_cast<int>(raw_boxes_.size()) - profile.raw_boxes_before);
    profile.diagnostics["segment_fallback.attempted_pairs"] = static_cast<double>(connector_result.attempted_pairs);
    profile.diagnostics["segment_fallback.connected"] = connector_result.connected ? 1.0 : 0.0;
    profile.diagnostics["segment_fallback.segment_edges_after"] = static_cast<double>(segment_edges_.size());

    const auto adj_t0 = Clock::now();
    apply_segment_edges_to_adjacency(segment_edges_, adjacency_);
    invalidate_query_cache();
    profile.adjacency_ms = std::chrono::duration<double, std::milli>(Clock::now() - adj_t0).count();
    profile.boxes_after = static_cast<int>(boxes_.size());
    profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
    profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
    profile.diagnostics["segment_fallback.islands_after"] = static_cast<double>(profile.adjacency_islands);
    profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    return profile;
}

RebuildProfile RBFPlanningForest::remove_obstacle_and_regrow(int obstacle_index) {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    RebuildProfile profile;
    profile.boxes_before = static_cast<int>(boxes_.size());
    profile.raw_boxes_before = static_cast<int>(raw_boxes_.size());
    profile.obstacles_before = scene_.n_obstacles();
    profile.removed_obstacle_index = obstacle_index;

    Obstacle removed_obstacle;
    if (!scene_.remove_obstacle_at(obstacle_index, &removed_obstacle)) {
        profile.boxes_after = profile.boxes_before;
        profile.raw_boxes_after = profile.raw_boxes_before;
        profile.obstacles_after = profile.obstacles_before;
        profile.adjacency_islands = island_count_partition_first();
        profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        return profile;
    }
    profile.obstacles_after = scene_.n_obstacles();
    reset_oracle(scene_);
    reserve_existing_boxes();
    const std::unordered_set<int> removed_indices{obstacle_index};
    const std::size_t first_new_box_index = boxes_.size();
    promote_unblocked_collision_cache(removed_indices, profile);

    profile.boxes_after = static_cast<int>(boxes_.size());
    profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
    const auto adj_t0_delete = Clock::now();
    if (partition_native_mode()) {
        refresh_dynamic_partition_after_append(profile,
                                               first_new_box_index,
                                               "remove.partition_append");
    } else {
        if (profile.boxes_added > 0) {
            connect_incremental_boxes(adjacency_, boxes_, first_new_box_index, config_.query.adjacency_tolerance);
            apply_segment_edges_to_adjacency(segment_edges_, adjacency_);
            invalidate_query_cache();
        }
        profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
    }
    profile.adjacency_ms = std::chrono::duration<double, std::milli>(Clock::now() - adj_t0_delete).count();
    profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    invalidate_query_cache();
    return profile;
}

RebuildProfile RBFPlanningForest::remove_obstacle_suffix_and_regrow(int target_obstacle_count) {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    RebuildProfile profile;
    profile.boxes_before = static_cast<int>(boxes_.size());
    profile.raw_boxes_before = static_cast<int>(raw_boxes_.size());
    profile.obstacles_before = scene_.n_obstacles();
    const int target_count = std::clamp(target_obstacle_count, 0, profile.obstacles_before);
    profile.removed_obstacle_index = target_count;

    if (target_count >= profile.obstacles_before) {
        profile.boxes_after = profile.boxes_before;
        profile.raw_boxes_after = profile.raw_boxes_before;
        profile.obstacles_after = profile.obstacles_before;
        profile.adjacency_islands = island_count_partition_first();
        profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        return profile;
    }

    std::vector<Obstacle> removed_obstacles;
    removed_obstacles.reserve(static_cast<std::size_t>(profile.obstacles_before - target_count));
    while (scene_.n_obstacles() > target_count) {
        Obstacle removed_obstacle;
        scene_.remove_obstacle_at(scene_.n_obstacles() - 1, &removed_obstacle);
        removed_obstacles.push_back(removed_obstacle);
    }
    profile.obstacles_after = scene_.n_obstacles();
    reset_oracle(scene_);
    reserve_existing_boxes();
    std::unordered_set<int> removed_indices;
    for (int index = target_count; index < profile.obstacles_before; ++index) {
        removed_indices.insert(index);
    }
    const std::size_t first_new_box_index = boxes_.size();
    promote_unblocked_collision_cache(removed_indices, profile);

    profile.boxes_after = static_cast<int>(boxes_.size());
    profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
    const auto adj_t0_delete = Clock::now();
    if (partition_native_mode()) {
        refresh_dynamic_partition_after_append(profile,
                                               first_new_box_index,
                                               "remove_suffix.partition_append");
    } else {
        if (profile.boxes_added > 0) {
            connect_incremental_boxes(adjacency_, boxes_, first_new_box_index, config_.query.adjacency_tolerance);
            apply_segment_edges_to_adjacency(segment_edges_, adjacency_);
            invalidate_query_cache();
        }
        profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
    }
    profile.adjacency_ms = std::chrono::duration<double, std::milli>(Clock::now() - adj_t0_delete).count();
    profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    invalidate_query_cache();
    return profile;
}

} // namespace rbf
