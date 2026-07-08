#include <SBF/safe_box_forest.h>

#include <SBF/oracle.h>

#include <algorithm>
#include <chrono>
#include <utility>
#include <vector>

#include "planning_forest_qroot_helpers.h"

namespace rbf {

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

}  // namespace rbf
