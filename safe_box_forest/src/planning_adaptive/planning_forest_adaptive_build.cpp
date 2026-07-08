#include <SBF/safe_box_forest.h>
#include <SBF/adaptive_grid_partition.h>

#include <algorithm>
#include <chrono>
#include <exception>
#include <limits>
#include <queue>
#include <unordered_set>

#include "planning_forest_adaptive_cover_utils.h"
#include "planning_forest_adaptive_merge.h"
#include "planning_forest_qroot_helpers.h"
#include "planning_forest_query_utils.h"

namespace rbf {

namespace {

bool intervals_overlap_local(const std::vector<Interval>& lhs,
                             const std::vector<Interval>& rhs,
                             double tolerance = 0.0) {
    if (lhs.size() != rhs.size()) {
        return false;
    }
    for (std::size_t dim = 0; dim < lhs.size(); ++dim) {
        if (!lhs[dim].overlaps(rhs[dim], tolerance)) {
            return false;
        }
    }
    return true;
}


}  // namespace

AdaptiveLeafSweepResult RBFPlanningForest::build_adaptive_deep_leaf_sweep_cover(
    const std::vector<Obstacle>& obstacles,
    const AdaptiveLeafSweepConfig& adaptive_config) {
    using Clock = std::chrono::steady_clock;
    const auto total_start = Clock::now();
    AdaptiveLeafSweepResult out;
    initialize_adaptive_leaf_sweep_result(out, adaptive_config);

    const AdaptiveLeafBuildSetup build_setup =
        make_adaptive_leaf_build_setup(adaptive_config);
    const bool adaptive_depth_enabled = build_setup.adaptive_depth_enabled;
    const int adaptive_depth_min = build_setup.adaptive_depth_min;
    const int initial_leaf_depth = build_setup.initial_leaf_depth;
    const int target_leaf_depth = build_setup.target_leaf_depth;
    LeafSweepConfig leaf_config = build_setup.leaf_config;
    const AdaptiveLeafSweepConfig partition_config = build_setup.partition_config;

    if (adaptive_depth_enabled && adaptive_config.fast_virtual_checkpoint_mode) {
        return build_adaptive_fast_virtual_checkpoint_cover(obstacles,
                                                            adaptive_config,
                                                            initial_leaf_depth,
                                                            adaptive_depth_min,
                                                            target_leaf_depth,
                                                            leaf_config,
                                                            partition_config,
                                                            total_start);
    }

    if (adaptive_config.node_budget <= 0 && !adaptive_depth_enabled) {
        return build_fixed_virtual_leaf_sweep_cover(obstacles,
                                                    adaptive_config,
                                                    initial_leaf_depth,
                                                    adaptive_depth_min,
                                                    target_leaf_depth,
                                                    leaf_config,
                                                    partition_config,
                                                    total_start);
    }

    out.leaf_sweep = build_leaf_sweep(obstacles,
                                      adaptive_config.shallow_start_depth,
                                      initial_leaf_depth,
                                      leaf_config);
    out.leaf_sweep_ms = out.leaf_sweep.total_ms;
    out.shallow_free_count = static_cast<int>(out.leaf_sweep.free_boxes.size());
    out.shallow_collision_count = static_cast<int>(out.leaf_sweep.collision_boxes.size());

    const double adjacency_tolerance = config_.query.adjacency_tolerance;
    const bool use_partition_backend = adaptive_config.planning_backend == "partition_native";
    BudgetedMergeStats merge_stats;
    AdjacencyBuildStats initial_adjacency_stats;
    std::unordered_set<int> main_ids;
    const double merge_ms = initialize_adaptive_build_topology(adaptive_config,
                                                               partition_config,
                                                               adjacency_tolerance,
                                                               out,
                                                               merge_stats,
                                                               initial_adjacency_stats,
                                                               main_ids,
                                                               use_partition_backend);
    auto refresh_main_from_partition = [&]() -> bool {
        if (!adaptive_partition_query_enabled_ || !adaptive_partition_) {
            return false;
        }
        const auto largest = adaptive_partition_->largest_component_box_ids_with_overlay();
        main_ids.clear();
        main_ids.insert(largest.begin(), largest.end());
        return !main_ids.empty();
    };
    std::vector<BoxNode> scoring_boxes = boxes_;
    std::vector<AdaptiveFrontierItem> deferred;
    deferred.reserve(out.leaf_sweep.collision_boxes.size());
    const auto planning_domain = oracle_ ? oracle_->planning_intervals() : std::vector<Interval>{};
    int probe_attempted = 0;
    const auto probe_seed_start = Clock::now();
    std::vector<Eigen::VectorXd> free_probes =
        oracle_ ? adaptive_generate_initial_free_probes(*oracle_,
                                                        planning_domain,
                                                        adaptive_config,
                                                        adaptive_depth_enabled,
                                                        probe_attempted)
                : std::vector<Eigen::VectorXd>{};
    const double initial_probe_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - probe_seed_start).count();
    out.seed_probe_count = probe_attempted;
    out.seed_probe_free_count = static_cast<int>(free_probes.size());

    auto item_less = [](const AdaptiveFrontierItem& lhs, const AdaptiveFrontierItem& rhs) {
        return lhs.score < rhs.score;
    };
    std::priority_queue<AdaptiveFrontierItem,
                        std::vector<AdaptiveFrontierItem>,
                        decltype(item_less)>
        frontier(item_less);

    auto refresh_score = [&](AdaptiveFrontierItem& item) {
        item.score = adaptive_frontier_score(scoring_boxes,
                                             item,
                                             main_ids,
                                             adaptive_config.overlap_depth_threshold,
                                             adjacency_tolerance);
    };
    auto push_frontier = [&](AdaptiveFrontierItem item) {
        item.free_seed_hits = adaptive_count_seed_hits(item, free_probes);
        if (item.free_seed_hits > 0) {
            out.diagnostics["adaptive.frontier_seed_hit_pushes"] += 1.0;
            out.diagnostics["adaptive.frontier_seed_hits_total"] += static_cast<double>(item.free_seed_hits);
        }
        refresh_score(item);
        frontier.push(std::move(item));
    };

    for (const auto& collision_box : out.leaf_sweep.collision_boxes) {
        AdaptiveFrontierItem item;
        item.node = collision_box.tree_id >= 0 ? collision_box.tree_id : collision_box.id;
        item.intervals = collision_box.joint_intervals;
        item.changed_dim = -1;
        if (!planning_domain.empty() && !intervals_overlap_local(item.intervals, planning_domain, 0.0)) {
            out.diagnostics["adaptive.initial_frontier_outside_domain"] += 1.0;
            continue;
        }
        push_frontier(std::move(item));
    }

    const auto adaptive_start = Clock::now();
    auto elapsed_total_ms = [&]() {
        return std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
    };
    auto budget_exhausted = [&]() {
        if (adaptive_config.time_budget_ms > 0.0 &&
            elapsed_total_ms() >= adaptive_config.time_budget_ms) {
            return true;
        }
        return adaptive_config.node_budget > 0 &&
               out.adaptive_validated >= adaptive_config.node_budget;
    };
    auto promote_deferred = [&]() {
        if (!adaptive_config.seed_promote_uncovered || deferred.empty() || free_probes.empty()) {
            return;
        }
        std::vector<AdaptiveFrontierItem> keep;
        keep.reserve(deferred.size());
        for (auto& item : deferred) {
            const int hits = adaptive_count_seed_hits(item, free_probes);
            if (hits > 0) {
                item.free_seed_hits = hits;
                refresh_score(item);
                frontier.push(std::move(item));
                out.adaptive_promoted += 1;
                out.diagnostics["adaptive.promoted_by_seed_probe"] += 1.0;
            } else {
                keep.push_back(std::move(item));
            }
        }
        deferred = std::move(keep);
    };

    const auto& split_descriptor = oracle_->database().split_policy_descriptor();
    std::size_t first_unconnected_new_index = boxes_.size();
    int pending_adjacency_boxes = 0;
    constexpr int kAdaptiveAdjacencyBatchSize = 512;
    std::vector<AdaptiveDepthSnapshot> depth_snapshots;
    double checkpoint_probe_ms_total = 0.0;
    auto evaluate_depth_snapshot = [&](int depth, bool allow_anchor_probe) {
        return evaluate_adaptive_depth_snapshot(depth,
                                                allow_anchor_probe,
                                                adaptive_depth_enabled,
                                                target_leaf_depth,
                                                out.leaf_sweep,
                                                free_probes,
                                                planning_domain,
                                                adaptive_config,
                                                use_partition_backend,
                                                main_ids,
                                                first_unconnected_new_index,
                                                pending_adjacency_boxes,
                                                adjacency_tolerance);
    };
    auto record_depth_snapshot = [&](AdaptiveDepthSnapshot snapshot) {
        checkpoint_probe_ms_total += snapshot.probe_ms;
        depth_snapshots.push_back(std::move(snapshot));
    };
    bool adaptive_depth_stop = false;
    int next_checkpoint_depth = initial_leaf_depth;
    if (adaptive_depth_enabled) {
        auto initial_snapshot = evaluate_depth_snapshot(initial_leaf_depth, true);
        if (initial_snapshot.readiness_met) {
            initial_snapshot.stop_reason = "coverage_ready";
            adaptive_depth_stop = true;
        } else if (initial_leaf_depth >= target_leaf_depth) {
            initial_snapshot.stop_reason = "max_depth";
            adaptive_depth_stop = true;
        } else {
            initial_snapshot.stop_reason = "checkpoint";
            next_checkpoint_depth = adaptive_next_depth_checkpoint(initial_leaf_depth, target_leaf_depth);
        }
        record_depth_snapshot(std::move(initial_snapshot));
    }
    std::vector<AdaptiveFrontierItem> checkpoint_hold;
    auto restore_checkpoint_hold = [&]() {
        for (auto& held : checkpoint_hold) {
            frontier.push(std::move(held));
        }
        checkpoint_hold.clear();
    };
    {
        const bool collect_overlap_ratio =
            adaptive_config.overlap_ratio_threshold > 0.0 &&
            adaptive_config.defer_min_depth >= 0;
        ScopedAdaptiveFullOverlapStats overlap_stats(*oracle_, collect_overlap_ratio);
        while (!frontier.empty() && !budget_exhausted() && !adaptive_depth_stop) {
            AdaptiveFrontierItem item = frontier.top();
            frontier.pop();
            if (item.intervals.empty()) {
                out.diagnostics["adaptive.empty_frontier_items"] += 1.0;
                continue;
            }
            const int depth = adaptive_virtual_depth(item.node);
            if (adaptive_depth_enabled && depth > next_checkpoint_depth) {
                checkpoint_hold.push_back(std::move(item));
                if (frontier.empty()) {
                    restore_checkpoint_hold();
                    auto snapshot = evaluate_depth_snapshot(next_checkpoint_depth, true);
                    if (snapshot.readiness_met) {
                        snapshot.stop_reason = "coverage_ready";
                        adaptive_depth_stop = true;
                    } else if (next_checkpoint_depth >= target_leaf_depth) {
                        snapshot.stop_reason = "max_depth";
                        adaptive_depth_stop = true;
                    } else {
                        snapshot.stop_reason = "checkpoint";
                        next_checkpoint_depth =
                            adaptive_next_depth_checkpoint(next_checkpoint_depth, target_leaf_depth);
                    }
                    record_depth_snapshot(std::move(snapshot));
                }
                continue;
            }
            adaptive_add_depth_counter(out.diagnostics, "adaptive.depth.validated.", depth);
            BoxValidation validation = BoxValidation::Unknown;
            OracleValidationDetail detail;
            try {
                validation = oracle_->validate_node(item.node, item.intervals, item.changed_dim);
                detail = oracle_->last_validation_detail();
            } catch (const std::exception&) {
                out.diagnostics["adaptive.validation_exceptions"] += 1.0;
                validation = BoxValidation::Unknown;
            }
            out.adaptive_validated += 1;
            item.overlap_depth = detail.aabb_overlap_depth;
            item.overlap_ratio = detail.aabb_overlap_volume_ratio;
            const bool item_has_seed_hit = item.free_seed_hits > 0;
            if (item_has_seed_hit) {
                out.diagnostics["adaptive.seed_hit_validated"] += 1.0;
            }

            if (validation == BoxValidation::Free) {
                BoxNode candidate = adaptive_make_box_from_intervals(item.intervals,
                                                                     item.node,
                                                                     next_box_id(),
                                                                     detail.safety_status,
                                                                     detail.strict_audit_required);
                bool contained = false;
                for (const auto& existing : boxes_) {
                    if (intervals_subset_local(candidate.joint_intervals,
                                               existing.joint_intervals,
                                               1e-12)) {
                        contained = true;
                        break;
                    }
                }
                if (contained) {
                    out.diagnostics["adaptive.free_contained_rejects"] += 1.0;
                    continue;
                }
                const std::size_t new_index = boxes_.size();
                (void)new_index;
                boxes_.push_back(candidate);
                raw_boxes_.push_back(candidate);
                scoring_boxes.push_back(candidate);
                oracle_->reserve_node(candidate.tree_id, candidate.id);
                out.adaptive_free_added += 1;
                pending_adjacency_boxes += 1;
                adaptive_add_depth_counter(out.diagnostics, "adaptive.depth.free.", depth);
                if (item_has_seed_hit) {
                    out.diagnostics["adaptive.seed_hit_free"] += 1.0;
                }
                if (use_partition_backend && adaptive_partition_query_enabled_ && adaptive_partition_) {
                    const int appended =
                        adaptive_partition_->append_boxes(boxes_, new_index, adjacency_tolerance);
                    out.diagnostics["adaptive.partition_incremental_boxes_appended"] +=
                        static_cast<double>(std::max(0, appended));
                    if (pending_adjacency_boxes >= kAdaptiveAdjacencyBatchSize) {
                        refresh_main_from_partition();
                        pending_adjacency_boxes = 0;
                        out.diagnostics["adaptive.partition_main_refreshes"] += 1.0;
                    }
                } else {
                    adjacency_[candidate.id];
                    if (pending_adjacency_boxes >= kAdaptiveAdjacencyBatchSize) {
                        connect_incremental_boxes(adjacency_,
                                                  boxes_,
                                                  first_unconnected_new_index,
                                                  adjacency_tolerance);
                        first_unconnected_new_index = boxes_.size();
                        pending_adjacency_boxes = 0;
                        main_ids = adaptive_largest_island_ids(adjacency_);
                        out.diagnostics["adaptive.adjacency_batch_updates"] += 1.0;
                    }
                }
                continue;
            }

            const bool high_overlap =
                adaptive_item_high_overlap(adaptive_config, item, depth);
            const bool protected_by_seed = item_has_seed_hit;
            const AdaptiveConnectivityDominance connectivity =
                adaptive_connectivity_dominance(scoring_boxes, item, main_ids, adjacency_tolerance);
            const bool protected_by_adjacency =
                high_overlap && !protected_by_seed &&
                (connectivity.connector_candidate || connectivity.adjacent_main > 0);
            if (depth >= target_leaf_depth) {
                deferred.push_back(std::move(item));
                out.adaptive_deferred += 1;
                out.diagnostics["adaptive.deferred_depth_cap"] += 1.0;
                adaptive_add_depth_counter(out.diagnostics, "adaptive.depth.deferred.", depth);
                if (item_has_seed_hit) {
                    out.diagnostics["adaptive.seed_hit_deferred"] += 1.0;
                }
                continue;
            }
            if (high_overlap && !protected_by_seed && !protected_by_adjacency) {
                deferred.push_back(std::move(item));
                out.adaptive_deferred += 1;
                out.diagnostics["adaptive.deferred_high_overlap"] += 1.0;
                adaptive_add_depth_counter(out.diagnostics, "adaptive.depth.deferred.", depth);
                continue;
            }
            if (depth >= adaptive_config.defer_min_depth &&
                !protected_by_seed &&
                connectivity.has_free_context &&
                connectivity.isolated) {
                deferred.push_back(std::move(item));
                out.adaptive_deferred += 1;
                out.diagnostics["adaptive.deferred_connectivity_isolated"] += 1.0;
                adaptive_add_depth_counter(out.diagnostics, "adaptive.depth.deferred.", depth);
                continue;
            }
            if (depth >= adaptive_config.defer_min_depth &&
                !protected_by_seed &&
                connectivity.has_free_context &&
                connectivity.single_component &&
                connectivity.adjacent_main == 0) {
                deferred.push_back(std::move(item));
                out.adaptive_deferred += 1;
                out.diagnostics["adaptive.deferred_connectivity_single_component"] += 1.0;
                adaptive_add_depth_counter(out.diagnostics, "adaptive.depth.deferred.", depth);
                continue;
            }

            AdaptiveFrontierItem left;
            AdaptiveFrontierItem right;
            if (!adaptive_virtual_split_node(split_descriptor, item, left, right)) {
                deferred.push_back(std::move(item));
                out.adaptive_deferred += 1;
                out.diagnostics["adaptive.deferred_split_failure"] += 1.0;
                continue;
            }
            out.adaptive_splits += 1;
            if (item_has_seed_hit) {
                out.diagnostics["adaptive.seed_hit_splits"] += 1.0;
            }
            adaptive_add_depth_counter(out.diagnostics, "adaptive.depth.split.", depth);
            if (planning_domain.empty() || intervals_overlap_local(left.intervals, planning_domain, 0.0)) {
                push_frontier(std::move(left));
            } else {
                out.diagnostics["adaptive.split_child_outside_domain"] += 1.0;
            }
            if (planning_domain.empty() || intervals_overlap_local(right.intervals, planning_domain, 0.0)) {
                push_frontier(std::move(right));
            } else {
                out.diagnostics["adaptive.split_child_outside_domain"] += 1.0;
            }
            if (adaptive_config.promotion_interval > 0 &&
                out.adaptive_validated % adaptive_config.promotion_interval == 0) {
                promote_deferred();
            }
        }
    }
    restore_checkpoint_hold();
    if (adaptive_depth_enabled && !adaptive_depth_stop && budget_exhausted()) {
        auto snapshot = evaluate_depth_snapshot(depth_snapshots.empty()
                                                    ? initial_leaf_depth
                                                    : depth_snapshots.back().depth,
                                                true);
        snapshot.stop_reason = "budget";
        adaptive_depth_stop = true;
        record_depth_snapshot(std::move(snapshot));
    } else if (adaptive_depth_enabled && !adaptive_depth_stop && frontier.empty()) {
        auto snapshot = evaluate_depth_snapshot(depth_snapshots.empty()
                                                    ? initial_leaf_depth
                                                    : depth_snapshots.back().depth,
                                                true);
        snapshot.stop_reason = "frontier_empty";
        adaptive_depth_stop = true;
        record_depth_snapshot(std::move(snapshot));
    }
    promote_deferred();
    while (!frontier.empty()) {
        deferred.push_back(frontier.top());
        frontier.pop();
    }
    out.unresolved_domains = static_cast<int>(deferred.size());
    out.adaptive_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - adaptive_start).count();

    AdjacencyBuildStats final_adjacency_stats;
    if (use_partition_backend && adaptive_partition_query_enabled_ && adaptive_partition_) {
        refresh_main_from_partition();
    } else if (!use_partition_backend) {
        rebuild_adjacency();
        final_adjacency_stats = last_adjacency_build_stats();
        main_ids = adaptive_largest_island_ids(adjacency_);
    } else {
        out.diagnostics["adaptive.partition_missing_no_graph_fallback"] = 1.0;
    }
    if (depth_snapshots.empty()) {
        auto snapshot = evaluate_depth_snapshot(adaptive_depth_enabled ? initial_leaf_depth : target_leaf_depth,
                                                true);
        snapshot.stop_reason = adaptive_depth_enabled ? "max_depth" : "fixed_depth";
        record_depth_snapshot(std::move(snapshot));
    } else if (!adaptive_depth_enabled || depth_snapshots.back().stop_reason == "checkpoint") {
        auto snapshot = evaluate_depth_snapshot(adaptive_depth_enabled
                                                    ? depth_snapshots.back().depth
                                                    : target_leaf_depth,
                                                true);
        snapshot.stop_reason = adaptive_depth_enabled ? "max_depth" : "fixed_depth";
        record_depth_snapshot(std::move(snapshot));
    }
    finalize_adaptive_deep_leaf_sweep_cover_result(out,
                                                   adaptive_config,
                                                   partition_config,
                                                   adaptive_depth_enabled,
                                                   initial_leaf_depth,
                                                   adaptive_depth_min,
                                                   target_leaf_depth,
                                                   merge_ms,
                                                   merge_stats,
                                                   initial_adjacency_stats,
                                                   final_adjacency_stats,
                                                   depth_snapshots,
                                                   initial_probe_ms,
                                                   checkpoint_probe_ms_total,
                                                   total_start,
                                                   use_partition_backend);
    return out;
}




}  // namespace rbf
