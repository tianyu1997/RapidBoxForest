#include <SBF/safe_box_forest.h>
#include <SBF/adaptive_grid_partition.h>
#include <SBF/box_graph.h>
#include <SBF/oracle.h>
#include <SBF/runtime.h>

#include <algorithm>
#include <chrono>
#include <unordered_set>

#include "planning_forest_adaptive_checkpoint.h"
#include "planning_forest_adaptive_commit.h"
#include "planning_forest_adaptive_cover_utils.h"
#include "planning_forest_adaptive_frontier.h"
#include "planning_forest_adaptive_merge.h"
#include "planning_forest_adaptive_validation.h"
#include "../qroot/planning_forest_qroot_helpers.h"
#include "../query_runtime/planning_forest_query_utils.h"

namespace rbf {

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

    AdaptiveFrontierQueue frontier;
    adaptive_seed_initial_frontier(out.leaf_sweep.collision_boxes,
                                   planning_domain,
                                   free_probes,
                                   scoring_boxes,
                                   main_ids,
                                   adaptive_config,
                                   adjacency_tolerance,
                                   frontier,
                                   out);

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
        const AdaptiveDepthCheckpointDecision checkpoint =
            advance_adaptive_depth_checkpoint(initial_snapshot, target_leaf_depth);
        adaptive_depth_stop = checkpoint.stop;
        next_checkpoint_depth = checkpoint.next_checkpoint_depth;
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
        StageContext adaptive_context = StageContext::from_runtime(config_.runtime);
        const int adaptive_threads = std::max(1, adaptive_context.executor().n_threads());
        const int validation_batch_limit =
            adaptive_config.parallel_virtual_validation && adaptive_threads > 1
                ? std::max(1, adaptive_config.validation_batch_size)
                : 1;
        AdaptiveFrontierValidationSession validation_session(*oracle_,
                                                             adaptive_context,
                                                             adaptive_threads,
                                                             validation_batch_limit,
                                                             collect_overlap_ratio,
                                                             out.diagnostics);
        auto process_validated_item = [&](AdaptiveFrontierItem item,
                                          BoxValidation validation,
                                          const OracleValidationDetail& detail,
                                          bool exception) {
            const int depth = adaptive_virtual_depth(item.node);
            if (exception) {
                out.diagnostics["adaptive.validation_exceptions"] += 1.0;
            }
            out.adaptive_validated += 1;
            item.overlap_depth = detail.aabb_overlap_depth;
            item.overlap_ratio = detail.aabb_overlap_volume_ratio;
            const bool item_has_seed_hit = item.free_seed_hits > 0;
            if (item_has_seed_hit) {
                out.diagnostics["adaptive.seed_hit_validated"] += 1.0;
            }

            if (validation == BoxValidation::Free) {
                adaptive_commit_free_box_candidate(item,
                                                   detail,
                                                   depth,
                                                   item_has_seed_hit,
                                                   next_box_id(),
                                                   use_partition_backend,
                                                   adaptive_partition_query_enabled_,
                                                   adaptive_partition_.get(),
                                                   adjacency_tolerance,
                                                   kAdaptiveAdjacencyBatchSize,
                                                   *oracle_,
                                                   boxes_,
                                                   raw_boxes_,
                                                   scoring_boxes,
                                                   adjacency_,
                                                   main_ids,
                                                   first_unconnected_new_index,
                                                   pending_adjacency_boxes,
                                                   out);
                return;
            }

            const AdaptiveConnectivityDominance connectivity =
                adaptive_connectivity_dominance(scoring_boxes, item, main_ids, adjacency_tolerance);
            if (adaptive_defer_frontier_item_if_needed(item,
                                                       depth,
                                                       target_leaf_depth,
                                                       connectivity,
                                                       adaptive_config,
                                                       deferred,
                                                       out)) {
                return;
            }

            adaptive_split_frontier_item_and_enqueue(std::move(item),
                                                     depth,
                                                     split_descriptor,
                                                     planning_domain,
                                                     free_probes,
                                                     scoring_boxes,
                                                     main_ids,
                                                     adaptive_config,
                                                     adjacency_tolerance,
                                                     deferred,
                                                     frontier,
                                                     out);
            if (adaptive_config.promotion_interval > 0 &&
                out.adaptive_validated % adaptive_config.promotion_interval == 0) {
                adaptive_promote_deferred_by_seed(deferred,
                                                  free_probes,
                                                  scoring_boxes,
                                                  main_ids,
                                                  adaptive_config,
                                                  adjacency_tolerance,
                                                  frontier,
                                                  out);
            }
        };
        while (!frontier.empty() && !budget_exhausted() && !adaptive_depth_stop) {
            const int remaining_budget =
                adaptive_config.node_budget > 0
                    ? std::max(0, adaptive_config.node_budget - out.adaptive_validated)
                    : validation_batch_limit;
            const int target_batch_size =
                std::max(1, std::min(validation_batch_limit, remaining_budget));
            std::vector<AdaptiveFrontierItem> validation_items;
            validation_items.reserve(static_cast<std::size_t>(target_batch_size));
            while (static_cast<int>(validation_items.size()) < target_batch_size &&
                   !frontier.empty() && !budget_exhausted() && !adaptive_depth_stop) {
                AdaptiveFrontierItem item = frontier.top();
                frontier.pop();
                if (item.intervals.empty()) {
                    out.diagnostics["adaptive.empty_frontier_items"] += 1.0;
                    continue;
                }
                const int depth = adaptive_virtual_depth(item.node);
                if (adaptive_depth_enabled && depth > next_checkpoint_depth) {
                    checkpoint_hold.push_back(std::move(item));
                    if (validation_items.empty() && frontier.empty()) {
                        restore_checkpoint_hold();
                        auto snapshot = evaluate_depth_snapshot(next_checkpoint_depth, true);
                        const AdaptiveDepthCheckpointDecision checkpoint =
                            advance_adaptive_depth_checkpoint(snapshot, target_leaf_depth);
                        adaptive_depth_stop = checkpoint.stop;
                        next_checkpoint_depth = checkpoint.next_checkpoint_depth;
                        record_depth_snapshot(std::move(snapshot));
                    }
                    break;
                }
                adaptive_add_depth_counter(out.diagnostics, "adaptive.depth.validated.", depth);
                validation_items.push_back(std::move(item));
                if (validation_batch_limit <= 1) {
                    break;
                }
            }
            if (validation_items.empty()) {
                continue;
            }
            const auto outcomes = validation_session.validate_batch(validation_items);
            for (std::size_t i = 0; i < validation_items.size(); ++i) {
                process_validated_item(std::move(validation_items[i]),
                                       outcomes[i].validation,
                                       outcomes[i].detail,
                                       outcomes[i].exception);
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
    adaptive_promote_deferred_by_seed(deferred,
                                      free_probes,
                                      scoring_boxes,
                                      main_ids,
                                      adaptive_config,
                                      adjacency_tolerance,
                                      frontier,
                                      out);
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
