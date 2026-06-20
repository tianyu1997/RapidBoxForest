#include <SBF/safe_box_forest.h>

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
    const auto merge_start = Clock::now();
    BudgetedMergeStats merge_stats;
    if (config_.enable_merger && !boxes_.empty()) {
        bool merged_by_partition = false;
        if (adaptive_config.planning_backend == "partition_native") {
            rebuild_adaptive_partition(partition_config, nullptr);
            if (adaptive_partition_query_enabled_ && adaptive_partition_) {
                AdaptiveGridPartitionMergeOptions options;
                options.max_ms = adaptive_config.max_merge_ms;
                options.max_rounds = adaptive_config.max_merge_rounds;
                options.grid_line_merge = true;
                options.containment_prune = false;
                const auto partition_merge =
                    adaptive_partition_->merge_boxes(boxes_, options, adjacency_tolerance);
                for (int released_id : partition_merge.released_box_ids) {
                    oracle_->release_box(released_id);
                }
                merge_stats.input_boxes = partition_merge.input_boxes;
                merge_stats.output_boxes = partition_merge.output_boxes;
                merge_stats.grid_merges = partition_merge.grid_merges;
                merge_stats.grid_rounds = partition_merge.rounds;
                merge_stats.containment_pruned = partition_merge.containment_pruned;
                merge_stats.stop_reason = partition_merge.stop_reason == 0 ? 1 : partition_merge.stop_reason;
                merge_stats.total_ms = partition_merge.total_ms;
                merge_stats.grid_ms = partition_merge.total_ms;
                out.diagnostics["adaptive.partition_merge_enabled"] = 1.0;
                out.diagnostics["adaptive.partition_merge_released_boxes"] =
                    static_cast<double>(partition_merge.released_box_ids.size());
                out.diagnostics["adaptive.partition_merge_containment_skipped"] =
                    static_cast<double>(partition_merge.containment_skipped);
                out.diagnostics["adaptive.partition_merge_containment_bucket_entries"] =
                    static_cast<double>(partition_merge.containment_bucket_entries);
                out.diagnostics["adaptive.partition_merge_containment_candidates"] =
                    static_cast<double>(partition_merge.containment_candidates);
                out.diagnostics["adaptive.partition_merge_containment_tests"] =
                    static_cast<double>(partition_merge.containment_tests);
                out.diagnostics["adaptive.partition_merge_containment_overflow"] =
                    static_cast<double>(partition_merge.containment_overflow);
                out.diagnostics["adaptive.partition_merge_containment_ms"] =
                    partition_merge.containment_ms;
                out.diagnostics["adaptive.partition_merge_line_ms"] =
                    partition_merge.line_merge_ms;
                merged_by_partition = true;
            }
        }
        if (!merged_by_partition) {
            MergerConfig leaf_merge_config = config_.merger;
            leaf_merge_config.containment_prune = true;
            merge_stats = budgeted_leaf_merge(*oracle_,
                                              boxes_,
                                              leaf_merge_config,
                                              adaptive_config.max_merge_ms,
                                              adaptive_config.max_merge_rounds,
                                              adaptive_config.max_merge_input_boxes,
                                              adjacency_tolerance);
        }
        raw_boxes_ = boxes_;
    } else {
        merge_stats.input_boxes = static_cast<int>(boxes_.size());
        merge_stats.output_boxes = static_cast<int>(boxes_.size());
        merge_stats.stop_reason = 0;
    }
    const double merge_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - merge_start).count();
    const bool use_partition_backend = adaptive_config.planning_backend == "partition_native";
    AdjacencyBuildStats initial_adjacency_stats;
    std::unordered_set<int> main_ids;
    auto refresh_main_from_partition = [&]() -> bool {
        if (!adaptive_partition_query_enabled_ || !adaptive_partition_) {
            return false;
        }
        const auto largest = adaptive_partition_->largest_component_box_ids_with_overlay();
        main_ids.clear();
        main_ids.insert(largest.begin(), largest.end());
        return !main_ids.empty();
    };
    if (use_partition_backend) {
        if (!adaptive_partition_query_enabled_ || !adaptive_partition_) {
            rebuild_adaptive_partition(partition_config, nullptr);
        }
        if (refresh_main_from_partition()) {
            out.diagnostics["adaptive.partition_skipped_graph_adjacency"] = 1.0;
        }
    }
    if (!use_partition_backend) {
        rebuild_adjacency();
        initial_adjacency_stats = last_adjacency_build_stats();
        main_ids = adaptive_largest_island_ids(adjacency_);
    } else if (main_ids.empty()) {
        out.diagnostics["adaptive.partition_missing_no_graph_fallback"] = 1.0;
    }
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
        const auto snapshot_start = Clock::now();
        if (use_partition_backend && adaptive_partition_query_enabled_ && adaptive_partition_) {
            refresh_main_from_partition();
        } else if (!use_partition_backend && pending_adjacency_boxes > 0) {
            connect_incremental_boxes(adjacency_,
                                      boxes_,
                                      first_unconnected_new_index,
                                      adjacency_tolerance);
            first_unconnected_new_index = boxes_.size();
            pending_adjacency_boxes = 0;
            main_ids = adaptive_largest_island_ids(adjacency_);
        }
        AdaptiveDepthSnapshot snapshot;
        snapshot.depth = depth;
        snapshot.free_probe_count = static_cast<int>(free_probes.size());
        snapshot.collision_count = static_cast<int>(out.leaf_sweep.collision_boxes.size());
        if (use_partition_backend && adaptive_partition_query_enabled_ && adaptive_partition_) {
            const auto& stats = adaptive_partition_->stats();
            snapshot.cell_count = stats.cells;
            snapshot.island_count = stats.islands;
            snapshot.main_island_cell_count = static_cast<int>(main_ids.size());
        } else {
            snapshot.cell_count = static_cast<int>(boxes_.size());
            snapshot.island_count = static_cast<int>(find_islands(adjacency_).size());
            snapshot.main_island_cell_count = static_cast<int>(main_ids.size());
        }
        BoxSpatialIndex coverage_index;
        const bool use_partition_coverage =
            use_partition_backend && adaptive_partition_query_enabled_ && adaptive_partition_;
        if (!use_partition_coverage) {
            coverage_index.rebuild(boxes_, adjacency_tolerance);
        }
        std::vector<const Eigen::VectorXd*> uncovered;
        uncovered.reserve(free_probes.size());
        for (const auto& point : free_probes) {
            const int owner = use_partition_coverage
                ? adaptive_partition_->locate_containing_box(point, false, adjacency_tolerance)
                : [&]() {
                      const int owner_index = coverage_index.covering_box(boxes_, point, adjacency_tolerance);
                      return owner_index >= 0 ? boxes_[static_cast<std::size_t>(owner_index)].id : -1;
                  }();
            if (owner >= 0) {
                snapshot.covered_count += 1;
                if (main_ids.find(owner) != main_ids.end()) {
                    snapshot.main_accessible_count += 1;
                }
            } else {
                uncovered.push_back(&point);
            }
        }
        const int anchor_cap = allow_anchor_probe
            ? std::max(0, adaptive_depth_enabled
                           ? adaptive_config.adaptive_depth_anchor_probe_cap
                           : adaptive_config.seed_anchor_probe_cap)
            : 0;
        if (anchor_cap > 0 && !planning_domain.empty() && !uncovered.empty()) {
            StageContext probe_context = StageContext::from_runtime(config_.runtime);
            FindFreeBoxOptions probe_options = config_.grower.find_free_box;
            probe_options.max_depth = target_leaf_depth;
            probe_options.reject_seed_collision = false;
            probe_options.deadline_ms = adaptive_depth_enabled ? 3.0 : 5.0;
            const double max_probe_ms = adaptive_depth_enabled
                ? std::max(0.0, adaptive_config.adaptive_depth_max_probe_ms)
                : 0.0;
            for (const Eigen::VectorXd* point : uncovered) {
                if (snapshot.anchor_probe_attempts >= anchor_cap) {
                    break;
                }
                const double elapsed_ms =
                    std::chrono::duration<double, std::milli>(Clock::now() - snapshot_start).count();
                if (max_probe_ms > 0.0 && elapsed_ms >= max_probe_ms) {
                    break;
                }
                ++snapshot.anchor_probe_attempts;
                const auto ffb = find_free_box_in_domain(*point, planning_domain, probe_context, probe_options);
                if (!ffb.found) {
                    continue;
                }
                snapshot.anchor_success_count += 1;
                const BoxNode anchor = adaptive_make_box_from_intervals(
                    ffb.intervals,
                    ffb.node,
                    -1,
                    ffb.validation_detail.safety_status,
                    ffb.validation_detail.strict_audit_required);
                const bool anchor_main_accessible = use_partition_coverage
                    ? adaptive_partition_->box_adjacent_to_any(anchor, main_ids, adjacency_tolerance)
                    : (!use_partition_backend &&
                       adaptive_has_adjacency_to_any(boxes_, anchor, &main_ids, adjacency_tolerance));
                if (anchor_main_accessible) {
                    snapshot.anchor_to_main_count += 1;
                }
            }
        }
        const double free_den = static_cast<double>(std::max(1, snapshot.free_probe_count));
        snapshot.p_box_covered = static_cast<double>(snapshot.covered_count) / free_den;
        snapshot.p_main_accessible = static_cast<double>(snapshot.main_accessible_count) / free_den;
        snapshot.main_connected_ratio =
            static_cast<double>(snapshot.main_accessible_count) /
            static_cast<double>(std::max(1, snapshot.covered_count));
        snapshot.p_anchor_to_main_uncovered =
            static_cast<double>(snapshot.anchor_to_main_count) /
            static_cast<double>(std::max(1, snapshot.anchor_probe_attempts));
        snapshot.probe_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - snapshot_start).count();
        snapshot.readiness_met =
            adaptive_depth_enabled &&
            adaptive_depth_snapshot_readiness_met(snapshot, adaptive_config);
        return snapshot;
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
