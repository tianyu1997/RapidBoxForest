#include <SBF/safe_box_forest.h>
#include <SBF/adaptive_grid_partition.h>

#include <algorithm>
#include <chrono>
#include <unordered_set>
#include <vector>

#include "planning_forest_adaptive_cover_utils.h"
#include "planning_forest_adaptive_diagnostics.h"
#include "planning_forest_adaptive_merge.h"
#include "planning_forest_diagnostics.h"
#include "planning_forest_qroot_helpers.h"
#include "planning_forest_query_utils.h"

namespace rbf {

AdaptiveLeafSweepResult RBFPlanningForest::build_fixed_virtual_leaf_sweep_cover(
    const std::vector<Obstacle>& obstacles,
    const AdaptiveLeafSweepConfig& adaptive_config,
    int initial_leaf_depth,
    int adaptive_depth_min,
    int target_leaf_depth,
    LeafSweepConfig leaf_config,
    const AdaptiveLeafSweepConfig& partition_config,
    std::chrono::steady_clock::time_point total_start) {
    using Clock = std::chrono::steady_clock;

    AdaptiveLeafSweepResult out;
    out.diagnostics["adaptive.offline_query_agnostic_build"] = 1.0;
    out.diagnostics["adaptive.qroot_pairs_total"] = 0.0;
    out.diagnostics["adaptive.qroot_uncovered_endpoints"] = 0.0;
    out.diagnostics["adaptive.fast_virtual_checkpoint_mode"] =
        adaptive_config.fast_virtual_checkpoint_mode ? 1.0 : 0.0;
    out.diagnostics["adaptive.fixed_virtual_layer_mode"] = 1.0;
    out.diagnostics["adaptive.terminal_controller_enabled"] = 0.0;

    leaf_config.collision_overlap_prune_min_depth = adaptive_config.defer_min_depth;
    leaf_config.collision_overlap_prune_threshold = adaptive_config.overlap_depth_threshold;
    leaf_config.collision_overlap_prune_min_threshold = adaptive_config.overlap_depth_min_threshold;
    leaf_config.collision_overlap_prune_decay_per_depth = adaptive_config.overlap_depth_decay_per_depth;
    leaf_config.collision_overlap_prune_ratio_threshold = adaptive_config.overlap_ratio_threshold;
    out.leaf_sweep = build_leaf_sweep(obstacles,
                                      adaptive_config.shallow_start_depth,
                                      target_leaf_depth,
                                      leaf_config);
    out.selected_leaf_depth = target_leaf_depth;
    out.adaptive_depth_readiness_met = false;
    out.adaptive_depth_stop_reason = "fixed_depth";
    out.leaf_sweep_ms = out.leaf_sweep.total_ms;
    out.shallow_free_count = static_cast<int>(out.leaf_sweep.free_boxes.size());
    out.shallow_collision_count = static_cast<int>(out.leaf_sweep.collision_boxes.size());
    out.adaptive_deferred = static_cast<int>(out.leaf_sweep.collision_boxes.size());
    out.unresolved_domains = static_cast<int>(out.leaf_sweep.collision_boxes.size());

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
    AdjacencyBuildStats adjacency_stats;
    std::unordered_set<int> main_ids;
    int partition_island_count_for_profile = 0;
    int partition_largest_island_for_profile = 0;
    if (use_partition_backend) {
        if (!adaptive_partition_query_enabled_ || !adaptive_partition_) {
            rebuild_adaptive_partition(partition_config, nullptr);
        }
        if (adaptive_partition_query_enabled_ && adaptive_partition_) {
            const auto largest = adaptive_partition_->largest_component_box_ids_with_overlay();
            main_ids.insert(largest.begin(), largest.end());
            const auto& partition_stats = adaptive_partition_->stats();
            partition_island_count_for_profile = partition_stats.islands;
            partition_largest_island_for_profile = partition_stats.largest_island;
            out.diagnostics["adaptive.partition_skipped_graph_adjacency"] = 1.0;
        }
    }
    if (!use_partition_backend) {
        rebuild_adjacency();
        adjacency_stats = last_adjacency_build_stats();
        main_ids = adaptive_largest_island_ids(adjacency_);
    } else if (main_ids.empty()) {
        out.diagnostics["adaptive.partition_missing_no_graph_fallback"] = 1.0;
    }

    const auto coverage_start = Clock::now();
    const auto planning_domain = oracle_ ? oracle_->planning_intervals() : std::vector<Interval>{};
    int probe_attempted = 0;
    std::vector<Eigen::VectorXd> free_probes =
        oracle_ ? adaptive_generate_free_probes(*oracle_,
                                                planning_domain,
                                                adaptive_config.seed_probe_count,
                                                adaptive_config.seed_probe_rng_seed,
                                                probe_attempted)
                : std::vector<Eigen::VectorXd>{};
    out.seed_probe_count = probe_attempted;
    out.seed_probe_free_count = static_cast<int>(free_probes.size());
    int uncovered_anchor_attempts = 0;
    StageContext probe_context = StageContext::from_runtime(config_.runtime);
    FindFreeBoxOptions probe_options = config_.grower.find_free_box;
    probe_options.max_depth = target_leaf_depth;
    probe_options.reject_seed_collision = false;
    probe_options.deadline_ms = 5.0;
    const int anchor_cap = std::max(0, adaptive_config.seed_anchor_probe_cap);
    BoxSpatialIndex coverage_index;
    const bool use_partition_coverage =
        use_partition_backend && adaptive_partition_query_enabled_ && adaptive_partition_;
    if (!use_partition_coverage) {
        coverage_index.rebuild(boxes_, adjacency_tolerance);
    }
    for (const auto& point : free_probes) {
        const int owner = use_partition_coverage
            ? adaptive_partition_->locate_containing_box(point, false, adjacency_tolerance)
            : [&]() {
                  const int owner_index = coverage_index.covering_box(boxes_, point, adjacency_tolerance);
                  return owner_index >= 0 ? boxes_[static_cast<std::size_t>(owner_index)].id : -1;
              }();
        if (owner >= 0) {
            out.seed_probe_box_covered += 1;
            if (main_ids.find(owner) != main_ids.end()) {
                out.seed_probe_main_accessible += 1;
            }
            continue;
        }
        if (uncovered_anchor_attempts >= anchor_cap || planning_domain.empty()) {
            continue;
        }
        ++uncovered_anchor_attempts;
        const auto ffb = find_free_box_in_domain(point, planning_domain, probe_context, probe_options);
        if (!ffb.found) {
            continue;
        }
        out.seed_probe_anchor_success += 1;
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
            out.seed_probe_main_accessible += 1;
        }
    }
    out.coverage_probe_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - coverage_start).count();
    const double free_den = static_cast<double>(std::max(1, out.seed_probe_free_count));
    out.p_box_covered = static_cast<double>(out.seed_probe_box_covered) / free_den;
    out.p_anchor_success = static_cast<double>(out.seed_probe_anchor_success) / free_den;
    out.p_main_accessible = static_cast<double>(out.seed_probe_main_accessible) / free_den;

    out.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
    out.profile = {};
    out.profile.raw_boxes = static_cast<int>(raw_boxes_.size());
    out.profile.final_boxes = static_cast<int>(boxes_.size());
    out.profile.segment_edges = static_cast<int>(segment_edges_.size());
    out.profile.grow_ms = out.leaf_sweep_ms;
    out.profile.total_ms = out.total_ms;
    if (use_partition_backend) {
        out.profile.grow_adjacency_islands = partition_island_count_for_profile;
        out.profile.adjacency_islands = partition_island_count_for_profile;
        out.profile.grow_largest_island = partition_largest_island_for_profile;
    } else {
        const auto graph_islands = find_islands(adjacency_);
        out.profile.grow_adjacency_islands = static_cast<int>(graph_islands.size());
        out.profile.adjacency_islands = out.profile.grow_adjacency_islands;
        for (const auto& island : graph_islands) {
            out.profile.grow_largest_island =
                std::max(out.profile.grow_largest_island, static_cast<int>(island.size()));
        }
    }
    out.profile.diagnostics = out.leaf_sweep.diagnostics;
    out.profile.diagnostics["adaptive.fast_leaf_sweep"] = 1.0;
    out.profile.diagnostics["adaptive.offline_query_agnostic_build"] = 1.0;
    out.profile.diagnostics["adaptive.qroot_pairs_total"] = 0.0;
    out.profile.diagnostics["adaptive.qroot_uncovered_endpoints"] = 0.0;
    out.profile.diagnostics["adaptive.leaf_sweep_ms"] = out.leaf_sweep_ms;
    record_depth_semantics_diagnostics(out.profile.diagnostics,
                                       "adaptive.",
                                       adaptive_config.shallow_start_depth,
                                       initial_leaf_depth,
                                       target_leaf_depth,
                                       config_.grower.find_free_box,
                                       target_leaf_depth);
    out.profile.diagnostics["adaptive.overlap_depth_threshold"] = adaptive_config.overlap_depth_threshold;
    out.profile.diagnostics["adaptive.overlap_depth_min_threshold"] = adaptive_config.overlap_depth_min_threshold;
    out.profile.diagnostics["adaptive.overlap_depth_decay_per_depth"] = adaptive_config.overlap_depth_decay_per_depth;
    out.profile.diagnostics["adaptive.merge_ms"] = merge_ms;
    record_adaptive_merge_diagnostics(out.profile.diagnostics, merge_stats);
    record_adaptive_partition_merge_diagnostics(out.profile.diagnostics, out.diagnostics);
    record_adaptive_adjacency_diagnostics(out.profile.diagnostics,
                                          "adaptive.adjacency_",
                                          adjacency_stats);
    out.profile.diagnostics["adaptive.adaptive_ms"] = 0.0;
    out.profile.diagnostics["adaptive.coverage_probe_ms"] = out.coverage_probe_ms;
    out.profile.diagnostics["adaptive.total_ms"] = out.total_ms;
    record_adaptive_depth_gate_diagnostics(out.profile.diagnostics,
                                           adaptive_config,
                                           false,
                                           adaptive_depth_min,
                                           target_leaf_depth);
    out.profile.diagnostics["adaptive.shallow_free_count"] = static_cast<double>(out.shallow_free_count);
    out.profile.diagnostics["adaptive.shallow_collision_count"] = static_cast<double>(out.shallow_collision_count);
    out.profile.diagnostics["adaptive.deferred"] = static_cast<double>(out.adaptive_deferred);
    out.profile.diagnostics["adaptive.unresolved_domains"] = static_cast<double>(out.unresolved_domains);
    record_adaptive_probe_diagnostics(out.profile.diagnostics,
                                      out,
                                      anchor_cap,
                                      uncovered_anchor_attempts);
    out.profile.diagnostics["adaptive.selected_leaf_depth"] =
        static_cast<double>(out.selected_leaf_depth);
    out.profile.diagnostics["adaptive.depth_readiness_met"] =
        out.adaptive_depth_readiness_met ? 1.0 : 0.0;
    rebuild_adaptive_partition(partition_config, &out.profile);
    if (adaptive_partition_ && !adaptive_partition_->empty()) {
        const auto& partition_stats = adaptive_partition_->stats();
        out.partition_cell_count = partition_stats.cells;
        out.partition_grid_cell_count = partition_stats.grid_cells;
        out.partition_non_grid_cell_count = partition_stats.non_grid_cells;
        out.partition_face_index_entries = partition_stats.face_index_entries;
        out.partition_islands = partition_stats.islands;
        out.partition_largest_island = partition_stats.largest_island;
    }
    if (oracle_) {
        const OracleCounters counters = oracle_->counters();
        normalize_external_evidence_diagnostics(out.profile.diagnostics, &counters);
    } else {
        normalize_external_evidence_diagnostics(out.profile.diagnostics);
    }
    record_portal_membership_policy(out.profile.diagnostics, config_.portal_membership_policy);
    out.diagnostics = out.profile.diagnostics;
    last_build_ = out.profile;
    if (config_.database.checkpoint_after_build && database_) {
        database_->checkpoint();
    }
    return out;
}

}  // namespace rbf
