#include <SBF/safe_box_forest.h>

#include <SBF/runtime.h>
#include <SBF/adaptive_grid_partition.h>
#include <SBF/box_graph.h>
#include <SBF/oracle.h>

#include "planning_forest_adaptive_cover_utils.h"
#include "planning_forest_adaptive_diagnostics.h"
#include "planning_forest_adaptive_merge.h"
#include "../planning_core/planning_forest_diagnostics.h"

#include <algorithm>
#include <chrono>

namespace rbf {

void RBFPlanningForest::finalize_adaptive_deep_leaf_sweep_cover_result(
    AdaptiveLeafSweepResult& out,
    const AdaptiveLeafSweepConfig& adaptive_config,
    const AdaptiveLeafSweepConfig& partition_config,
    bool adaptive_depth_enabled,
    int initial_leaf_depth,
    int adaptive_depth_min,
    int target_leaf_depth,
    double merge_ms,
    const BudgetedMergeStats& merge_stats,
    const AdjacencyBuildStats& initial_adjacency_stats,
    const AdjacencyBuildStats& final_adjacency_stats,
    std::vector<AdaptiveDepthSnapshot>& depth_snapshots,
    double initial_probe_ms,
    double checkpoint_probe_ms_total,
    std::chrono::steady_clock::time_point total_start,
    bool use_partition_backend) {
    apply_adaptive_final_depth_snapshot(out, depth_snapshots.back());
    out.adaptive_depth_snapshots_json = adaptive_depth_snapshots_to_json(depth_snapshots);
    out.coverage_probe_ms = initial_probe_ms + checkpoint_probe_ms_total;

    out.total_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - total_start).count();
    out.profile = {};
    out.profile.raw_boxes = static_cast<int>(raw_boxes_.size());
    out.profile.final_boxes = static_cast<int>(boxes_.size());
    out.profile.segment_edges = static_cast<int>(segment_edges_.size());
    out.profile.grow_ms = out.leaf_sweep_ms + out.adaptive_ms;
    out.profile.total_ms = out.total_ms;
    out.profile.grow_largest_island = 0;
    if (use_partition_backend) {
        if (adaptive_partition_query_enabled_ && adaptive_partition_) {
            const auto& partition_stats = adaptive_partition_->stats();
            out.profile.grow_adjacency_islands = partition_stats.islands;
            out.profile.adjacency_islands = partition_stats.islands;
            out.profile.grow_largest_island = partition_stats.largest_island;
        }
    } else {
        const auto graph_islands = find_islands(adjacency_);
        out.profile.grow_adjacency_islands = static_cast<int>(graph_islands.size());
        out.profile.adjacency_islands = out.profile.grow_adjacency_islands;
        for (const auto& island : graph_islands) {
            out.profile.grow_largest_island =
                std::max(out.profile.grow_largest_island, static_cast<int>(island.size()));
        }
    }
    out.profile.connector_ms = 0.0;
    out.profile.diagnostics = out.leaf_sweep.diagnostics;
    merge_diagnostic_snapshot(out.profile.diagnostics, out.diagnostics);
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
                                          "adaptive.initial_adjacency_",
                                          initial_adjacency_stats);
    record_adaptive_adjacency_diagnostics(out.profile.diagnostics,
                                          "adaptive.adjacency_",
                                          final_adjacency_stats);
    out.profile.diagnostics["adaptive.adaptive_ms"] = out.adaptive_ms;
    out.profile.diagnostics["adaptive.coverage_probe_ms"] = out.coverage_probe_ms;
    out.profile.diagnostics["adaptive.total_ms"] = out.total_ms;
    record_adaptive_depth_gate_diagnostics(out.profile.diagnostics,
                                           adaptive_config,
                                           adaptive_depth_enabled,
                                           adaptive_depth_min,
                                           target_leaf_depth);
    out.profile.diagnostics["adaptive.shallow_free_count"] = static_cast<double>(out.shallow_free_count);
    out.profile.diagnostics["adaptive.shallow_collision_count"] = static_cast<double>(out.shallow_collision_count);
    out.profile.diagnostics["adaptive.free_added"] = static_cast<double>(out.adaptive_free_added);
    out.profile.diagnostics["adaptive.validated"] = static_cast<double>(out.adaptive_validated);
    out.profile.diagnostics["adaptive.splits"] = static_cast<double>(out.adaptive_splits);
    out.profile.diagnostics["adaptive.deferred"] = static_cast<double>(out.adaptive_deferred);
    out.profile.diagnostics["adaptive.promoted"] = static_cast<double>(out.adaptive_promoted);
    out.profile.diagnostics["adaptive.unresolved_domains"] = static_cast<double>(out.unresolved_domains);
    const int final_anchor_cap = adaptive_depth_enabled
        ? std::max(0, adaptive_config.adaptive_depth_anchor_probe_cap)
        : std::max(0, adaptive_config.seed_anchor_probe_cap);
    const int final_anchor_attempts = depth_snapshots.empty()
        ? 0
        : depth_snapshots.back().anchor_probe_attempts;
    record_adaptive_probe_diagnostics(out.profile.diagnostics,
                                      out,
                                      final_anchor_cap,
                                      final_anchor_attempts);
    out.profile.diagnostics["adaptive.selected_leaf_depth"] = static_cast<double>(out.selected_leaf_depth);
    out.profile.diagnostics["adaptive.depth_readiness_met"] =
        out.adaptive_depth_readiness_met ? 1.0 : 0.0;
    if (use_partition_backend && adaptive_partition_query_enabled_ && adaptive_partition_) {
        refresh_adaptive_partition_diagnostics(&out.profile);
    } else {
        rebuild_adaptive_partition(partition_config, &out.profile);
    }
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
}

}  // namespace rbf
