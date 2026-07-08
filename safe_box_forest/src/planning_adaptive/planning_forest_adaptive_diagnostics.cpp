#include "planning_forest_adaptive_diagnostics.h"

#include <SBF/runtime.h>

#include "planning_forest_adaptive_merge.h"
#include "../planning_core/planning_forest_diagnostics.h"

#include <SBF/box_graph.h>
#include <SBF/planning_result.h>

namespace rbf {

void record_adaptive_merge_diagnostics(std::unordered_map<std::string, double>& diagnostics,
                                       const BudgetedMergeStats& stats) {
    diagnostics["adaptive.merge_input_boxes"] = static_cast<double>(stats.input_boxes);
    diagnostics["adaptive.merge_output_boxes"] = static_cast<double>(stats.output_boxes);
    diagnostics["adaptive.merge_grid_ms"] = stats.grid_ms;
    diagnostics["adaptive.merge_grid_merges"] = static_cast<double>(stats.grid_merges);
    diagnostics["adaptive.merge_grid_rounds"] = static_cast<double>(stats.grid_rounds);
    diagnostics["adaptive.merge_tree_ms"] = stats.tree_ms;
    diagnostics["adaptive.merge_tree_merges"] = static_cast<double>(stats.tree_merges);
    diagnostics["adaptive.merge_tree_rounds"] = static_cast<double>(stats.tree_rounds);
    diagnostics["adaptive.merge_containment_ms"] = stats.containment_ms;
    diagnostics["adaptive.merge_exact_ms"] = stats.exact_ms;
    diagnostics["adaptive.merge_containment_pruned"] =
        static_cast<double>(stats.containment_pruned);
    diagnostics["adaptive.merge_exact_merges"] = static_cast<double>(stats.exact_merges);
    diagnostics["adaptive.merge_rounds"] = static_cast<double>(stats.rounds);
    diagnostics["adaptive.merge_stop_reason"] = static_cast<double>(stats.stop_reason);
}

void record_adaptive_partition_merge_diagnostics(
    std::unordered_map<std::string, double>& diagnostics,
    const std::unordered_map<std::string, double>& source) {
    constexpr const char* kKeys[] = {
        "adaptive.partition_merge_enabled",
        "adaptive.partition_merge_released_boxes",
        "adaptive.partition_merge_containment_skipped",
        "adaptive.partition_merge_containment_bucket_entries",
        "adaptive.partition_merge_containment_candidates",
        "adaptive.partition_merge_containment_tests",
        "adaptive.partition_merge_containment_overflow",
        "adaptive.partition_merge_containment_ms",
        "adaptive.partition_merge_line_ms",
        "adaptive.partition_skipped_graph_adjacency",
    };
    for (const char* key : kKeys) {
        diagnostics[key] = diagnostic_map_value(source, key);
    }
}

void record_adaptive_adjacency_diagnostics(std::unordered_map<std::string, double>& diagnostics,
                                           const std::string& prefix,
                                           const AdjacencyBuildStats& stats) {
    diagnostics[prefix + "ms"] = stats.build_ms;
    diagnostics[prefix + "boxes"] = static_cast<double>(stats.boxes);
    diagnostics[prefix + "selected_dims"] = static_cast<double>(stats.selected_dims);
    diagnostics[prefix + "primary_dim"] = static_cast<double>(stats.primary_dim);
    diagnostics[prefix + "candidates"] = static_cast<double>(stats.candidate_pairs);
    diagnostics[prefix + "exact_tests"] = static_cast<double>(stats.exact_tests);
    diagnostics[prefix + "edges"] = static_cast<double>(stats.edges);
}

void record_adaptive_depth_gate_diagnostics(
    std::unordered_map<std::string, double>& diagnostics,
    const AdaptiveLeafSweepConfig& config,
    bool adaptive_depth_enabled,
    int adaptive_depth_min,
    int target_leaf_depth) {
    diagnostics["adaptive.depth_enabled"] = adaptive_depth_enabled ? 1.0 : 0.0;
    diagnostics["adaptive.depth_min"] = static_cast<double>(adaptive_depth_min);
    diagnostics["adaptive.depth_max"] = static_cast<double>(target_leaf_depth);
    diagnostics["adaptive.depth_min_covered_probes"] =
        static_cast<double>(config.adaptive_depth_min_covered_probes);
    diagnostics["adaptive.depth_min_main_probes"] =
        static_cast<double>(config.adaptive_depth_min_main_probes);
    diagnostics["adaptive.depth_min_main_ratio"] = config.adaptive_depth_min_main_ratio;
    diagnostics["adaptive.depth_min_cells"] =
        static_cast<double>(config.adaptive_depth_min_cells);
    diagnostics["adaptive.depth_min_main_cells"] =
        static_cast<double>(config.adaptive_depth_min_main_cells);
    diagnostics["adaptive.depth_max_online_cells"] =
        static_cast<double>(config.adaptive_depth_max_online_cells);
}

void record_adaptive_probe_diagnostics(std::unordered_map<std::string, double>& diagnostics,
                                       const AdaptiveLeafSweepResult& result,
                                       int anchor_cap,
                                       int anchor_attempts) {
    diagnostics["adaptive.seed_probe_count"] = static_cast<double>(result.seed_probe_count);
    diagnostics["adaptive.seed_probe_free_count"] =
        static_cast<double>(result.seed_probe_free_count);
    diagnostics["adaptive.seed_probe_box_covered"] =
        static_cast<double>(result.seed_probe_box_covered);
    diagnostics["adaptive.seed_probe_anchor_success"] =
        static_cast<double>(result.seed_probe_anchor_success);
    diagnostics["adaptive.seed_probe_main_accessible"] =
        static_cast<double>(result.seed_probe_main_accessible);
    diagnostics["adaptive.seed_anchor_probe_cap"] = static_cast<double>(anchor_cap);
    diagnostics["adaptive.seed_anchor_probe_attempts"] = static_cast<double>(anchor_attempts);
    diagnostics["adaptive.p_box_covered"] = result.p_box_covered;
    diagnostics["adaptive.p_anchor_success"] = result.p_anchor_success;
    diagnostics["adaptive.p_main_accessible"] = result.p_main_accessible;
    diagnostics["adaptive.p_anchor_to_main_uncovered"] = result.p_anchor_to_main_uncovered;
}

}  // namespace rbf
