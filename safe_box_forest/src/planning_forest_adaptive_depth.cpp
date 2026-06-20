#include "planning_forest_adaptive_cover_utils.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace rbf {

void initialize_adaptive_leaf_sweep_result(AdaptiveLeafSweepResult& result,
                                           const AdaptiveLeafSweepConfig& config) {
    result.diagnostics["adaptive.offline_query_agnostic_build"] = 1.0;
    result.diagnostics["adaptive.qroot_pairs_total"] = 0.0;
    result.diagnostics["adaptive.qroot_uncovered_endpoints"] = 0.0;
    result.diagnostics["adaptive.fast_virtual_checkpoint_mode"] =
        config.fast_virtual_checkpoint_mode ? 1.0 : 0.0;
    result.diagnostics["adaptive.terminal_controller_enabled"] =
        config.fast_virtual_checkpoint_mode ? 0.0 : 1.0;
}

bool adaptive_depth_snapshot_readiness_met(const AdaptiveDepthSnapshot& snapshot,
                                           const AdaptiveLeafSweepConfig& config) {
    const int min_covered_probes = std::max(0, config.adaptive_depth_min_covered_probes);
    const int min_main_probes = std::max(0, config.adaptive_depth_min_main_probes);
    const int min_cells = std::max(0, config.adaptive_depth_min_cells);
    const int min_main_cells = std::max(0, config.adaptive_depth_min_main_cells);
    if (snapshot.cell_count <= 0 || snapshot.main_island_cell_count <= 0) {
        return false;
    }
    const bool probe_gate =
        snapshot.covered_count >= min_covered_probes &&
        snapshot.main_accessible_count >= min_main_probes &&
        (min_covered_probes <= 0 ||
         snapshot.main_connected_ratio >= config.adaptive_depth_min_main_ratio);
    const bool cell_gate =
        snapshot.cell_count >= min_cells &&
        snapshot.main_island_cell_count >= min_main_cells;
    return probe_gate &&
           cell_gate &&
           (config.adaptive_depth_max_online_cells <= 0 ||
            snapshot.cell_count <= config.adaptive_depth_max_online_cells);
}

AdaptiveDepthSnapshot adaptive_snapshot_from_fast_candidate(const AdaptiveLeafSweepResult& candidate,
                                                            int depth,
                                                            const AdaptiveLeafSweepConfig& config) {
    AdaptiveDepthSnapshot snapshot;
    snapshot.depth = depth;
    snapshot.free_probe_count = candidate.seed_probe_free_count;
    snapshot.covered_count = candidate.seed_probe_box_covered;
    snapshot.main_accessible_count =
        std::min(candidate.seed_probe_main_accessible, candidate.seed_probe_box_covered);
    snapshot.anchor_success_count = candidate.seed_probe_anchor_success;
    snapshot.anchor_to_main_count =
        std::max(0, candidate.seed_probe_main_accessible - snapshot.main_accessible_count);
    const auto attempts_it = candidate.profile.diagnostics.find("adaptive.seed_anchor_probe_attempts");
    if (attempts_it != candidate.profile.diagnostics.end()) {
        snapshot.anchor_probe_attempts = static_cast<int>(std::llround(attempts_it->second));
    }
    snapshot.cell_count = candidate.partition_cell_count > 0
        ? candidate.partition_cell_count
        : candidate.profile.final_boxes;
    snapshot.collision_count = candidate.shallow_collision_count;
    snapshot.island_count = candidate.partition_islands > 0
        ? candidate.partition_islands
        : candidate.profile.adjacency_islands;
    snapshot.main_island_cell_count = candidate.partition_largest_island > 0
        ? candidate.partition_largest_island
        : candidate.profile.grow_largest_island;
    const double free_den = static_cast<double>(std::max(1, snapshot.free_probe_count));
    snapshot.p_box_covered = static_cast<double>(snapshot.covered_count) / free_den;
    snapshot.p_main_accessible = static_cast<double>(snapshot.main_accessible_count) / free_den;
    snapshot.main_connected_ratio =
        static_cast<double>(snapshot.main_accessible_count) /
        static_cast<double>(std::max(1, snapshot.covered_count));
    snapshot.p_anchor_to_main_uncovered =
        static_cast<double>(snapshot.anchor_to_main_count) /
        static_cast<double>(std::max(1, snapshot.anchor_probe_attempts));
    snapshot.probe_ms = candidate.coverage_probe_ms;
    snapshot.readiness_met = adaptive_depth_snapshot_readiness_met(snapshot, config);
    return snapshot;
}

std::string adaptive_depth_snapshots_to_json(const std::vector<AdaptiveDepthSnapshot>& snapshots) {
    std::ostringstream out;
    out << '[';
    for (std::size_t i = 0; i < snapshots.size(); ++i) {
        const auto& snap = snapshots[i];
        if (i > 0) {
            out << ',';
        }
        out << '{'
            << "\"depth\":" << snap.depth
            << ",\"free_probe_count\":" << snap.free_probe_count
            << ",\"covered_count\":" << snap.covered_count
            << ",\"main_accessible_count\":" << snap.main_accessible_count
            << ",\"anchor_success_count\":" << snap.anchor_success_count
            << ",\"anchor_to_main_count\":" << snap.anchor_to_main_count
            << ",\"anchor_probe_attempts\":" << snap.anchor_probe_attempts
            << ",\"cell_count\":" << snap.cell_count
            << ",\"collision_count\":" << snap.collision_count
            << ",\"island_count\":" << snap.island_count
            << ",\"main_island_cell_count\":" << snap.main_island_cell_count
            << ",\"p_box_covered\":" << snap.p_box_covered
            << ",\"p_main_accessible\":" << snap.p_main_accessible
            << ",\"main_connected_ratio\":" << snap.main_connected_ratio
            << ",\"p_anchor_to_main_uncovered\":" << snap.p_anchor_to_main_uncovered
            << ",\"probe_ms\":" << snap.probe_ms
            << ",\"readiness_met\":" << (snap.readiness_met ? "true" : "false")
            << ",\"stop_reason\":\"" << snap.stop_reason << "\""
            << '}';
    }
    out << ']';
    return out.str();
}

int adaptive_next_depth_checkpoint(int depth, int target_leaf_depth) {
    const int step = depth < 16 ? 1 : 2;
    return std::min(target_leaf_depth, depth + step);
}

void apply_adaptive_final_depth_snapshot(AdaptiveLeafSweepResult& result,
                                         const AdaptiveDepthSnapshot& snapshot) {
    result.selected_leaf_depth = snapshot.depth;
    result.adaptive_depth_readiness_met = snapshot.readiness_met;
    result.adaptive_depth_stop_reason = snapshot.stop_reason;
    result.seed_probe_box_covered = snapshot.covered_count;
    result.seed_probe_anchor_success = snapshot.anchor_success_count;
    result.seed_probe_main_accessible =
        snapshot.main_accessible_count + snapshot.anchor_to_main_count;
    result.p_box_covered = snapshot.p_box_covered;
    const double free_den = static_cast<double>(std::max(1, snapshot.free_probe_count));
    result.p_anchor_success = static_cast<double>(snapshot.anchor_success_count) / free_den;
    result.p_main_accessible = static_cast<double>(result.seed_probe_main_accessible) / free_den;
    result.p_anchor_to_main_uncovered = snapshot.p_anchor_to_main_uncovered;
}

}  // namespace rbf
