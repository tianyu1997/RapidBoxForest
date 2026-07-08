#include <SBF/safe_box_forest.h>

#include <SBF/runtime.h>
#include <SBF/oracle.h>

#include <algorithm>
#include <chrono>
#include <vector>

#include "planning_forest_adaptive_checkpoint.h"
#include "planning_forest_adaptive_cover_utils.h"
#include "../planning_core/planning_forest_diagnostics.h"
#include "../qroot/planning_forest_qroot_helpers.h"

namespace rbf {

AdaptiveLeafSweepResult RBFPlanningForest::build_adaptive_fast_virtual_checkpoint_cover(
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

    std::vector<AdaptiveDepthSnapshot> depth_snapshots;
    AdaptiveLeafSweepResult selected;
    bool have_selected = false;
    double accumulated_leaf_sweep_ms = 0.0;
    int depth = initial_leaf_depth;
    int sweep_count = 0;
    std::vector<int> checkpoint_depths;
    for (int checkpoint = initial_leaf_depth;
         checkpoint <= target_leaf_depth;
         checkpoint = adaptive_next_depth_checkpoint(checkpoint, target_leaf_depth)) {
        checkpoint_depths.push_back(checkpoint);
        if (checkpoint >= target_leaf_depth) {
            break;
        }
    }
    const auto adaptive_sweep_start = Clock::now();
    LeafSweepConfig checkpoint_leaf_config = leaf_config;
    checkpoint_leaf_config.checkpoint_depths = checkpoint_depths;
    checkpoint_leaf_config.checkpoint_callback = [&](const LeafSweepResult& checkpoint_leaf,
                                                     int checkpoint_depth) {
        depth = checkpoint_depth;
        ++sweep_count;
        accumulated_leaf_sweep_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - adaptive_sweep_start).count();
        AdaptiveLeafSweepResult candidate =
            materialize_adaptive_fast_checkpoint_candidate(obstacles,
                                                           adaptive_config,
                                                           target_leaf_depth,
                                                           partition_config,
                                                           total_start,
                                                           checkpoint_leaf,
                                                           depth,
                                                           sweep_count,
                                                           accumulated_leaf_sweep_ms);
        auto snapshot = adaptive_snapshot_from_fast_candidate(candidate, depth, adaptive_config);
        const AdaptiveDepthCheckpointDecision checkpoint =
            advance_adaptive_depth_checkpoint(snapshot, target_leaf_depth);
        depth_snapshots.push_back(snapshot);

        selected = std::move(candidate);
        selected.selected_leaf_depth = depth;
        have_selected = true;
        return checkpoint.stop;
    };
    out.leaf_sweep = build_leaf_sweep(obstacles,
                                      adaptive_config.shallow_start_depth,
                                      target_leaf_depth,
                                      checkpoint_leaf_config);
    if (!have_selected) {
        ++sweep_count;
        accumulated_leaf_sweep_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - adaptive_sweep_start).count();
        selected = materialize_adaptive_fast_checkpoint_candidate(obstacles,
                                                                  adaptive_config,
                                                                  target_leaf_depth,
                                                                  partition_config,
                                                                  total_start,
                                                                  out.leaf_sweep,
                                                                  target_leaf_depth,
                                                                  sweep_count,
                                                                  accumulated_leaf_sweep_ms);
        auto snapshot = adaptive_snapshot_from_fast_candidate(selected, target_leaf_depth, adaptive_config);
        advance_adaptive_depth_checkpoint(snapshot, target_leaf_depth);
        depth_snapshots.push_back(snapshot);
        selected.selected_leaf_depth = target_leaf_depth;
        selected.adaptive_depth_readiness_met = snapshot.readiness_met;
        selected.adaptive_depth_stop_reason = snapshot.stop_reason;
        have_selected = true;
    }
    if (have_selected) {
        for (const auto& [key, value] : out.leaf_sweep.diagnostics) {
            if (key.find("worker_oracle.") != std::string::npos ||
                key.find("external") != std::string::npos ||
                key.find("canonical_frame") != std::string::npos) {
                set_diagnostic_max(selected.profile.diagnostics, key, value);
            }
        }
        const auto& final_snapshot = depth_snapshots.back();
        selected.selected_leaf_depth = final_snapshot.depth;
        selected.adaptive_depth_readiness_met = final_snapshot.readiness_met;
        selected.adaptive_depth_stop_reason = final_snapshot.stop_reason;
        selected.adaptive_depth_snapshots_json =
            adaptive_depth_snapshots_to_json(depth_snapshots);
        selected.seed_probe_box_covered = final_snapshot.covered_count;
        selected.seed_probe_main_accessible =
            final_snapshot.main_accessible_count + final_snapshot.anchor_to_main_count;
        selected.p_box_covered = final_snapshot.p_box_covered;
        selected.p_main_accessible =
            static_cast<double>(selected.seed_probe_main_accessible) /
            static_cast<double>(std::max(1, final_snapshot.free_probe_count));
        selected.p_anchor_to_main_uncovered = final_snapshot.p_anchor_to_main_uncovered;
        selected.total_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
        selected.profile.total_ms = selected.total_ms;
        selected.profile.grow_ms = selected.total_ms;
        selected.profile.diagnostics["adaptive.in_sweep_checkpoints"] =
            static_cast<double>(sweep_count);
        selected.profile.diagnostics["adaptive.fast_checkpoint_mode"] = 1.0;
        selected.profile.diagnostics["adaptive.fast_virtual_checkpoint_mode"] = 1.0;
        selected.profile.diagnostics["adaptive.terminal_controller_enabled"] = 0.0;
        selected.profile.diagnostics["adaptive.in_sweep_checkpoint_mode"] = 1.0;
        selected.profile.diagnostics["adaptive.selected_leaf_depth"] =
            static_cast<double>(selected.selected_leaf_depth);
        selected.profile.diagnostics["adaptive.depth_readiness_met"] =
            selected.adaptive_depth_readiness_met ? 1.0 : 0.0;
        selected.profile.diagnostics["adaptive.depth_enabled"] = 1.0;
        selected.profile.diagnostics["adaptive.depth_min"] = static_cast<double>(adaptive_depth_min);
        selected.profile.diagnostics["adaptive.depth_max"] = static_cast<double>(target_leaf_depth);
        record_depth_semantics_diagnostics(selected.profile.diagnostics,
                                           "adaptive.",
                                           adaptive_config.shallow_start_depth,
                                           initial_leaf_depth,
                                           target_leaf_depth,
                                           config_.grower.find_free_box,
                                           target_leaf_depth);
        if (oracle_) {
            const OracleCounters counters = oracle_->counters();
            normalize_external_evidence_diagnostics(selected.profile.diagnostics, &counters);
        } else {
            normalize_external_evidence_diagnostics(selected.profile.diagnostics);
        }
        record_portal_membership_policy(selected.profile.diagnostics, config_.portal_membership_policy);
        selected.diagnostics = selected.profile.diagnostics;
        last_build_ = selected.profile;
        if (config_.database.checkpoint_after_build && database_) {
            database_->checkpoint();
        }
        return selected;
    }

    return out;
}

}  // namespace rbf
