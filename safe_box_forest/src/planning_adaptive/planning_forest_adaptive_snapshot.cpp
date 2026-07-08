#include <SBF/safe_box_forest.h>
#include <SBF/adaptive_grid_partition.h>

#include "planning_forest_adaptive_cover_utils.h"
#include "planning_forest_qroot_helpers.h"
#include "planning_forest_query_utils.h"

#include <algorithm>
#include <chrono>

namespace rbf {

AdaptiveDepthSnapshot RBFPlanningForest::evaluate_adaptive_depth_snapshot(
    int depth,
    bool allow_anchor_probe,
    bool adaptive_depth_enabled,
    int target_leaf_depth,
    const LeafSweepResult& leaf_sweep,
    const std::vector<Eigen::VectorXd>& free_probes,
    const std::vector<Interval>& planning_domain,
    const AdaptiveLeafSweepConfig& adaptive_config,
    bool use_partition_backend,
    std::unordered_set<int>& main_ids,
    std::size_t& first_unconnected_new_index,
    int& pending_adjacency_boxes,
    double adjacency_tolerance) {
    using Clock = std::chrono::steady_clock;
    const auto snapshot_start = Clock::now();
    if (use_partition_backend && adaptive_partition_query_enabled_ && adaptive_partition_) {
        const auto largest = adaptive_partition_->largest_component_box_ids_with_overlay();
        main_ids.clear();
        main_ids.insert(largest.begin(), largest.end());
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
    snapshot.collision_count = static_cast<int>(leaf_sweep.collision_boxes.size());
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
}

}  // namespace rbf
