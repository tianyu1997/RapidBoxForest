#include "planning_forest_adaptive_commit.h"

#include <SBF/adaptive_grid_partition.h>
#include <SBF/planning_result.h>
#include <LECTDatabase/sbf/oracle.h>

#include "../qroot/planning_forest_qroot_helpers.h"
#include "../query_runtime/planning_forest_query_utils.h"

#include <algorithm>

namespace rbf {

bool adaptive_commit_free_box_candidate(
    const AdaptiveFrontierItem& item,
    const OracleValidationDetail& detail,
    int depth,
    bool item_has_seed_hit,
    int new_box_id,
    bool use_partition_backend,
    bool adaptive_partition_query_enabled,
    AdaptiveGridPartition* adaptive_partition,
    double adjacency_tolerance,
    int adjacency_batch_size,
    DatabaseBoxOracle& oracle,
    std::vector<BoxNode>& boxes,
    std::vector<BoxNode>& raw_boxes,
    std::vector<BoxNode>& scoring_boxes,
    AdjacencyGraph& adjacency,
    std::unordered_set<int>& main_ids,
    std::size_t& first_unconnected_new_index,
    int& pending_adjacency_boxes,
    AdaptiveLeafSweepResult& result) {
    BoxNode candidate = adaptive_make_box_from_intervals(item.intervals,
                                                         item.node,
                                                         new_box_id,
                                                         detail.safety_status,
                                                         detail.strict_audit_required);
    for (const auto& existing : boxes) {
        if (intervals_subset_local(candidate.joint_intervals,
                                   existing.joint_intervals,
                                   1e-12)) {
            result.diagnostics["adaptive.free_contained_rejects"] += 1.0;
            return false;
        }
    }

    const std::size_t new_index = boxes.size();
    boxes.push_back(candidate);
    raw_boxes.push_back(candidate);
    scoring_boxes.push_back(candidate);
    oracle.reserve_node(candidate.tree_id, candidate.id);
    result.adaptive_free_added += 1;
    pending_adjacency_boxes += 1;
    adaptive_add_depth_counter(result.diagnostics, "adaptive.depth.free.", depth);
    if (item_has_seed_hit) {
        result.diagnostics["adaptive.seed_hit_free"] += 1.0;
    }

    if (use_partition_backend && adaptive_partition_query_enabled && adaptive_partition != nullptr) {
        const int appended =
            adaptive_partition->append_boxes(boxes, new_index, adjacency_tolerance);
        result.diagnostics["adaptive.partition_incremental_boxes_appended"] +=
            static_cast<double>(std::max(0, appended));
        if (pending_adjacency_boxes >= adjacency_batch_size) {
            const auto largest = adaptive_partition->largest_component_box_ids_with_overlay();
            main_ids.clear();
            main_ids.insert(largest.begin(), largest.end());
            pending_adjacency_boxes = 0;
            result.diagnostics["adaptive.partition_main_refreshes"] += 1.0;
        }
        return true;
    }

    adjacency[candidate.id];
    if (pending_adjacency_boxes >= adjacency_batch_size) {
        connect_incremental_boxes(adjacency,
                                  boxes,
                                  first_unconnected_new_index,
                                  adjacency_tolerance);
        first_unconnected_new_index = boxes.size();
        pending_adjacency_boxes = 0;
        main_ids = adaptive_largest_island_ids(adjacency);
        result.diagnostics["adaptive.adjacency_batch_updates"] += 1.0;
    }
    return true;
}

}  // namespace rbf
