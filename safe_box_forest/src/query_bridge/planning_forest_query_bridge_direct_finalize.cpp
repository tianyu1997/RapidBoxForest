#include <SBF/safe_box_forest.h>

#include "planning_forest_query_bridge_corridor_diagnostics.h"

namespace rbf {

void RBFPlanningForest::refresh_query_bridge_direct_corridor_partition(
    std::size_t boxes_before) {
    if (boxes_.size() > boxes_before) {
        append_adaptive_partition_boxes(boxes_before,
                                        &last_build_,
                                        "query_bridge.direct_corridor");
    }
    sync_adaptive_partition_segment_edges(&last_build_,
                                          "query_bridge.direct_corridor");
}

int RBFPlanningForest::finish_query_bridge_direct_corridor(
    std::size_t boxes_before,
    int value) {
    refresh_query_bridge_direct_corridor_partition(boxes_before);
    return value;
}

int RBFPlanningForest::finish_query_bridge_direct_corridor_attempt(
    const std::vector<Eigen::VectorXd>& corridor_path,
    const RRTConnectConfig& bridge_rrt,
    const CollisionChecker& checker,
    StageContext& context,
    int query_index,
    int bridge_edge_query_index,
    std::size_t boxes_before_direct_corridor,
    int direct_added,
    int repair_added,
    int local_segment_edges_added,
    bool allow_residual_segments,
    bool add_full_residual_overlay_when_connected,
    bool local_corridor_connected,
    int source_box_id,
    int target_box_id,
    const std::vector<std::vector<int>>& sample_layers,
    const std::vector<int>& final_bad) {
    bool adopt_certified_subchain_attempted = false;
    auto finish_with_committed_count = [&](int extra_edges) {
        return finish_query_bridge_direct_corridor(
            boxes_before_direct_corridor,
            direct_added + repair_added + local_segment_edges_added + extra_edges);
    };

    if (final_bad.empty() &&
        source_box_id >= 0 &&
        target_box_id >= 0 &&
        (local_corridor_connected ||
         box_only_path_connected_partition_first(source_box_id, target_box_id))) {
        try_promote_query_bridge_direct_transition(source_box_id,
                                                   target_box_id,
                                                   sample_layers,
                                                   boxes_before_direct_corridor,
                                                   context,
                                                   query_index,
                                                   bridge_edge_query_index,
                                                   "box_connected",
                                                   adopt_certified_subchain_attempted);
        const int edge_added = add_verified_query_box_corridor_edge(
            source_box_id,
            target_box_id,
            corridor_path,
            bridge_rrt.segment_resolution,
            bridge_edge_query_index);
        return finish_with_committed_count(edge_added > 0 ? 1 : 0);
    }
    if (!final_bad.empty() &&
        allow_residual_segments &&
        local_segment_edges_added > 0 &&
        source_box_id >= 0 &&
        target_box_id >= 0) {
        refresh_query_bridge_direct_corridor_partition(boxes_before_direct_corridor);
        const bool locally_overlay_connected =
            overlay_path_connected_partition_first(source_box_id, target_box_id);
        query_bridge_record_direct_corridor_local_residual_overlay(
            context,
            query_index,
            locally_overlay_connected);
        if (locally_overlay_connected) {
            int full_edge_id = -1;
            if (add_full_residual_overlay_when_connected) {
                full_edge_id =
                    try_add_query_direct_corridor_full_residual_edge(
                        source_box_id,
                        target_box_id,
                        corridor_path,
                        bridge_rrt,
                        checker,
                        context,
                        bridge_edge_query_index,
                        query_index,
                        true,
                        false);
            }
            try_promote_query_bridge_direct_transition(source_box_id,
                                                       target_box_id,
                                                       sample_layers,
                                                       boxes_before_direct_corridor,
                                                       context,
                                                       query_index,
                                                       bridge_edge_query_index,
                                                       "local_residual_overlay",
                                                       adopt_certified_subchain_attempted);
            invalidate_query_cache();
            return finish_with_committed_count(full_edge_id >= 0 ? 1 : 0);
        }
        const int edge_id = try_add_query_direct_corridor_full_residual_edge(
            source_box_id,
            target_box_id,
            corridor_path,
            bridge_rrt,
            checker,
            context,
            bridge_edge_query_index,
            query_index,
            false,
            true);
        if (edge_id == -2) {
            invalidate_query_cache();
            return finish_with_committed_count(0);
        }
        try_promote_query_bridge_direct_transition(source_box_id,
                                                   target_box_id,
                                                   sample_layers,
                                                   boxes_before_direct_corridor,
                                                   context,
                                                   query_index,
                                                   bridge_edge_query_index,
                                                   "full_residual",
                                                   adopt_certified_subchain_attempted);
        invalidate_query_cache();
        return finish_with_committed_count(edge_id >= 0 ? 1 : 0);
    }
    return finish_query_bridge_direct_corridor(boxes_before_direct_corridor, 0);
}

}  // namespace rbf
