#include "planning_forest_query_bridge_corridor_options.h"

#include "env_config.h"

#include <algorithm>

namespace rbf {

QueryBridgeEdgeRuntimeOptions query_bridge_edge_runtime_options() {
    QueryBridgeEdgeRuntimeOptions options;
    options.scene_reusable_edges =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_SCENE_REUSABLE_EDGES", 0) != 0;
    options.direct_segment_after_rrt =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_DIRECT_SEGMENT_AFTER_RRT", 0) != 0;
    options.fast_direct_segment_after_rrt =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_FAST_DIRECT_SEGMENT_AFTER_RRT", 0) != 0;
    options.fast_direct_random_shortcut_iters =
        std::max(0,
                 detail::env_int_or_default(
                     "RBF_QUERY_BRIDGE_FAST_DIRECT_RANDOM_SHORTCUT_ITERS",
                     0));
    options.direct_segment_after_rrt_min_length = std::max(
        0.0,
        detail::env_double_or_default("RBF_QUERY_BRIDGE_DIRECT_SEGMENT_AFTER_RRT_MIN_LENGTH",
                                      0.0));
    return options;
}

QueryBridgeWaypointShortcutOptions query_bridge_waypoint_shortcut_options(
    bool direct_segment_after_rrt_candidate) {
    QueryBridgeWaypointShortcutOptions options;
    options.enabled = direct_segment_after_rrt_candidate;
    options.min_gain = 1e-6;
    return options;
}

bool query_bridge_internal_simplify_enabled(bool direct_segment_after_rrt_candidate) {
    return direct_segment_after_rrt_candidate;
}

QueryBridgeDirectCorridorRuntimeOptions query_bridge_direct_corridor_runtime_options(
    int query_index,
    double audit_step) {
    QueryBridgeDirectCorridorRuntimeOptions options;
    options.max_length =
        std::max(0.0, detail::env_double_or_default("RBF_QUERY_BRIDGE_DIRECT_MAX_LENGTH", 6.5));
    options.audit_step = audit_step > 0.0 ? audit_step : 0.01;
    const double base_sample_step =
        detail::env_double_or_default("RBF_QUERY_BRIDGE_DIRECT_SAMPLE_STEP",
                                      options.audit_step);
    options.sample_step = std::max(
        1e-4,
        detail::env_indexed_double_or_default("RBF_QUERY_BRIDGE_DIRECT_SAMPLE_STEP",
                                              query_index,
                                              base_sample_step));
    options.partition_neighbor_candidates =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_PARTITION_NEIGHBOR_CANDIDATES", 0) != 0;
    options.immediate_partition_append =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_DIRECT_APPEND_PARTITION_IMMEDIATE", 0) != 0;
    options.partition_append_batch_size = options.immediate_partition_append
        ? std::max(1,
                   detail::env_int_or_default(
                       "RBF_QUERY_BRIDGE_DIRECT_PARTITION_APPEND_BATCH_SIZE",
                       32))
        : 0;
    options.full_residual_overlay_when_connected =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_FULL_RESIDUAL_OVERLAY_WHEN_CONNECTED",
                                   0) != 0;
    return options;
}

}  // namespace rbf
