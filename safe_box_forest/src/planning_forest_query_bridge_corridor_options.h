#pragma once

#include <SBF/runtime.h>

namespace rbf {

struct QueryBridgeEdgeRuntimeOptions {
    bool scene_reusable_edges = false;
    bool direct_segment_after_rrt = false;
    bool fast_direct_segment_after_rrt = false;
    int fast_direct_random_shortcut_iters = 0;
    double direct_segment_after_rrt_min_length = 0.0;
};

struct QueryBridgeWaypointShortcutOptions {
    bool enabled = false;
    double min_gain = 0.0;
};

struct QueryBridgeDirectCorridorRuntimeOptions {
    double max_length = 0.0;
    double audit_step = 0.01;
    double sample_step = 0.01;
    bool partition_neighbor_candidates = false;
    bool immediate_partition_append = false;
    int partition_append_batch_size = 0;
    bool local_sample_assimilation = true;
    bool group_residual_gaps = false;
    bool full_residual_overlay_when_connected = false;
};

QueryBridgeEdgeRuntimeOptions query_bridge_edge_runtime_options();

QueryBridgeWaypointShortcutOptions query_bridge_waypoint_shortcut_options(
    bool direct_segment_after_rrt_candidate);

bool query_bridge_internal_simplify_enabled(bool direct_segment_after_rrt_candidate);

QueryBridgeDirectCorridorRuntimeOptions query_bridge_direct_corridor_runtime_options(
    int query_index,
    double audit_step);

}  // namespace rbf
