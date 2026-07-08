#pragma once

#include <SBF/find_free_box_types.h>
#include <SBF/runtime_fwd.h>

namespace rbf {

struct RBFPlanningConfig;

struct QueryBridgeEdgeRuntimeOptions {
    bool scene_reusable_edges = false;
    bool direct_segment_after_rrt = false;
    bool fast_direct_segment_after_rrt = false;
    bool direct_segment_edges_enabled = false;
    bool direct_start_goal_segment_enabled = false;
    bool fast_direct_segment_after_rrt_enabled = false;
    int fast_direct_random_shortcut_iters = 0;
};

struct QueryBridgeWaypointShortcutOptions {
    bool enabled = false;
    double min_gain = 0.0;
};

struct QueryBridgeDirectCorridorRuntimeOptions {
    double max_length = 0.0;
    double audit_step = 0.01;
    double sample_step = 0.01;
    int partition_append_batch_size = 0;
    bool full_residual_overlay_when_connected = false;
};

QueryBridgeEdgeRuntimeOptions query_bridge_edge_runtime_options_from_config(
    const RBFPlanningConfig& config);

void record_query_bridge_edge_runtime_diagnostics(
    StageContext& context,
    const QueryBridgeEdgeRuntimeOptions& options);

QueryBridgeWaypointShortcutOptions query_bridge_waypoint_shortcut_options(
    bool direct_segment_after_rrt_candidate);

bool query_bridge_internal_simplify_enabled(bool direct_segment_after_rrt_candidate);

QueryBridgeDirectCorridorRuntimeOptions query_bridge_direct_corridor_runtime_options(
    const RBFPlanningConfig& config,
    int query_index,
    double audit_step);

FindFreeBoxOptions query_bridge_direct_ffb_options(
    const RBFPlanningConfig& config,
    int max_depth);

}  // namespace rbf
