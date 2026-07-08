#include "planning_forest_query_bridge_corridor_options.h"

#include <SBF/safe_box_forest.h>

#include <algorithm>

namespace rbf {

QueryBridgeEdgeRuntimeOptions query_bridge_edge_runtime_options_from_config(
    const RBFPlanningConfig& config) {
    QueryBridgeEdgeRuntimeOptions options;
    options.scene_reusable_edges = config.query_bridge_scene_reusable_edges;
    options.direct_segment_after_rrt = config.query_bridge_direct_segment_after_rrt;
    options.fast_direct_segment_after_rrt = config.query_bridge_fast_direct_segment_after_rrt;
    const bool segment_edges_active =
        config.connector.segment_edges_enabled &&
        config.connector.rrt_segment_edges;
    options.direct_segment_edges_enabled =
        options.direct_segment_after_rrt && segment_edges_active;
    options.direct_start_goal_segment_enabled =
        options.direct_segment_edges_enabled;
    options.fast_direct_segment_after_rrt_enabled =
        options.direct_segment_edges_enabled &&
        options.fast_direct_segment_after_rrt;
    options.fast_direct_random_shortcut_iters =
        std::max(0, config.query_bridge_fast_direct_random_shortcut_iters);
    return options;
}

void record_query_bridge_edge_runtime_diagnostics(
    StageContext& context,
    const QueryBridgeEdgeRuntimeOptions& options) {
    context.diagnostics().set_value("query_bridge.scene_reusable_edges",
                                    options.scene_reusable_edges ? 1.0 : 0.0);
    context.diagnostics().set_value(
        "query_bridge.direct_segment_after_rrt",
        options.direct_segment_after_rrt ? 1.0 : 0.0);
    context.diagnostics().set_value(
        "query_bridge.direct_start_goal_segment",
        options.direct_start_goal_segment_enabled ? 1.0 : 0.0);
    context.diagnostics().set_value(
        "query_bridge.fast_direct_segment_after_rrt",
        options.fast_direct_segment_after_rrt_enabled ? 1.0 : 0.0);
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
    const RBFPlanningConfig& config,
    int query_index,
    double audit_step) {
    QueryBridgeDirectCorridorRuntimeOptions options;
    options.max_length = std::max(0.0, config.query_bridge_direct_max_length);
    options.audit_step = audit_step > 0.0 ? audit_step : 0.01;
    const double base_sample_step =
        config.query_bridge_direct_sample_step > 0.0
            ? config.query_bridge_direct_sample_step
            : options.audit_step;
    double sample_step = base_sample_step;
    if (query_index >= 0 &&
        static_cast<std::size_t>(query_index) <
            config.query_bridge_direct_sample_steps_by_query.size()) {
        const double indexed =
            config.query_bridge_direct_sample_steps_by_query[query_index];
        if (indexed > 0.0) {
            sample_step = indexed;
        }
    }
    options.sample_step = std::max(1e-4, sample_step);
    options.partition_append_batch_size = 32;
    options.full_residual_overlay_when_connected =
        config.query_bridge_full_residual_overlay_when_connected;
    return options;
}

FindFreeBoxOptions query_bridge_direct_ffb_options(
    const RBFPlanningConfig& config,
    int max_depth) {
    FindFreeBoxOptions options = config.connector.pave.find_free_box;
    options.max_depth = max_depth;
    if (config.query_bridge_ffb_start_depth >= 0) {
        options.start_depth = config.query_bridge_ffb_start_depth;
        options.skip_to_depth = config.query_bridge_ffb_start_depth;
    }
    options.reject_seed_collision = false;
    options.skip_existing_cover_check = true;
    options.materialize_result_node = false;
    options.record_diagnostics = false;
    return options;
}

}  // namespace rbf
