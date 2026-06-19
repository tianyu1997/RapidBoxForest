#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>
#include <SBF/connector.h>

#include "planning_forest_audit.h"
#include "planning_forest_query_bridge_corridor_graph.h"
#include "planning_forest_query_bridge_corridor_options.h"
#include "planning_forest_query_bridge_path_utils.h"
#include "planning_forest_diagnostics.h"
#include "planning_forest_qroot_helpers.h"
#include "planning_forest_query_utils.h"
#include "virtual_sparse_ffb.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace rbf {

namespace {

ChainPaveConfig make_dense_query_bridge_pave_config(const ChainPaveConfig& base,
                                                    int ffb_depth) {
    ChainPaveConfig config = base;
    config.max_chain = std::max(config.max_chain, 256);
    config.refine_covered_waypoints = true;
    config.fill_gaps = true;
    config.find_free_box.max_depth = ffb_depth;
    config.gap_fill_sample_step = 0.0025;
    config.gap_fill_time_budget_ms = 0.0;
    config.gap_fill_max_ffb_calls = -1;
    config.gap_fill_min_arc_gain = 0.0;
    config.require_connected_chain = true;
    return config;
}

ChainPaveConfig make_deferred_query_bridge_pave_config(const ChainPaveConfig& base,
                                                       int ffb_depth,
                                                       bool short_local_bridge) {
    ChainPaveConfig config = base;
    config.max_chain = std::max(config.max_chain, 256);
    config.refine_covered_waypoints = true;
    config.fill_gaps = true;
    config.find_free_box.max_depth = ffb_depth;
    config.gap_fill_sample_step = std::min(config.gap_fill_sample_step, 0.02);
    config.gap_fill_time_budget_ms =
        std::max(config.gap_fill_time_budget_ms, short_local_bridge ? 350.0 : 200.0);
    config.gap_fill_max_ffb_calls =
        std::max(config.gap_fill_max_ffb_calls, short_local_bridge ? 768 : 512);
    config.gap_fill_min_arc_gain = 0.0;
    config.require_connected_chain = true;
    return config;
}

} // namespace

int RBFPlanningForest::bridge_query_with_waypoint_path(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    const std::vector<Eigen::VectorXd>& waypoint_path,
    bool short_local_bridge,
    const RRTConnectConfig& bridge_rrt,
    int query_index,
    bool allow_residual_segments) {
    if (waypoint_path.empty() || boxes_.empty() || !oracle_) {
        return 0;
    }
    auto finish_bridge = [&](int added_total) {
        sync_adaptive_partition_segment_edges(&last_build_, "query_bridge");
        if (added_total > 0) {
            refresh_adaptive_partition_diagnostics(&last_build_);
        }
        return added_total;
    };
    const int start_box_id = locate_box_partition_first(start, config_.query.nearest_if_outside);
    if (start_box_id < 0) {
        return 0;
    }
    const int goal_box_id = locate_box_partition_first(goal, config_.query.nearest_if_outside);
    if (goal_box_id < 0 || goal_box_id == start_box_id) {
        return 0;
    }
    StageContext context = StageContext::from_runtime(config_.runtime);
    const QueryBridgeEdgeRuntimeOptions edge_options =
        query_bridge_edge_runtime_options();
    const int bridge_edge_query_index =
        edge_options.scene_reusable_edges ? -1 : query_index;
    context.diagnostics().set_value("query_bridge.scene_reusable_edges",
                                    edge_options.scene_reusable_edges ? 1.0 : 0.0);
    context.diagnostics().set_value("query_bridge.direct_segment_after_rrt",
                                    edge_options.direct_segment_after_rrt ? 1.0 : 0.0);
    ScopedStageDiagnosticsFlush pave_diagnostics_flush(last_build_, context);
    CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
    const double bridge_waypoint_length = query_bridge_waypoint_length(waypoint_path);
    const bool direct_segment_after_rrt_candidate =
        edge_options.direct_segment_after_rrt &&
        config_.connector.segment_edges_enabled &&
        config_.connector.rrt_segment_edges;
    context.diagnostics().set_value("query_bridge.direct_segment_after_rrt_candidate",
                                    direct_segment_after_rrt_candidate ? 1.0 : 0.0);
    int direct_segment_edges_added = 0;
    int box_corridor_edges_added = 0;
    const bool defer_query_segment_edge = true;
    const double query_bridge_depth_failures_before =
        boundary_max_depth_failure_count_local(context);
    int next_id = next_box_id();
    auto capped_ffb_depth = [&](int requested_depth) {
        const int max_tree_depth = std::max(1, config_.database.max_tree_depth);
        return std::min(max_tree_depth, std::max(1, requested_depth));
    };
    const int query_bridge_ffb_depth = capped_ffb_depth(
        config_.query_bridge_pave_depth > 0
            ? config_.query_bridge_pave_depth
            : config_.connector.pave.find_free_box.max_depth);
    context.diagnostics().set_value("query_bridge.pave_ffb_depth",
                                    static_cast<double>(query_bridge_ffb_depth));
    std::vector<Eigen::VectorXd> corridor_path = waypoint_path;
    const QueryBridgeWaypointShortcutOptions waypoint_shortcut_options =
        query_bridge_waypoint_shortcut_options(direct_segment_after_rrt_candidate);
    query_bridge_apply_waypoint_shortcut(corridor_path,
                                         checker,
                                         config_.query,
                                         waypoint_shortcut_options,
                                         context,
                                         query_index);
    const bool bridge_internal_simplify =
        query_bridge_internal_simplify_enabled(direct_segment_after_rrt_candidate);
    context.diagnostics().set_value("query_bridge.internal_simplify_enabled",
                                    bridge_internal_simplify ? 1.0 : 0.0);
    query_bridge_apply_internal_simplify(start,
                                         goal,
                                         corridor_path,
                                         checker,
                                         audit_robot_,
                                         config_.connector.rrt,
                                         config_.query,
                                         config_.grower.rng_seed,
                                         bridge_internal_simplify);
    int dense_repair_added = 0;
    bool dense_repair_attempted = false;
    const double audited_bridge_length = query_bridge_waypoint_length(corridor_path);
    context.diagnostics().set_value("query_bridge.direct_segment_after_rrt_final_length",
                                    audited_bridge_length);
    direct_segment_edges_added =
        try_add_query_direct_segment_after_rrt_edge(start_box_id,
                                                    goal_box_id,
                                                    corridor_path,
                                                    bridge_rrt,
                                                    checker,
                                                    context,
                                                    bridge_waypoint_length,
                                                    audited_bridge_length,
                                                    bridge_edge_query_index,
                                                    direct_segment_after_rrt_candidate);
    if (direct_segment_edges_added > 0) {
        return finish_bridge(direct_segment_edges_added);
    }
    const double direct_corridor_audit_step = config_.query.audit_segment_step > 0.0
        ? config_.query.audit_segment_step
        : 0.01;
    const QueryBridgeDirectCorridorRuntimeOptions direct_corridor_options =
        query_bridge_direct_corridor_runtime_options(query_index,
                                                     direct_corridor_audit_step);
    const double dense_box_corridor_max_length = direct_corridor_options.max_length;
    const bool dense_box_corridor_candidate =
        defer_query_segment_edge &&
        audited_bridge_length > 0.0 &&
        audited_bridge_length <= dense_box_corridor_max_length;
    if (dense_box_corridor_candidate) {
        const int direct_corridor_added = try_query_bridge_direct_ffb_corridor(
            start,
            goal,
            corridor_path,
            bridge_rrt,
            checker,
            context,
            query_index,
            bridge_edge_query_index,
            query_bridge_ffb_depth,
            audited_bridge_length,
            allow_residual_segments,
            next_id);
        if (direct_corridor_added > 0) {
            return direct_corridor_added;
        }
        if (partition_native_mode()) {
            skip_graph_query_bridge_pave_if_partition_native(
                context,
                "query_bridge.partition_graph_dense_chain_pave_skipped");
            dense_repair_attempted = true;
        }
    }
    if (dense_box_corridor_candidate && !partition_native_mode()) {
        dense_repair_attempted = true;
        ChainPaveConfig dense_config =
            make_dense_query_bridge_pave_config(config_.connector.pave,
                                                query_bridge_ffb_depth);
        dense_repair_added = run_query_bridge_chain_pave(
            corridor_path,
            start_box_id,
            next_id,
            context,
            dense_config,
            "query_bridge.dense_boundary_pave");
        auto [source_box_id, target_box_id] =
            run_query_bridge_reverse_boundary_pave(start,
                                                   goal,
                                                   corridor_path,
                                                   dense_config,
                                                   dense_repair_added,
                                                   dense_repair_added,
                                                   next_id,
                                                   context);
        const int maybe_box_corridor_edges_added =
            try_add_query_box_corridor_edge(source_box_id,
                                            target_box_id,
                                            corridor_path,
                                            bridge_rrt.segment_resolution,
                                            bridge_edge_query_index);
        if (maybe_box_corridor_edges_added >= 0) {
            box_corridor_edges_added = maybe_box_corridor_edges_added;
            return finish_bridge(dense_repair_added + box_corridor_edges_added);
        }
    }
    ChainPaveConfig pave_config =
        defer_query_segment_edge
            ? make_deferred_query_bridge_pave_config(config_.connector.pave,
                                                     query_bridge_ffb_depth,
                                                     short_local_bridge)
            : config_.connector.pave;
    int added = 0;
    if (!skip_graph_query_bridge_pave_if_partition_native(
            context,
            "query_bridge.partition_graph_forward_chain_pave_skipped")) {
        added = run_query_bridge_chain_pave(
            corridor_path,
            start_box_id,
            next_id,
            context,
            pave_config,
            "query_bridge.forward_boundary_pave");
    }
    auto [source_box_id, target_box_id] =
        run_query_bridge_reverse_boundary_pave(start,
                                               goal,
                                               corridor_path,
                                               pave_config,
                                               added,
                                               added,
                                               next_id,
                                               context);
    if (added > 0) {
        const int maybe_box_corridor_edges_added =
            try_add_query_box_corridor_edge(source_box_id,
                                            target_box_id,
                                            corridor_path,
                                            bridge_rrt.segment_resolution,
                                            bridge_edge_query_index);
        if (maybe_box_corridor_edges_added >= 0) {
            box_corridor_edges_added = maybe_box_corridor_edges_added;
            return finish_bridge(added + box_corridor_edges_added);
        }
    }
    if (dense_box_corridor_candidate && !dense_repair_attempted && !partition_native_mode()) {
        ChainPaveConfig dense_config =
            make_dense_query_bridge_pave_config(config_.connector.pave,
                                                query_bridge_ffb_depth);
        dense_repair_added = run_query_bridge_chain_pave(
            corridor_path,
            start_box_id,
            next_id,
            context,
            dense_config,
            "query_bridge.dense_boundary_retry");
        std::tie(source_box_id, target_box_id) =
            run_query_bridge_reverse_boundary_pave(start,
                                                   goal,
                                                   corridor_path,
                                                   dense_config,
                                                   dense_repair_added,
                                                   dense_repair_added,
                                                   next_id,
                                                   context);
        const int maybe_box_corridor_edges_added =
            try_add_query_box_corridor_edge(source_box_id,
                                            target_box_id,
                                            corridor_path,
                                            bridge_rrt.segment_resolution,
                                            bridge_edge_query_index);
        if (maybe_box_corridor_edges_added >= 0) {
            box_corridor_edges_added = maybe_box_corridor_edges_added;
            return finish_bridge(added + dense_repair_added + box_corridor_edges_added);
        }
    }
    if (!skip_graph_query_bridge_pave_if_partition_native(
            context,
            "query_bridge.partition_graph_gap_connector_skipped")) {
        IslandConnectorConfig gap_config = config_.connector;
        gap_config.max_total_bridge_boxes = 0;
        IslandConnector gap_connector(*oracle_, robot_, checker, gap_config);
        const auto gap_result = gap_connector.connect_all(boxes_, adjacency_, segment_edges_, next_id, context);
        (void)gap_result;
        invalidate_query_cache();
    }
    source_box_id = locate_box_partition_first(start, config_.query.nearest_if_outside);
    target_box_id = locate_box_partition_first(goal, config_.query.nearest_if_outside);
    if (added > 0) {
        const int maybe_box_corridor_edges_added =
            try_add_query_box_corridor_edge(source_box_id,
                                            target_box_id,
                                            corridor_path,
                                            bridge_rrt.segment_resolution,
                                            bridge_edge_query_index);
        if (maybe_box_corridor_edges_added >= 0) {
            box_corridor_edges_added = maybe_box_corridor_edges_added;
            return finish_bridge(added + box_corridor_edges_added);
        }
    }
    direct_segment_edges_added =
        try_add_query_residual_segment_edge(source_box_id,
                                            target_box_id,
                                            corridor_path,
                                            bridge_rrt,
                                            checker,
                                            context,
                                            query_bridge_depth_failures_before,
                                            bridge_edge_query_index,
                                            defer_query_segment_edge);
    return finish_bridge(added + dense_repair_added + box_corridor_edges_added + direct_segment_edges_added);
}

} // namespace rbf
