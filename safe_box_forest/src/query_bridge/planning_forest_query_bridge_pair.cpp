#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>
#include <SBF/connector.h>

#include "planning_forest_audit.h"
#include "planning_forest_diagnostics.h"
#include "planning_forest_qroot_helpers.h"
#include "planning_forest_query_bridge_corridor_options.h"
#include "planning_forest_query_bridge_rrt_utils.h"
#include "planning_forest_query_utils.h"
#include "virtual_sparse_ffb.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rbf {

int RBFPlanningForest::bridge_query(const Eigen::Ref<const Eigen::VectorXd>& start,
                                const Eigen::Ref<const Eigen::VectorXd>& goal) {
    if (boxes_.empty() || !oracle_) {
        return 0;
    }
    QueryResult current = query(start, goal);
    if (current.success && current.repair_count == 0) {
        const double direct = (goal - start).norm();
        const bool graph_only = current.segment_edges_used == 0;
        const bool short_enough =
            direct <= 1e-9 ||
            current.path_length <= std::max(direct * 1.35, direct + 0.35);
        if (graph_only && short_enough) {
            return 0;
        }
    }
    return bridge_query_known_needed(start, goal);
}

int RBFPlanningForest::bridge_query_known_needed(const Eigen::Ref<const Eigen::VectorXd>& start,
                                             const Eigen::Ref<const Eigen::VectorXd>& goal) {
    if (boxes_.empty() || !oracle_) {
        return 0;
    }
    StageContext context = StageContext::from_runtime(config_.runtime);
    ScopedStageDiagnosticsFlush diagnostics_flush(last_build_, context);
    int start_box_id = locate_box_partition_first(start, config_.query.nearest_if_outside);
    if (start_box_id < 0) {
        start_box_id = anchor_query_endpoint_box(start, context);
    }
    if (start_box_id < 0) {
        return 0;
    }
    int goal_box_id = locate_box_partition_first(goal, config_.query.nearest_if_outside);
    if (goal_box_id < 0) {
        goal_box_id = anchor_query_endpoint_box(goal, context);
    }
    if (goal_box_id < 0 || goal_box_id == start_box_id) {
        return 0;
    }
    CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
    const QueryBridgeEdgeRuntimeOptions edge_options =
        query_bridge_edge_runtime_options_from_config(config_);
    record_query_bridge_edge_runtime_diagnostics(context, edge_options);
    if (edge_options.direct_start_goal_segment_enabled) {
        const int added = try_add_query_direct_start_goal_segment_edge(start_box_id,
                                                                       goal_box_id,
                                                                       start,
                                                                       goal,
                                                                       context,
                                                                       -1);
        if (added > 0) {
            return added;
        }
    }
    RRTConnectConfig bridge_rrt = with_query_root_hull_domain(config_.connector.rrt, *oracle_, start, goal);
    bridge_rrt.segment_resolution = std::max(bridge_rrt.segment_resolution, config_.query.audit_resolution);
    const double bridge_distance = (goal - start).norm();
    const bool short_local_bridge = query_bridge_short_local_distance(bridge_distance);
    std::vector<RRTConnectConfig> short_local_profiles;
    if (short_local_bridge) {
        query_bridge_configure_short_local_profiles(bridge_rrt, short_local_profiles);
        context.diagnostics().add_counter("query_bridge.short_local_profile");
        context.diagnostics().set_value("query_bridge.short_local_step_size",
                                        bridge_rrt.step_size);
        context.diagnostics().set_value("query_bridge.short_local_goal_bias",
                                        bridge_rrt.goal_bias);
        context.diagnostics().set_value("query_bridge.short_local_radius",
                                        bridge_rrt.local_sampling_radius);
        context.diagnostics().set_value("query_bridge.short_local_profiles",
                                        static_cast<double>(short_local_profiles.size()));
    }
    const int bridge_attempts =
        std::max(1, config_.connector.max_pairs_per_gap);
    const int run_seed = config_.grower.rng_seed;
    const int bridge_seed_base = derived_planner_seed(run_seed, kSeedQueryBridgeOffset);
    context.diagnostics().set_value("query_bridge.run_seed", static_cast<double>(run_seed));
    context.diagnostics().set_value("query_bridge.seed_base", static_cast<double>(bridge_seed_base));
    const QueryBridgeParallelRrtOptions parallel_options =
        query_bridge_parallel_rrt_options_from_config(config_);
    auto waypoint_path = best_audited_rrt_bridge_path(start,
                                                      goal,
                                                      checker,
                                                      audit_robot_,
                                                      context,
                                                      bridge_rrt,
                                                      bridge_attempts,
                                                      config_.connector.per_pair_timeout_ms * bridge_attempts,
                                                      bridge_seed_base,
                                                      config_.query.audit_resolution,
                                                      config_.query.audit_segment_step,
                                                      parallel_options,
                                                      short_local_profiles.empty() ? nullptr : &short_local_profiles,
                                                      short_local_bridge ? 1 : 7919);
    if (waypoint_path.empty()) {
        return 0;
    }
    return bridge_query_with_waypoint_path(start,
                                           goal,
                                           waypoint_path,
                                           short_local_bridge,
                                           bridge_rrt);
}

} // namespace rbf
