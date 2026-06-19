#include "planning_forest_query_bridge_path_utils.h"

#include "planning_forest_audit.h"
#include "planning_forest_query_bridge_corridor_graph.h"
#include "planning_forest_query_utils.h"

#include <algorithm>
#include <chrono>
#include <string>

namespace rbf {

void query_bridge_set_task_value(StageContext& context,
                                 int query_index,
                                 const std::string& suffix,
                                 double value) {
    if (query_index < 0) {
        return;
    }
    context.diagnostics().set_value(
        "query_bridge.batch_task." + std::to_string(query_index) + "." + suffix,
        value);
}

void query_bridge_apply_waypoint_shortcut(
    std::vector<Eigen::VectorXd>& corridor_path,
    const CollisionChecker& checker,
    const QueryConfig& query_config,
    const QueryBridgeWaypointShortcutOptions& options,
    StageContext& context,
    int query_index) {
    context.diagnostics().set_value("query_bridge.waypoint_shortcut_enabled",
                                    options.enabled ? 1.0 : 0.0);
    if (!options.enabled || corridor_path.size() <= 2) {
        return;
    }

    using Clock = std::chrono::steady_clock;
    const auto shortcut_t0 = Clock::now();
    const double before_length = query_bridge_waypoint_length(corridor_path);
    std::vector<Eigen::VectorXd> shortened =
        collision_shortcut_path(corridor_path,
                                checker,
                                collision_shortcut_resolution(query_config));
    const double after_length = query_bridge_waypoint_length(shortened);
    context.diagnostics().add_counter("query_bridge.waypoint_shortcut_attempts");
    query_bridge_set_task_value(context,
                                query_index,
                                "waypoint_shortcut_before_length",
                                before_length);
    query_bridge_set_task_value(context,
                                query_index,
                                "waypoint_shortcut_after_length",
                                after_length);
    if (!shortened.empty() &&
        after_length + options.min_gain < before_length) {
        const PathAuditCheck shortcut_audit =
            audit_waypoint_path(shortened,
                                checker,
                                query_config.audit_resolution,
                                query_config.audit_segment_step);
        if (shortcut_audit.passed) {
            context.diagnostics().add_counter("query_bridge.waypoint_shortcut_accepts");
            context.diagnostics().add_counter("query_bridge.waypoint_shortcut_delta",
                                              before_length - after_length);
            query_bridge_set_task_value(context,
                                        query_index,
                                        "waypoint_shortcut_accepted",
                                        1.0);
            query_bridge_set_task_value(context,
                                        query_index,
                                        "waypoint_shortcut_delta",
                                        before_length - after_length);
            corridor_path = std::move(shortened);
        } else {
            context.diagnostics().add_counter("query_bridge.waypoint_shortcut_audit_rejects");
            query_bridge_set_task_value(context,
                                        query_index,
                                        "waypoint_shortcut_audit_reject",
                                        1.0);
        }
    }
    context.diagnostics().record_timing(
        "query_bridge.waypoint_shortcut_ms_total",
        std::chrono::duration<double, std::milli>(Clock::now() - shortcut_t0).count());
}

void query_bridge_apply_internal_simplify(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    std::vector<Eigen::VectorXd>& corridor_path,
    const CollisionChecker& checker,
    const Robot& audit_robot,
    const RRTConnectConfig& connector_rrt,
    const QueryConfig& query_config,
    int rng_seed,
    bool enabled) {
    if (!enabled ||
        !query_config.final_rrt_simplify ||
        query_config.final_rrt_simplify_timeout_ms <= 0.0 ||
        corridor_path.size() < 2) {
        return;
    }

    using Clock = std::chrono::steady_clock;
    const auto simplify_t0 = Clock::now();
    auto elapsed_ms = [&]() {
        return std::chrono::duration<double, std::milli>(Clock::now() - simplify_t0).count();
    };
    RRTConnectConfig simplify_config = connector_rrt;
    simplify_config.max_iters = std::max(1, query_config.final_rrt_simplify_max_iters);
    simplify_config.segment_resolution =
        std::max(simplify_config.segment_resolution, query_config.audit_resolution);
    simplify_config.segment_step = query_config.audit_segment_step;
    simplify_config.shortcut_path = true;
    const int attempts = std::max(1, query_config.final_rrt_simplify_attempts);
    double best_length = query_bridge_waypoint_length(corridor_path);
    for (int attempt = 0; attempt < attempts; ++attempt) {
        const double remaining_ms = query_config.final_rrt_simplify_timeout_ms - elapsed_ms();
        if (remaining_ms <= 0.0) {
            break;
        }
        const int attempts_left = attempts - attempt;
        simplify_config.timeout_ms =
            std::max(1.0, remaining_ms / static_cast<double>(attempts_left));
        const int simplify_seed =
            derived_planner_seed(rng_seed, kSeedBridgeSimplifyOffset, attempt);
        auto candidate = rrt_connect(start,
                                     goal,
                                     checker,
                                     audit_robot,
                                     simplify_config,
                                     simplify_seed);
        if (candidate.empty()) {
            continue;
        }
        const double candidate_length = query_bridge_waypoint_length(candidate);
        if (candidate_length + 1e-12 >= best_length) {
            continue;
        }
        if (audit_waypoint_path(candidate,
                                checker,
                                query_config.audit_resolution,
                                query_config.audit_segment_step)
                .passed) {
            best_length = candidate_length;
            corridor_path = std::move(candidate);
        }
    }
}

} // namespace rbf
