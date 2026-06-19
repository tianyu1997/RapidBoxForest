#include "planning_forest_query_bridge_attempt_paths.h"

#include "planning_forest_audit.h"
#include "planning_forest_query_bridge_diagnostics.h"
#include "planning_forest_query_utils.h"

#include <SBF/box_graph.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace rbf {
namespace {

void select_query_bridge_attempt_paths(
    QueryBridgeSearchTask& task,
    std::vector<std::vector<Eigen::VectorXd>>& attempt_paths,
    double& best_length,
    const QueryBridgeHybridizeAttemptOptions& hybrid_options,
    const Robot& audit_robot,
    const Scene& scene,
    const RBFPlanningConfig& config,
    StageContext& context) {
    std::vector<std::pair<double, std::size_t>> valid_paths;
    valid_paths.reserve(attempt_paths.size());
    for (std::size_t index = 0; index < attempt_paths.size(); ++index) {
        if (attempt_paths[index].empty()) {
            continue;
        }
        const double length = path_length(attempt_paths[index]);
        if (!std::isfinite(length)) {
            continue;
        }
        valid_paths.emplace_back(length, index);
    }
    if (valid_paths.empty()) {
        return;
    }
    if (hybrid_options.enabled && valid_paths.size() >= 2U) {
        CollisionChecker checker = make_audit_checker(audit_robot, scene, config.query);
        const double best_input_length =
            std::min(best_length,
                     std::min_element(valid_paths.begin(),
                                      valid_paths.end(),
                                      [](const auto& lhs, const auto& rhs) {
                                          return lhs.first < rhs.first;
                                      })
                         ->first);
        std::vector<Eigen::VectorXd> hybrid =
            hybridize_collision_free_paths(attempt_paths,
                                           checker,
                                           collision_shortcut_resolution(config.query),
                                           hybrid_options.max_paths,
                                           hybrid_options.max_vertices,
                                           hybrid_options.max_cross_checks);
        context.diagnostics().add_counter(
            "query_bridge.hybridize_attempt_paths_tasks");
        if (!hybrid.empty()) {
            const double hybrid_length = path_length(hybrid);
            context.diagnostics().add_counter(
                "query_bridge.hybridize_attempt_paths_candidates");
            context.diagnostics().add_counter(
                query_bridge_task_key(task.index,
                                      "hybridize_attempt_paths_candidates"));
            if (hybrid_length + 1e-12 < best_input_length) {
                const PathAuditCheck audit =
                    audit_waypoint_path(hybrid,
                                        checker,
                                        config.query.audit_resolution,
                                        config.query.audit_segment_step);
                if (audit.passed) {
                    const std::size_t index = attempt_paths.size();
                    attempt_paths.push_back(std::move(hybrid));
                    valid_paths.emplace_back(hybrid_length, index);
                    context.diagnostics().add_counter(
                        "query_bridge.hybridize_attempt_paths_accepts");
                    context.diagnostics().add_counter(
                        "query_bridge.hybridize_attempt_paths_delta",
                        best_input_length - hybrid_length);
                    context.diagnostics().add_counter(
                        query_bridge_task_key(task.index,
                                              "hybridize_attempt_paths_accepts"));
                } else {
                    context.diagnostics().add_counter(
                        "query_bridge.hybridize_attempt_paths_audit_rejects");
                }
            }
        }
    }
    std::sort(valid_paths.begin(), valid_paths.end(),
              [](const auto& lhs, const auto& rhs) {
                  if (std::abs(lhs.first - rhs.first) > 1e-12) {
                      return lhs.first < rhs.first;
                  }
                  return lhs.second < rhs.second;
              });
    if (task.waypoint_path.empty() || valid_paths.front().first < best_length) {
        best_length = valid_paths.front().first;
        task.waypoint_path = std::move(attempt_paths[valid_paths.front().second]);
    }
}

}  // namespace

void adopt_query_bridge_waypoint_after_rrt(
    QueryBridgeSearchTask& task,
    std::vector<std::vector<Eigen::VectorXd>>& attempt_paths_for_task,
    double& best_length,
    const QueryBridgeHybridizeAttemptOptions& hybrid_options,
    const Robot& audit_robot,
    const Scene& scene,
    const RBFPlanningConfig& config,
    StageContext& context) {
    select_query_bridge_attempt_paths(task,
                                      attempt_paths_for_task,
                                      best_length,
                                      hybrid_options,
                                      audit_robot,
                                      scene,
                                      config,
                                      context);
}

}  // namespace rbf
