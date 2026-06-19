#include <SBF/safe_box_forest.h>

#include "planning_forest_audit.h"
#include "planning_forest_query_bridge_diagnostics.h"
#include "planning_forest_query_bridge_policy.h"
#include "planning_forest_query_bridge_task.h"
#include "planning_forest_query_utils.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace rbf {

int RBFPlanningForest::try_add_query_direct_start_goal_segment_edge(
    int source_box_id,
    int target_box_id,
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    StageContext& context,
    int query_index,
    int batch_task_index) {
    const auto add_task_counter = [&](const std::string& suffix) {
        if (batch_task_index >= 0) {
            context.diagnostics().add_counter(
                query_bridge_task_key(static_cast<std::size_t>(batch_task_index), suffix));
        }
    };
    if (source_box_id < 0 || target_box_id < 0 || source_box_id == target_box_id) {
        context.diagnostics().add_counter(
            "query_bridge.direct_start_goal_segment_missing_endpoint");
        return 0;
    }
    std::vector<Eigen::VectorXd> direct_path{start, goal};
    context.diagnostics().add_counter(
        "query_bridge.direct_start_goal_segment_attempts");
    add_task_counter("direct_start_goal_segment_attempts");
    CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
    const PathAuditCheck audit =
        audit_waypoint_path(direct_path,
                            checker,
                            config_.query.audit_resolution,
                            config_.query.audit_segment_step);
    if (!audit.passed) {
        context.diagnostics().add_counter(
            "query_bridge.direct_start_goal_segment_audit_rejects");
        add_task_counter("direct_start_goal_segment_audit_rejects");
        return 0;
    }
    const int edge_id = add_segment_edge_partition_first(
        source_box_id,
        target_box_id,
        direct_path,
        SegmentEdgeType::QueryBridge,
        config_.query.audit_resolution,
        SegmentEdgeValidation::CollisionChecked,
        true,
        query_index);
    if (edge_id < 0) {
        context.diagnostics().add_counter(
            "query_bridge.direct_start_goal_segment_add_fail");
        add_task_counter("direct_start_goal_segment_add_fail");
        return 0;
    }
    context.diagnostics().add_counter(
        "query_bridge.direct_start_goal_segment_edges");
    add_task_counter("direct_start_goal_segment_edges");
    invalidate_query_cache();
    sync_adaptive_partition_segment_edges(&last_build_,
                                          "query_bridge.direct_start_goal_segment");
    refresh_adaptive_partition_diagnostics(&last_build_);
    return 1;
}

int RBFPlanningForest::try_add_query_direct_start_goal_segment_for_points(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    StageContext& context,
    int query_index,
    int batch_task_index) {
    const int source_box_id = locate_query_bridge_box(start);
    const int target_box_id = locate_query_bridge_box(goal);
    return try_add_query_direct_start_goal_segment_edge(source_box_id,
                                                       target_box_id,
                                                       start,
                                                       goal,
                                                       context,
                                                       query_index,
                                                       batch_task_index);
}

void RBFPlanningForest::run_query_bridge_direct_start_goal_segments(
    std::vector<QueryBridgeSearchTask>& tasks,
    std::vector<int>& added_by_query,
    StageContext& context,
    bool scene_reusable_edges) {
    for (auto& task : tasks) {
        if (task.direct_start_goal_satisfied) {
            continue;
        }
        const int added = try_add_query_direct_start_goal_segment_for_points(
            task.start,
            task.goal,
            context,
            query_bridge_edge_query_index(scene_reusable_edges, task),
            static_cast<int>(task.index));
        task.direct_start_goal_satisfied = added > 0;
        if (added > 0) {
            added_by_query[task.index] += added;
        }
    }
}

int RBFPlanningForest::try_add_query_fast_direct_segment_after_rrt_edge(
    int source_box_id,
    int target_box_id,
    const std::vector<std::vector<Eigen::VectorXd>>& candidate_paths,
    const RRTConnectConfig& bridge_rrt,
    StageContext& context,
    double min_length,
    int query_index,
    int batch_task_index) {
    const auto add_task_counter = [&](const std::string& suffix, double value = 1.0) {
        if (batch_task_index >= 0) {
            context.diagnostics().add_counter(
                query_bridge_task_key(static_cast<std::size_t>(batch_task_index), suffix),
                value);
        }
    };
    const auto set_task_value = [&](const std::string& suffix, double value) {
        if (batch_task_index >= 0) {
            context.diagnostics().set_value(
                query_bridge_task_key(static_cast<std::size_t>(batch_task_index), suffix),
                value);
        }
    };
    if (candidate_paths.empty() ||
        !(path_length(candidate_paths.front()) >= min_length)) {
        context.diagnostics().add_counter(
            "query_bridge.fast_direct_segment_after_rrt_length_rejects");
        add_task_counter("fast_direct_segment_after_rrt_length_rejects");
        return 0;
    }
    if (source_box_id < 0 || target_box_id < 0 || source_box_id == target_box_id) {
        context.diagnostics().add_counter(
            "query_bridge.fast_direct_segment_after_rrt_missing_endpoint");
        add_task_counter("fast_direct_segment_after_rrt_missing_endpoint");
        return 0;
    }
    CollisionChecker strict_checker = make_audit_checker(audit_robot_, scene_, config_.query);
    int edge_id = -1;
    double added_length = std::numeric_limits<double>::infinity();
    for (std::size_t candidate_index = 0;
         candidate_index < candidate_paths.size();
         ++candidate_index) {
        const auto& candidate_path = candidate_paths[candidate_index];
        if (path_length(candidate_path) + 1e-12 < min_length) {
            continue;
        }
        context.diagnostics().add_counter(
            "query_bridge.fast_direct_segment_after_rrt_add_candidates");
        const PathAuditCheck candidate_audit =
            audit_waypoint_path(candidate_path,
                                strict_checker,
                                config_.query.audit_resolution,
                                config_.query.audit_segment_step);
        if (!candidate_audit.passed) {
            context.diagnostics().add_counter(
                "query_bridge.fast_direct_segment_after_rrt_candidate_audit_rejects");
            add_task_counter("fast_direct_segment_after_rrt_candidate_audit_rejects");
            continue;
        }
        edge_id = add_segment_edge_partition_first(
            source_box_id,
            target_box_id,
            candidate_path,
            SegmentEdgeType::QueryBridge,
            bridge_rrt.segment_resolution,
            SegmentEdgeValidation::CollisionChecked,
            true,
            query_index);
        if (edge_id >= 0) {
            added_length = path_length(candidate_path);
            if (candidate_index > 0) {
                context.diagnostics().add_counter(
                    "query_bridge.fast_direct_segment_after_rrt_fallback_candidate_success");
            }
            break;
        }
        context.diagnostics().add_counter(
            "query_bridge.fast_direct_segment_after_rrt_add_candidate_fail");
    }
    if (edge_id < 0) {
        context.diagnostics().add_counter(
            "query_bridge.fast_direct_segment_after_rrt_add_fail");
        add_task_counter("fast_direct_segment_after_rrt_add_fail");
        return 0;
    }
    invalidate_query_cache();
    context.diagnostics().add_counter(
        "query_bridge.fast_direct_segment_after_rrt_edges");
    add_task_counter("fast_direct_segment_after_rrt_edges");
    set_task_value("fast_direct_segment_after_rrt_length", added_length);
    return 1;
}

int RBFPlanningForest::try_add_query_fast_direct_segment_after_rrt_path(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    const std::vector<Eigen::VectorXd>& waypoint_path,
    const RRTConnectConfig& bridge_rrt,
    StageContext& context,
    bool enabled,
    int random_shortcut_iters,
    double min_length,
    int shortcut_query_index,
    int edge_query_index,
    int batch_task_index) {
    if (!enabled || waypoint_path.empty()) {
        return 0;
    }
    const auto add_task_counter = [&](const std::string& suffix, double value = 1.0) {
        if (batch_task_index >= 0) {
            context.diagnostics().add_counter(
                query_bridge_task_key(static_cast<std::size_t>(batch_task_index), suffix),
                value);
        }
    };

    std::vector<std::vector<Eigen::VectorXd>> candidate_paths;
    candidate_paths.push_back(waypoint_path);
    if (waypoint_path.size() > 2U) {
        const double before_length = path_length(waypoint_path);
        CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
        std::vector<Eigen::VectorXd> shortened =
            collision_shortcut_path(waypoint_path,
                                    checker,
                                    collision_shortcut_resolution(config_.query));
        const double after_length = path_length(shortened);
        context.diagnostics().add_counter(
            "query_bridge.fast_direct_segment_after_rrt_shortcut_attempts");
        add_task_counter("fast_direct_segment_after_rrt_shortcut_attempts");
        if (!shortened.empty() && after_length + 1e-12 < before_length) {
            candidate_paths.push_back(std::move(shortened));
            context.diagnostics().add_counter(
                "query_bridge.fast_direct_segment_after_rrt_shortcut_accepts");
            context.diagnostics().add_counter(
                "query_bridge.fast_direct_segment_after_rrt_shortcut_delta",
                before_length - after_length);
            add_task_counter("fast_direct_segment_after_rrt_shortcut_accepts");
            add_task_counter("fast_direct_segment_after_rrt_shortcut_delta",
                             before_length - after_length);
        }
        const auto& random_source = candidate_paths.back();
        if (random_shortcut_iters > 0 && random_source.size() > 2U) {
            const double random_before_length = path_length(random_source);
            const std::uint32_t shortcut_seed =
                static_cast<std::uint32_t>(
                    0x9e3779b9U ^
                    ((static_cast<std::uint32_t>(shortcut_query_index) + 1U) *
                     2654435761U) ^
                    (static_cast<std::uint32_t>(batch_task_index + 1) * 2246822519U) ^
                    static_cast<std::uint32_t>(random_source.size()));
            std::vector<Eigen::VectorXd> random_shortened =
                random_collision_shortcut_path(random_source,
                                               checker,
                                               collision_shortcut_resolution(config_.query),
                                               random_shortcut_iters,
                                               shortcut_seed);
            const double random_after_length = path_length(random_shortened);
            context.diagnostics().add_counter(
                "query_bridge.fast_direct_segment_after_rrt_random_shortcut_attempts");
            add_task_counter("fast_direct_segment_after_rrt_random_shortcut_attempts");
            context.diagnostics().add_counter(
                "query_bridge.fast_direct_segment_after_rrt_random_shortcut_iters",
                static_cast<double>(random_shortcut_iters));
            if (!random_shortened.empty() &&
                random_after_length + 1e-12 < random_before_length) {
                candidate_paths.push_back(std::move(random_shortened));
                context.diagnostics().add_counter(
                    "query_bridge.fast_direct_segment_after_rrt_random_shortcut_accepts");
                context.diagnostics().add_counter(
                    "query_bridge.fast_direct_segment_after_rrt_random_shortcut_delta",
                    random_before_length - random_after_length);
                add_task_counter("fast_direct_segment_after_rrt_random_shortcut_accepts");
                add_task_counter("fast_direct_segment_after_rrt_random_shortcut_delta",
                                 random_before_length - random_after_length);
            }
        }
    }
    std::sort(candidate_paths.begin(),
              candidate_paths.end(),
              [](const auto& lhs, const auto& rhs) {
                  return path_length(lhs) < path_length(rhs);
              });
    candidate_paths.erase(std::unique(candidate_paths.begin(),
                                      candidate_paths.end(),
                                      [](const auto& lhs, const auto& rhs) {
                                          if (lhs.size() != rhs.size()) {
                                              return false;
                                          }
                                          for (std::size_t index = 0; index < lhs.size(); ++index) {
                                              if ((lhs[index] - rhs[index]).norm() > 1e-12) {
                                                  return false;
                                              }
                                          }
                                          return true;
                                      }),
                          candidate_paths.end());
    const int source_box_id = locate_query_bridge_box(start);
    const int target_box_id = locate_query_bridge_box(goal);
    return try_add_query_fast_direct_segment_after_rrt_edge(source_box_id,
                                                           target_box_id,
                                                           candidate_paths,
                                                           bridge_rrt,
                                                           context,
                                                           min_length,
                                                           edge_query_index,
                                                           batch_task_index);
}

} // namespace rbf
