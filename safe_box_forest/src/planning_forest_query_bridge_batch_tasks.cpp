#include <SBF/safe_box_forest.h>

#include <SBF/oracle.h>

#include "planning_forest_query_bridge_diagnostics.h"
#include "planning_forest_query_bridge_options.h"
#include "planning_forest_query_bridge_policy.h"
#include "planning_forest_query_bridge_rrt_utils.h"
#include "planning_forest_query_bridge_task.h"
#include "planning_forest_query_utils.h"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace rbf {

std::vector<QueryBridgeSearchTask> RBFPlanningForest::prepare_query_bridge_batch_tasks(
    const std::vector<Eigen::VectorXd>& starts,
    const std::vector<Eigen::VectorXd>& goals,
    const QueryBridgeBatchOptions& options,
    const std::unordered_set<int>& forced_query_indices,
    const QueryBridgeAcceptanceThresholds& bridge_acceptance,
    std::size_t& partition_refresh_base) {
    std::vector<QueryBridgeSearchTask> tasks;
    tasks.reserve(starts.size());
    for (std::size_t index = 0; index < starts.size(); ++index) {
        if (starts[index].size() != goals[index].size()) {
            throw std::invalid_argument(
                "bridge_queries received a start/goal dimension mismatch");
        }
        const bool forced_task =
            forced_query_indices.find(static_cast<int>(index)) !=
            forced_query_indices.end();
        QueryResult initial_query;
        bool has_initial_query = false;
        if (!forced_task) {
            initial_query = query(starts[index], goals[index]);
            has_initial_query = true;
            if (query_bridge_result_acceptable(initial_query,
                                               starts[index],
                                               goals[index],
                                               bridge_acceptance)) {
                query_bridge_mark_task_skip(last_build_, index, 1.0, "initial_good");
                continue;
            }
        }
        int start_box_id = locate_query_bridge_box(starts[index]);
        if (start_box_id < 0) {
            start_box_id = anchor_query_endpoint_box_with_diagnostics(starts[index]);
        }
        if (start_box_id < 0) {
            query_bridge_mark_task_skip(last_build_, index, 2.0, "start_anchor_failed");
            continue;
        }
        int goal_box_id = locate_query_bridge_box(goals[index]);
        if (goal_box_id < 0) {
            goal_box_id = anchor_query_endpoint_box_with_diagnostics(goals[index]);
        }
        sync_query_bridge_partition_boxes(partition_refresh_base,
                                          "query_bridge.endpoint_anchor");
        if (start_box_id >= 0) {
            start_box_id = refresh_query_bridge_box_or_anchor(start_box_id,
                                                              starts[index],
                                                              "start");
        }
        if (goal_box_id >= 0) {
            goal_box_id = refresh_query_bridge_box_or_anchor(goal_box_id,
                                                             goals[index],
                                                             "goal");
        }
        if (goal_box_id < 0 || goal_box_id == start_box_id) {
            query_bridge_mark_task_skip(
                last_build_,
                index,
                goal_box_id < 0 ? 3.0 : 4.0,
                goal_box_id < 0 ? "goal_anchor_failed" : "same_box");
            continue;
        }

        QueryBridgeSearchTask task;
        task.index = index;
        task.query_index = query_bridge_batch_global_query_index(options, index);
        last_build_.diagnostics["query_bridge.batch_task." +
                                std::to_string(index) +
                                ".global_index"] =
            static_cast<double>(task.query_index);
        task.start = starts[index];
        task.goal = goals[index];
        if (last_adaptive_partition_config_.hipac_online_connectivity &&
            has_initial_query &&
            initial_query.success &&
            initial_query.audit_passed &&
            !initial_query.path.empty()) {
            task.hipac_candidate_path = initial_query.path;
        }
        task.bridge_rrt = with_query_root_hull_domain(config_.connector.rrt,
                                                      *oracle_,
                                                      task.start,
                                                      task.goal);
        task.bridge_rrt.segment_resolution =
            std::max(task.bridge_rrt.segment_resolution,
                     config_.query.audit_resolution);
        const double bridge_distance = (task.goal - task.start).norm();
        task.short_local_bridge = query_bridge_short_local_distance(bridge_distance);
        if (task.short_local_bridge) {
            query_bridge_configure_short_local_profiles(task.bridge_rrt,
                                                        task.short_local_profiles);
        }
        task.attempts = std::max(1, config_.connector.max_pairs_per_gap);
        tasks.push_back(std::move(task));
    }

    std::stable_sort(tasks.begin(),
                     tasks.end(),
                     [](const QueryBridgeSearchTask& lhs,
                        const QueryBridgeSearchTask& rhs) {
        const bool lhs_short = lhs.short_local_bridge;
        const bool rhs_short = rhs.short_local_bridge;
        if (lhs_short != rhs_short) {
            return !lhs_short && rhs_short;
        }
        return lhs.index < rhs.index;
    });
    return tasks;
}

}  // namespace rbf
