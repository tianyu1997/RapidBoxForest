#include "planning_forest_query_bridge_policy.h"

namespace rbf {

bool query_bridge_should_check_current_query(
    const QueryBridgeSearchTask& task,
    bool respect_forced,
    const std::unordered_set<int>& forced_query_indices) {
    if (respect_forced &&
        forced_query_indices.find(static_cast<int>(task.index)) !=
            forced_query_indices.end()) {
        return false;
    }
    return true;
}

bool query_bridge_parallel_task_rrt_enabled(
    const QueryBridgeRetryOptions& retry_options) {
    return retry_options.no_path_retry_attempts == 0 &&
           retry_options.no_path_retry_budget_stages == 0;
}

bool query_bridge_task_has_explicit_satisfaction(
    const QueryBridgeSearchTask& task) {
    return task.hipac_online_satisfied ||
           task.direct_start_goal_satisfied;
}

int query_bridge_edge_query_index(bool scene_reusable_edges,
                                  const QueryBridgeSearchTask& task) {
    return scene_reusable_edges ? -1 : task.query_index;
}

QueryBridgeAttemptPlan query_bridge_prepare_attempt_plan(
    const QueryBridgeSearchTask& task,
    const std::unordered_set<int>& forced_query_indices,
    const QueryBridgeRetryOptions& retry_options,
    StageContext& context) {
    const bool forced =
        forced_query_indices.find(static_cast<int>(task.index)) !=
        forced_query_indices.end();
    QueryBridgeAttemptPlan plan =
        query_bridge_attempt_plan(task, forced, retry_options);
    record_query_bridge_forced_attempts(context,
                                        task.index,
                                        plan.forced,
                                        plan.effective_attempts);
    return plan;
}

}  // namespace rbf
