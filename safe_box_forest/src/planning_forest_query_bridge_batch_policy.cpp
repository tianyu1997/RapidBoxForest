#include "planning_forest_query_bridge_policy.h"

#include <algorithm>

namespace rbf {

bool query_bridge_should_check_current_query(
    const QueryBridgeSearchTask& task,
    bool respect_forced,
    const QueryBridgeIndexOptions& index_options) {
    if (query_bridge_index_segment_only(index_options, task.index)) {
        return false;
    }
    if (respect_forced && query_bridge_index_forced(index_options, task.index)) {
        return false;
    }
    return true;
}

bool query_bridge_has_segment_only_task(
    const std::vector<QueryBridgeSearchTask>& tasks,
    const QueryBridgeIndexOptions& index_options) {
    return std::any_of(tasks.begin(), tasks.end(), [&](const QueryBridgeSearchTask& task) {
        return query_bridge_index_segment_only(index_options, task.index);
    });
}

bool query_bridge_parallel_task_rrt_enabled(
    const QueryBridgeBatchExecutionOptions& batch_options,
    bool has_segment_only_task,
    const QueryBridgeRetryOptions& retry_options) {
    return batch_options.parallel_task_rrt &&
           !has_segment_only_task &&
           retry_options.no_path_retry_attempts == 0 &&
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
    const QueryBridgeIndexOptions& index_options,
    const QueryBridgeRetryOptions& retry_options,
    StageContext& context) {
    QueryBridgeAttemptPlan plan =
        query_bridge_attempt_plan(task,
                                  query_bridge_index_forced(index_options, task.index),
                                  retry_options);
    record_query_bridge_forced_attempts(context,
                                        task.index,
                                        plan.forced,
                                        plan.effective_attempts);
    return plan;
}

}  // namespace rbf
