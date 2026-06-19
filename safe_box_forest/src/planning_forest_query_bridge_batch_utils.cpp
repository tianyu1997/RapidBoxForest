#include "planning_forest_query_bridge_batch_utils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace rbf {

std::string query_bridge_task_key(std::size_t index, const std::string& suffix) {
    return "query_bridge.batch_task." + std::to_string(index) + "." + suffix;
}

bool query_bridge_should_check_current_query(
    const QueryBridgeSearchTask& task,
    bool respect_forced,
    const QueryBridgeIndexOptions& index_options,
    const QueryBridgeRetryOptions& retry_options) {
    if (query_bridge_index_segment_only(index_options, task.index)) {
        return false;
    }
    if (respect_forced && query_bridge_index_forced(index_options, task.index)) {
        return false;
    }
    return retry_options.skip_deferred_short_edges;
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
    if (plan.partition_path_first) {
        record_query_bridge_partition_path_first_task(context, task.index);
    }
    record_query_bridge_forced_attempts(context,
                                        task.index,
                                        plan.forced,
                                        plan.effective_attempts);
    return plan;
}

QueryBridgePartitionInitialPathDecision query_bridge_partition_initial_path_decision(
    const QueryResult& initial_query,
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal,
    const QueryBridgeAcceptanceThresholds& thresholds,
    const QueryBridgePartitionPathFirstOptions& options) {
    QueryBridgePartitionInitialPathDecision decision;
    decision.direct_distance = (goal - start).norm();
    decision.raw_length =
        initial_query.raw_path_length > 1e-12
            ? initial_query.raw_path_length
            : initial_query.path_length;
    decision.segment_fraction =
        decision.raw_length > 1e-12
            ? initial_query.segment_edge_length / decision.raw_length
            : std::numeric_limits<double>::infinity();
    decision.segment_reasonable =
        std::isfinite(decision.segment_fraction) &&
        decision.segment_fraction <= options.max_segment_fraction;
    decision.length_reasonable =
        decision.direct_distance <= 1e-9 ||
        initial_query.path_length <=
            std::max(decision.direct_distance * thresholds.path_ratio,
                     decision.direct_distance + thresholds.path_additive) ||
        initial_query.path_length <= thresholds.max_path_length;
    decision.accepted =
        decision.segment_reasonable &&
        (decision.length_reasonable || options.allow_long);
    return decision;
}

}  // namespace rbf
