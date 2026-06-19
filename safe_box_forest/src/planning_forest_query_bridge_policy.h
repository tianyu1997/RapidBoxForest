#pragma once

#include "planning_forest_query_bridge_diagnostics.h"
#include "planning_forest_query_bridge_options.h"
#include "planning_forest_query_bridge_rrt_utils.h"
#include "planning_forest_query_bridge_task.h"

#include <vector>

namespace rbf {

bool query_bridge_should_check_current_query(
    const QueryBridgeSearchTask& task,
    bool respect_forced,
    const QueryBridgeIndexOptions& index_options);

bool query_bridge_has_segment_only_task(
    const std::vector<QueryBridgeSearchTask>& tasks,
    const QueryBridgeIndexOptions& index_options);

bool query_bridge_parallel_task_rrt_enabled(
    const QueryBridgeBatchExecutionOptions& batch_options,
    bool has_segment_only_task,
    const QueryBridgeRetryOptions& retry_options);

bool query_bridge_task_has_explicit_satisfaction(
    const QueryBridgeSearchTask& task);

int query_bridge_edge_query_index(bool scene_reusable_edges,
                                  const QueryBridgeSearchTask& task);

QueryBridgeAttemptPlan query_bridge_prepare_attempt_plan(
    const QueryBridgeSearchTask& task,
    const QueryBridgeIndexOptions& index_options,
    const QueryBridgeRetryOptions& retry_options,
    StageContext& context);

}  // namespace rbf
