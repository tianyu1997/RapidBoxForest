#pragma once

#include "planning_forest_query_bridge_diagnostics.h"
#include "planning_forest_query_bridge_options.h"
#include "planning_forest_query_bridge_rrt_utils.h"
#include "planning_forest_query_bridge_task.h"

#include <unordered_set>

namespace rbf {

bool query_bridge_should_check_current_query(
    const QueryBridgeSearchTask& task,
    bool respect_forced,
    const std::unordered_set<int>& forced_query_indices);

bool query_bridge_parallel_task_rrt_enabled(
    const QueryBridgeRetryOptions& retry_options);

bool query_bridge_task_has_explicit_satisfaction(
    const QueryBridgeSearchTask& task);

int query_bridge_edge_query_index(bool scene_reusable_edges,
                                  const QueryBridgeSearchTask& task);

QueryBridgeAttemptPlan query_bridge_prepare_attempt_plan(
    const QueryBridgeSearchTask& task,
    const std::unordered_set<int>& forced_query_indices,
    const QueryBridgeRetryOptions& retry_options,
    StageContext& context);

}  // namespace rbf
