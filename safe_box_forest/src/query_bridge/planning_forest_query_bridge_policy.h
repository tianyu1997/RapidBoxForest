#pragma once

#include <SBF/query_bridge_config.h>

#include "planning_forest_query_bridge_diagnostics.h"
#include "planning_forest_query_bridge_options.h"
#include "planning_forest_query_bridge_rrt_utils.h"
#include "planning_forest_query_bridge_task.h"

#include <cstddef>
#include <unordered_set>
#include <vector>

namespace rbf {

bool query_bridge_should_check_current_query(
    const QueryBridgeSearchTask& task,
    bool respect_forced,
    const std::unordered_set<int>& forced_query_indices);

bool query_bridge_current_query_good(
    const QueryResult& current,
    const QueryBridgeSearchTask& task,
    const QueryBridgeAcceptanceThresholds& bridge_acceptance);

bool query_bridge_parallel_task_rrt_enabled(
    const QueryBridgeRetryOptions& retry_options);

bool query_bridge_task_has_explicit_satisfaction(
    const QueryBridgeSearchTask& task);

int query_bridge_edge_query_index(bool scene_reusable_edges,
                                  const QueryBridgeSearchTask& task);

std::unordered_set<int> query_bridge_forced_query_index_set(
    const std::vector<int>& forced_query_indices,
    std::size_t batch_size);

int query_bridge_batch_global_query_index(
    const QueryBridgeBatchOptions& options,
    std::size_t index);

QueryBridgeAttemptPlan query_bridge_prepare_attempt_plan(
    const QueryBridgeSearchTask& task,
    const std::unordered_set<int>& forced_query_indices,
    const QueryBridgeRetryOptions& retry_options,
    StageContext& context);

}  // namespace rbf
