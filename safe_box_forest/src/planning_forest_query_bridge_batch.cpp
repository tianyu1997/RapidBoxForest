#include <SBF/safe_box_forest.h>

#include <SBF/oracle.h>

#include "planning_forest_diagnostics.h"
#include "planning_forest_query_bridge_corridor_options.h"
#include "planning_forest_query_bridge_diagnostics.h"
#include "planning_forest_query_bridge_options.h"
#include "planning_forest_query_bridge_policy.h"
#include "planning_forest_query_bridge_rrt_utils.h"
#include "planning_forest_query_bridge_task.h"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <unordered_set>
#include <vector>

namespace rbf {

namespace {

using QueryBridgeClock = std::chrono::steady_clock;

double query_bridge_elapsed_ms_since(QueryBridgeClock::time_point t0) {
    return std::chrono::duration<double, std::milli>(
        QueryBridgeClock::now() - t0).count();
}

}  // namespace

std::vector<int> RBFPlanningForest::finish_query_bridge_batch_result(
    const std::vector<int>& added_by_query,
    std::size_t partition_refresh_base,
    std::size_t segment_edges_before_partition_refresh,
    bool oracle_counters_before_valid,
    const OracleCounters& oracle_counters_before) {
    if (oracle_counters_before_valid && oracle_) {
        const auto after = oracle_->counters();
        add_query_bridge_oracle_counter_delta(last_build_,
                                             oracle_counters_before,
                                             after);
    }
    const bool changed =
        boxes_.size() != partition_refresh_base ||
        segment_edges_.size() != segment_edges_before_partition_refresh ||
        std::any_of(added_by_query.begin(),
                    added_by_query.end(),
                    [](int added) { return added > 0; });
    if (boxes_.size() > partition_refresh_base) {
        append_adaptive_partition_boxes(partition_refresh_base,
                                        &last_build_,
                                        "query_bridge.batch");
    } else if (changed) {
        sync_adaptive_partition_segment_edges(&last_build_, "query_bridge.batch");
        refresh_adaptive_partition_diagnostics(&last_build_);
    }
    return added_by_query;
}

std::vector<int> RBFPlanningForest::bridge_queries(const std::vector<Eigen::VectorXd>& starts,
                                                   const std::vector<Eigen::VectorXd>& goals) {
    return bridge_queries(starts, goals, QueryBridgeBatchOptions{});
}

std::vector<int> RBFPlanningForest::bridge_queries(const std::vector<Eigen::VectorXd>& starts,
                                                   const std::vector<Eigen::VectorXd>& goals,
                                                   const QueryBridgeBatchOptions& options) {
    if (starts.size() != goals.size()) {
        throw std::invalid_argument("bridge_queries requires starts/goals with matching sizes");
    }
    std::vector<int> added_by_query(starts.size(), 0);
    std::size_t partition_refresh_base = boxes_.size();
    const std::size_t segment_edges_before_partition_refresh = segment_edges_.size();
    OracleCounters oracle_counters_before;
    bool oracle_counters_before_valid = false;
    if (starts.empty() || !oracle_) {
        return added_by_query;
    }
    oracle_counters_before = oracle_->counters();
    oracle_counters_before_valid = true;

    const QueryBridgeAcceptanceThresholds bridge_acceptance =
        query_bridge_acceptance_thresholds_from_config(config_);
    const std::unordered_set<int> forced_query_indices =
        query_bridge_forced_query_index_set(options.forced_query_indices, starts.size());

    std::vector<QueryBridgeSearchTask> tasks =
        prepare_query_bridge_batch_tasks(starts,
                                         goals,
                                         options,
                                         forced_query_indices,
                                         bridge_acceptance,
                                         partition_refresh_base);

    if (tasks.empty()) {
        return finish_query_bridge_batch_result(
            added_by_query,
            partition_refresh_base,
            segment_edges_before_partition_refresh,
            oracle_counters_before_valid,
            oracle_counters_before);
    }
    const auto batch_t0 = QueryBridgeClock::now();
    StageContext batch_context = StageContext::from_runtime(config_.runtime);
    const QueryBridgeEdgeRuntimeOptions edge_options =
        query_bridge_edge_runtime_options_from_config(config_);
    const bool scene_reusable_edges = edge_options.scene_reusable_edges;
    record_query_bridge_edge_runtime_diagnostics(batch_context, edge_options);
    ScopedStageDiagnosticsFlush batch_diagnostics_flush(last_build_, batch_context);
    batch_context.diagnostics().set_value("query_bridge.batch_tasks_initial",
                                          static_cast<double>(tasks.size()));
    if (edge_options.direct_start_goal_segment_enabled) {
        run_query_bridge_direct_start_goal_segments(tasks,
                                                    added_by_query,
                                                    batch_context,
                                                    scene_reusable_edges);
    }
    const QueryBridgeRetryOptions retry_options =
        query_bridge_retry_options_from_config(config_);
    record_query_bridge_retry_diagnostics(batch_context, retry_options);
    const QueryBridgeParallelRrtOptions parallel_rrt_options =
        query_bridge_parallel_rrt_options_from_config(config_);
    record_query_bridge_parallel_rrt_diagnostics(batch_context, parallel_rrt_options);
    record_query_bridge_acceptance_diagnostics(batch_context, bridge_acceptance);
    const QueryBridgeHybridizeAttemptOptions hybrid_options =
        query_bridge_hybridize_attempt_options_from_config(config_);
    if (last_adaptive_partition_config_.hipac_online_connectivity &&
        last_adaptive_partition_config_.hipac_online_before_query_bridge) {
        for (auto& task : tasks) {
            run_query_bridge_hipac_online_sequence_task(task,
                                                        added_by_query[task.index],
                                                        batch_context,
                                                        scene_reusable_edges,
                                                        bridge_acceptance);
        }
    }

    batch_context.diagnostics().set_value("query_bridge.attempt_offset",
                                          static_cast<double>(retry_options.attempt_offset));
    if (query_bridge_parallel_task_rrt_enabled(retry_options)) {
        run_query_bridge_batch_parallel_rrt(tasks,
                                            added_by_query,
                                            forced_query_indices,
                                            bridge_acceptance,
                                            retry_options,
                                            parallel_rrt_options,
                                            hybrid_options,
                                            edge_options,
                                            scene_reusable_edges,
                                            batch_context,
                                            batch_t0);
        batch_context.diagnostics().set_value("query_bridge.batch_total_ms",
                                              query_bridge_elapsed_ms_since(batch_t0));
        return finish_query_bridge_batch_result(
            added_by_query,
            partition_refresh_base,
            segment_edges_before_partition_refresh,
            oracle_counters_before_valid,
            oracle_counters_before);
    }

    run_query_bridge_batch_serial_rrt(tasks,
                                      added_by_query,
                                      forced_query_indices,
                                      bridge_acceptance,
                                      retry_options,
                                      parallel_rrt_options,
                                      hybrid_options,
                                      edge_options,
                                      scene_reusable_edges,
                                      batch_context);

    batch_context.diagnostics().set_value("query_bridge.batch_total_ms",
                                          query_bridge_elapsed_ms_since(batch_t0));
    return finish_query_bridge_batch_result(
        added_by_query,
        partition_refresh_base,
        segment_edges_before_partition_refresh,
        oracle_counters_before_valid,
        oracle_counters_before);
}

} // namespace rbf
