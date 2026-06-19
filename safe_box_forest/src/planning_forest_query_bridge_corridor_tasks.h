#pragma once

#include <SBF/find_free_box.h>
#include <SBF/runtime.h>

#include <Eigen/Core>

#include <cstddef>
#include <functional>
#include <vector>

namespace rbf {

struct QueryBridgeDirectFfbTask {
    Eigen::VectorXd seed;
    std::size_t sample_index = 0;
    int transition_hint = 0;
};

struct QueryBridgeDirectFfbTaskBuildOptions {
    int max_transition_hint = 0;
    int max_group_seeds = 3;
    bool grouped_direct_seeds = false;
    bool center_out_direct_tasks = false;
};

struct QueryBridgeDirectFfbTaskBuildResult {
    std::vector<QueryBridgeDirectFfbTask> tasks;
    int uncovered_gap_groups = 0;
};

struct QueryBridgeDirectFfbTaskRuntimeOptions {
    QueryBridgeDirectFfbTaskBuildOptions build;
    bool coverage_order_direct_tasks = true;
};

struct QueryBridgeDirectFfbTaskPlan {
    std::vector<QueryBridgeDirectFfbTask> tasks;
    QueryBridgeDirectFfbTaskRuntimeOptions runtime;
    int uncovered_gap_groups = 0;
    double build_ms = 0.0;
};

struct QueryBridgeFfbTaskCommitResult {
    int box_index = -1;
    bool added_box = false;
};

struct QueryBridgeFfbTaskExecutionStats {
    int calls = 0;
    int added = 0;
    double ffb_ms = 0.0;
    double loop_ms = 0.0;
};

QueryBridgeDirectFfbTaskBuildResult query_bridge_build_direct_ffb_tasks(
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<bool>& covered,
    const QueryBridgeDirectFfbTaskBuildOptions& options);

QueryBridgeDirectFfbTaskRuntimeOptions query_bridge_direct_ffb_task_runtime_options(
    std::size_t sample_count);

QueryBridgeDirectFfbTaskPlan query_bridge_prepare_direct_ffb_task_plan(
    StageContext& context,
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<bool>& covered,
    int ffb_start_depth,
    bool detailed_timing);

QueryBridgeFfbTaskExecutionStats query_bridge_run_direct_ffb_tasks(
    StageContext& context,
    const std::vector<QueryBridgeDirectFfbTask>& tasks,
    const std::vector<bool>& covered,
    const std::function<FindFreeBoxResult(const QueryBridgeDirectFfbTask&)>& find_box,
    const std::function<QueryBridgeFfbTaskCommitResult(FindFreeBoxResult&&,
                                                       const QueryBridgeDirectFfbTask&)>& commit_box,
    bool detailed_timing);

}  // namespace rbf
