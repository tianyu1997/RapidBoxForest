#include "planning_forest_query_bridge_rrt_utils.h"

#include <SBF/runtime.h>

#include <SBF/box_graph.h>

#include "planning_forest_query_bridge_task.h"
#include "../query_runtime/planning_forest_query_utils.h"

#include <algorithm>

namespace rbf {

bool query_bridge_parallel_rrt_path_good_enough(
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal,
    const std::vector<Eigen::VectorXd>& path,
    const QueryBridgeParallelRrtOptions& options) {
    if (path.empty()) {
        return false;
    }
    const double direct = (goal - start).norm();
    if (direct <= 1e-9) {
        return true;
    }
    const double length = path_length(path);
    return length <= std::max(direct * options.early_stop_ratio,
                              direct + options.early_stop_additive);
}

bool query_bridge_task_rrt_path_good_enough(
    const QueryBridgeSearchTask& task,
    const std::vector<Eigen::VectorXd>& path,
    const QueryBridgeParallelRrtOptions& options) {
    return query_bridge_parallel_rrt_path_good_enough(task.start,
                                                      task.goal,
                                                      path,
                                                      options);
}

std::shared_ptr<std::atomic<bool>> query_bridge_parallel_rrt_cancel_flag(
    const QueryBridgeParallelRrtOptions& options,
    const std::shared_ptr<std::atomic<bool>>& fallback_cancel) {
    if (!options.early_stop) {
        return fallback_cancel;
    }
    return std::make_shared<std::atomic<bool>>(false);
}

bool query_bridge_parallel_rrt_cancelled(
    const std::shared_ptr<std::atomic<bool>>& cancel_flag) {
    return cancel_flag && cancel_flag->load(std::memory_order_relaxed);
}

void query_bridge_maybe_stop_parallel_rrt_after_success(
    bool path_good_enough,
    const QueryBridgeParallelRrtOptions& options,
    std::atomic<int>& early_successes,
    const std::shared_ptr<std::atomic<bool>>& cancel_flag) {
    if (!options.early_stop || !path_good_enough || !cancel_flag) {
        return;
    }
    const int successes =
        early_successes.fetch_add(1, std::memory_order_relaxed) + 1;
    if (successes >= options.early_stop_min_successes) {
        cancel_flag->store(true, std::memory_order_relaxed);
    }
}

void record_query_bridge_parallel_rrt_early_stop(
    StageContext& context,
    const QueryBridgeParallelRrtOptions& options,
    const std::shared_ptr<std::atomic<bool>>& cancel_flag,
    const std::atomic<int>& early_successes) {
    if (!options.early_stop) {
        return;
    }
    context.diagnostics().add_counter(
        "query_bridge.parallel_rrt_early_stop_successes",
        static_cast<double>(early_successes.load(std::memory_order_relaxed)));
    context.diagnostics().add_counter(
        query_bridge_parallel_rrt_cancelled(cancel_flag)
            ? "query_bridge.parallel_rrt_early_stop_triggered"
            : "query_bridge.parallel_rrt_early_stop_not_triggered");
}

}  // namespace rbf
