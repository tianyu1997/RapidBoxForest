#include "planning_forest_query_bridge_corridor_tasks.h"

#include <algorithm>
#include <chrono>

namespace rbf {

QueryBridgeDirectFfbTaskBuildResult query_bridge_build_direct_ffb_tasks(
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<bool>& covered,
    const QueryBridgeDirectFfbTaskBuildOptions& options) {
    QueryBridgeDirectFfbTaskBuildResult result;
    result.tasks.reserve(samples.size());
    const int max_transition_hint = std::max(0, options.max_transition_hint);
    auto append_sample = [&](std::size_t sample_index) {
        result.tasks.push_back(
            {samples[sample_index],
             sample_index,
             std::min(static_cast<int>(sample_index), max_transition_hint)});
    };
    auto sample_covered = [&](std::size_t sample_index) {
        return sample_index < covered.size() && covered[sample_index];
    };
    bool in_gap = false;
    for (std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {
        if (sample_covered(sample_index)) {
            in_gap = false;
            continue;
        }
        if (!in_gap) {
            result.uncovered_gap_groups += 1;
            in_gap = true;
        }
        append_sample(sample_index);
    }
    return result;
}

QueryBridgeDirectFfbTaskPlan query_bridge_prepare_direct_ffb_task_plan(
    StageContext& context,
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<bool>& covered,
    int ffb_start_depth) {
    QueryBridgeDirectFfbTaskPlan plan;
    QueryBridgeDirectFfbTaskBuildResult build =
        query_bridge_build_direct_ffb_tasks(
            samples,
            covered,
            QueryBridgeDirectFfbTaskBuildOptions{
                std::max(0, static_cast<int>(samples.size()) - 2)});
    plan.uncovered_gap_groups = build.uncovered_gap_groups;
    plan.tasks = std::move(build.tasks);

    context.diagnostics().set_value("query_bridge.direct_corridor_ffb_start_depth",
                                    static_cast<double>(ffb_start_depth));
    context.diagnostics().set_value("query_bridge.direct_corridor_uncovered_gap_groups",
                                    static_cast<double>(plan.uncovered_gap_groups));
    context.diagnostics().set_value("query_bridge.direct_corridor_direct_tasks",
                                    static_cast<double>(plan.tasks.size()));
    return plan;
}

QueryBridgeFfbTaskExecutionStats query_bridge_run_direct_ffb_tasks(
    StageContext& context,
    const std::vector<QueryBridgeDirectFfbTask>& tasks,
    const std::vector<bool>& covered,
    const std::function<FindFreeBoxResult(const QueryBridgeDirectFfbTask&)>& find_box,
    const std::function<QueryBridgeFfbTaskCommitResult(FindFreeBoxResult&&,
                                                       const QueryBridgeDirectFfbTask&)>& commit_box) {
    using Clock = std::chrono::steady_clock;
    QueryBridgeFfbTaskExecutionStats stats;
    for (const auto& task : tasks) {
        if (task.sample_index < covered.size() && covered[task.sample_index]) {
            context.diagnostics().add_counter(
                "query_bridge.direct_corridor_direct_skip_covered");
            continue;
        }
        const auto ffb_t0 = Clock::now();
        FindFreeBoxResult result = find_box(task);
        stats.ffb_ms +=
            std::chrono::duration<double, std::milli>(Clock::now() - ffb_t0).count();
        stats.calls += 1;
        const QueryBridgeFfbTaskCommitResult commit = commit_box(std::move(result), task);
        if (commit.box_index >= 0 && commit.added_box) {
            stats.added += 1;
        }
    }
    return stats;
}

}  // namespace rbf
