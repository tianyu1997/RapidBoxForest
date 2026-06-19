#include "planning_forest_query_bridge_corridor_tasks.h"

#include <algorithm>
#include <chrono>
#include <cmath>

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
    if (options.grouped_direct_seeds) {
        const int max_group_seeds = std::max(1, options.max_group_seeds);
        std::size_t sample_index = 0;
        while (sample_index < samples.size()) {
            while (sample_index < samples.size() && sample_covered(sample_index)) {
                ++sample_index;
            }
            if (sample_index >= samples.size()) {
                break;
            }
            const std::size_t begin = sample_index;
            while (sample_index < samples.size() && !sample_covered(sample_index)) {
                ++sample_index;
            }
            const std::size_t end = sample_index - 1;
            result.uncovered_gap_groups += 1;
            std::vector<std::size_t> chosen;
            chosen.reserve(static_cast<std::size_t>(
                std::min(max_group_seeds, static_cast<int>(end - begin + 1))));
            auto push_unique_index = [&](std::size_t index) {
                index = std::min(end, std::max(begin, index));
                if (std::find(chosen.begin(), chosen.end(), index) == chosen.end()) {
                    chosen.push_back(index);
                }
            };
            const std::size_t group_count = end - begin + 1;
            if (group_count <= static_cast<std::size_t>(max_group_seeds)) {
                for (std::size_t index = begin; index <= end; ++index) {
                    push_unique_index(index);
                }
            } else {
                push_unique_index((begin + end) / 2);
                if (static_cast<int>(chosen.size()) < max_group_seeds) {
                    push_unique_index(begin);
                }
                if (static_cast<int>(chosen.size()) < max_group_seeds) {
                    push_unique_index(end);
                }
                if (static_cast<int>(chosen.size()) < max_group_seeds && end > begin + 1) {
                    push_unique_index(begin + (end - begin) / 4);
                }
                if (static_cast<int>(chosen.size()) < max_group_seeds && end > begin + 1) {
                    push_unique_index(begin + (3 * (end - begin)) / 4);
                }
                for (int rank = 1;
                     static_cast<int>(chosen.size()) < max_group_seeds &&
                     rank < max_group_seeds - 1;
                     ++rank) {
                    const double alpha = static_cast<double>(rank) /
                                         static_cast<double>(max_group_seeds - 1);
                    const auto offset = static_cast<std::size_t>(
                        std::llround(alpha * static_cast<double>(end - begin)));
                    push_unique_index(begin + offset);
                }
            }
            for (std::size_t index : chosen) {
                append_sample(index);
            }
        }
    } else if (options.center_out_direct_tasks) {
        std::size_t sample_index = 0;
        while (sample_index < samples.size()) {
            while (sample_index < samples.size() && sample_covered(sample_index)) {
                ++sample_index;
            }
            if (sample_index >= samples.size()) {
                break;
            }
            const std::size_t begin = sample_index;
            while (sample_index < samples.size() && !sample_covered(sample_index)) {
                ++sample_index;
            }
            const std::size_t end = sample_index - 1;
            result.uncovered_gap_groups += 1;
            const std::size_t center = (begin + end) / 2;
            append_sample(center);
            for (std::size_t radius = 1;
                 center >= begin + radius || center + radius <= end;
                 ++radius) {
                if (center >= begin + radius) {
                    append_sample(center - radius);
                }
                if (center + radius <= end) {
                    append_sample(center + radius);
                }
            }
        }
    } else {
        for (std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {
            if (!sample_covered(sample_index)) {
                append_sample(sample_index);
            }
        }
    }
    return result;
}

QueryBridgeDirectFfbTaskPlan query_bridge_prepare_direct_ffb_task_plan(
    StageContext& context,
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<bool>& covered,
    int ffb_start_depth) {
    QueryBridgeDirectFfbTaskPlan plan;
    plan.runtime = query_bridge_direct_ffb_task_runtime_options(samples.size());
    QueryBridgeDirectFfbTaskBuildResult build =
        query_bridge_build_direct_ffb_tasks(samples, covered, plan.runtime.build);
    plan.uncovered_gap_groups = build.uncovered_gap_groups;
    plan.tasks = std::move(build.tasks);

    context.diagnostics().set_value("query_bridge.direct_corridor_direct_grouped_seeds",
                                    plan.runtime.build.grouped_direct_seeds ? 1.0 : 0.0);
    context.diagnostics().set_value("query_bridge.direct_corridor_coverage_order_direct_tasks",
                                    plan.runtime.coverage_order_direct_tasks ? 1.0 : 0.0);
    context.diagnostics().set_value("query_bridge.direct_corridor_center_out_direct_tasks",
                                    plan.runtime.build.center_out_direct_tasks ? 1.0 : 0.0);
    context.diagnostics().set_value("query_bridge.direct_corridor_ffb_start_depth",
                                    static_cast<double>(ffb_start_depth));
    context.diagnostics().set_value("query_bridge.direct_corridor_uncovered_gap_groups",
                                    static_cast<double>(plan.uncovered_gap_groups));
    context.diagnostics().set_value("query_bridge.direct_corridor_direct_max_seeds_per_gap",
                                    static_cast<double>(plan.runtime.build.max_group_seeds));
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
