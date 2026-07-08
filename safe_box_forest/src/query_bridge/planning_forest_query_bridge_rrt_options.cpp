#include "planning_forest_query_bridge_rrt_utils.h"

#include <SBF/runtime.h>

#include <SBF/planning_config.h>

#include <algorithm>
#include <string>

namespace rbf {

QueryBridgeRetryOptions query_bridge_retry_options_from_config(
    const RBFPlanningConfig& config) {
    QueryBridgeRetryOptions options;
    options.no_path_retry_attempts =
        std::max(0, config.query_bridge_no_path_retry_attempts);
    options.no_path_retry_stop_on_first_success =
        config.query_bridge_no_path_retry_stop_on_first_success;
    options.forced_attempts =
        std::max(1, config.query_bridge_forced_attempts);
    options.attempt_offset =
        std::max(0, config.query_bridge_attempt_offset);
    options.rrt_fixed_iters =
        std::max(0, config.query_bridge_rrt_fixed_iters);
    options.local_radius_schedule =
        config.query_bridge_local_radius_schedule;
    options.no_path_retry_budget_iters =
        config.query_bridge_no_path_retry_budget_iters;
    options.no_path_retry_budget_attempts =
        config.query_bridge_no_path_retry_budget_attempts;
    options.no_path_retry_budget_stages =
        std::min(options.no_path_retry_budget_iters.size(),
                 options.no_path_retry_budget_attempts.size());
    return options;
}

void record_query_bridge_retry_diagnostics(StageContext& context,
                                           const QueryBridgeRetryOptions& options) {
    context.diagnostics().set_value("query_bridge.no_path_retry_attempts_default",
                                    static_cast<double>(options.no_path_retry_attempts));
    context.diagnostics().set_value("query_bridge.no_path_retry_stop_on_first_success",
                                    options.no_path_retry_stop_on_first_success ? 1.0 : 0.0);
    context.diagnostics().set_value("query_bridge.rrt_fixed_iters",
                                    static_cast<double>(options.rrt_fixed_iters));
    context.diagnostics().set_value("query_bridge.local_radius_schedule_size",
                                    static_cast<double>(options.local_radius_schedule.size()));
    context.diagnostics().set_value("query_bridge.no_path_retry_budget_stages",
                                    static_cast<double>(options.no_path_retry_budget_stages));
    for (std::size_t stage = 0; stage < options.no_path_retry_budget_stages; ++stage) {
        const std::string prefix =
            "query_bridge.no_path_retry_budget_stage." + std::to_string(stage) + ".";
        context.diagnostics().set_value(
            prefix + "iters",
            static_cast<double>(options.no_path_retry_budget_iters[stage]));
        context.diagnostics().set_value(
            prefix + "attempts",
            static_cast<double>(options.no_path_retry_budget_attempts[stage]));
    }
}

QueryBridgeParallelRrtOptions query_bridge_parallel_rrt_options_from_config(
    const RBFPlanningConfig& config) {
    QueryBridgeParallelRrtOptions options;
    options.early_stop = config.query_bridge_parallel_rrt_early_stop;
    options.early_stop_min_successes =
        std::max(1, config.query_bridge_parallel_rrt_early_stop_min_successes);
    options.early_stop_ratio =
        std::max(1.0, config.query_bridge_parallel_rrt_early_stop_ratio);
    options.early_stop_additive =
        std::max(0.0, config.query_bridge_parallel_rrt_early_stop_additive);
    return options;
}

void record_query_bridge_parallel_rrt_diagnostics(
    StageContext& context,
    const QueryBridgeParallelRrtOptions& options) {
    context.diagnostics().set_value("query_bridge.parallel_rrt_early_stop_enabled",
                                    options.early_stop ? 1.0 : 0.0);
}

}  // namespace rbf
