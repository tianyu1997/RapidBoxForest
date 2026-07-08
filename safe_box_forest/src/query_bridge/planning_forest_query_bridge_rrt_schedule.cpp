#include "planning_forest_query_bridge_rrt_utils.h"

#include "planning_forest_query_bridge_task.h"
#include "../query_runtime/planning_forest_query_utils.h"

#include <algorithm>
#include <utility>

namespace rbf {

bool query_bridge_short_local_distance(double bridge_distance) {
    return bridge_distance > 0.55 && bridge_distance < 0.85;
}

void query_bridge_configure_short_local_profiles(
    RRTConnectConfig& bridge_rrt,
    std::vector<RRTConnectConfig>& short_local_profiles) {
    bridge_rrt.step_size = std::min(bridge_rrt.step_size, 0.25);
    bridge_rrt.goal_bias = 0.08;
    bridge_rrt.local_sampling_radius =
        bridge_rrt.local_sampling_radius > 0.0
            ? std::min(bridge_rrt.local_sampling_radius, 0.85)
            : 0.85;
    auto add_profile = [&](double step_size, double goal_bias, double radius) {
        RRTConnectConfig profile = bridge_rrt;
        profile.step_size = step_size;
        profile.goal_bias = goal_bias;
        profile.local_sampling_radius = radius;
        profile.shortcut_path = true;
        short_local_profiles.push_back(std::move(profile));
    };
    add_profile(0.25, 0.08, 0.90);
    add_profile(0.50, 0.20, 1.00);
    add_profile(0.35, 0.10, 1.00);
    add_profile(0.25, 0.08, 0.45);
}

RRTConnectConfig query_bridge_rrt_config_for_attempt(
    const QueryBridgeSearchTask& task,
    int attempt,
    int scheduled_attempt,
    int override_fixed_iters,
    double default_timeout_ms,
    const QueryBridgeRetryOptions& options) {
    RRTConnectConfig config =
        task.short_local_profiles.empty()
            ? task.bridge_rrt
            : task.short_local_profiles[
                  static_cast<std::size_t>(scheduled_attempt) % task.short_local_profiles.size()];
    if (!options.local_radius_schedule.empty() &&
        attempt >= 0 &&
        static_cast<std::size_t>(attempt) < options.local_radius_schedule.size()) {
        const double scheduled_radius =
            options.local_radius_schedule[static_cast<std::size_t>(attempt)];
        if (scheduled_radius >= 0.0) {
            config.local_sampling_radius = scheduled_radius;
        }
    }
    const int effective_fixed_iters =
        override_fixed_iters > 0 ? override_fixed_iters : options.rrt_fixed_iters;
    if (effective_fixed_iters > 0) {
        config.max_iters = effective_fixed_iters;
        config.timeout_ms = 0.0;
    } else {
        config.timeout_ms = std::max(1.0, default_timeout_ms);
    }
    return config;
}

int query_bridge_rrt_seed_for_attempt(const QueryBridgeSearchTask& task,
                                      int rng_seed,
                                      int scheduled_attempt) {
    return derived_planner_seed(rng_seed,
                                kSeedBatchBridgeOffset,
                                scheduled_attempt,
                                task.query_index,
                                task.short_local_bridge ? 0 : kSeedAttemptStride);
}

QueryBridgeAttemptPlan query_bridge_attempt_plan(
    const QueryBridgeSearchTask& task,
    bool forced,
    const QueryBridgeRetryOptions& options) {
    QueryBridgeAttemptPlan plan;
    plan.forced = forced;
    plan.base_attempts =
        forced ? std::max(std::max(1, task.attempts), options.forced_attempts)
               : std::max(1, task.attempts);
    plan.effective_attempts = plan.base_attempts;
    if (plan.effective_attempts > 0 &&
        !options.local_radius_schedule.empty()) {
        plan.effective_attempts =
            std::max(plan.effective_attempts,
                     static_cast<int>(options.local_radius_schedule.size()) + 1);
    }
    return plan;
}

}  // namespace rbf
