#include "planning_forest_query_utils.h"

#include "planning_forest_audit.h"
#include "planning_forest_query_bridge_rrt_utils.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <limits>
#include <memory>

namespace rbf {

std::vector<Eigen::VectorXd> best_audited_rrt_bridge_path(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    const CollisionChecker& checker,
    const Robot& robot,
    StageContext& context,
    const RRTConnectConfig& base_config,
    int attempts,
    double total_timeout_ms,
    int seed_base,
    int audit_resolution,
    double audit_segment_step,
    const QueryBridgeParallelRrtOptions& parallel_options,
    const std::vector<RRTConnectConfig>* attempt_configs,
    int seed_stride) {
    using Clock = std::chrono::steady_clock;
    std::vector<Eigen::VectorXd> best;
    double best_length = std::numeric_limits<double>::infinity();
    const int safe_attempts = std::max(1, attempts);
    const double safe_total_ms = total_timeout_ms > 0.0 ? total_timeout_ms : base_config.timeout_ms;
    auto early_stop_path_good = [&](const std::vector<Eigen::VectorXd>& path) {
        return query_bridge_parallel_rrt_path_good_enough(start,
                                                         goal,
                                                         path,
                                                         parallel_options);
    };
    record_query_bridge_parallel_rrt_diagnostics(context, parallel_options);

    if (context.executor().n_threads() > 1 && safe_attempts > 1) {
        const double per_attempt_ms =
            safe_total_ms > 0.0
                ? std::max(1.0, safe_total_ms / static_cast<double>(safe_attempts))
                : base_config.timeout_ms;
        std::vector<std::vector<Eigen::VectorXd>> audited_paths(static_cast<std::size_t>(safe_attempts));
        std::shared_ptr<std::atomic<bool>> local_cancel =
            parallel_options.early_stop ? std::make_shared<std::atomic<bool>>(false)
                                        : context.native_cancel_flag();
        std::atomic<int> early_successes{0};
        context.executor().parallel_for(0, safe_attempts, [&](int attempt) {
            if (context.should_stop() ||
                (local_cancel && local_cancel->load(std::memory_order_relaxed))) {
                return;
            }
            RRTConnectConfig config =
                (attempt_configs != nullptr && !attempt_configs->empty())
                    ? (*attempt_configs)[static_cast<std::size_t>(attempt) % attempt_configs->size()]
                    : base_config;
            if (per_attempt_ms > 0.0) {
                config.timeout_ms = per_attempt_ms;
            }
            std::vector<Eigen::VectorXd> path =
                rrt_connect(start,
                            goal,
                            checker,
                            robot,
                            config,
                            seed_base + attempt * std::max(1, seed_stride),
                            local_cancel);
            if (path.empty()) {
                return;
            }
            const PathAuditCheck audit =
                audit_waypoint_path(path, checker, audit_resolution, audit_segment_step);
            if (!audit.passed) {
                return;
            }
            if (parallel_options.early_stop && early_stop_path_good(path)) {
                const int successes =
                    early_successes.fetch_add(1, std::memory_order_relaxed) + 1;
                if (successes >= parallel_options.early_stop_min_successes && local_cancel) {
                    local_cancel->store(true, std::memory_order_relaxed);
                }
            }
            audited_paths[static_cast<std::size_t>(attempt)] = std::move(path);
        });

        const int audited_successes = static_cast<int>(std::count_if(
            audited_paths.begin(),
            audited_paths.end(),
            [](const auto& path) { return !path.empty(); }));
        for (auto& path : audited_paths) {
            if (path.empty()) {
                continue;
            }
            const double length = path_length(path);
            if (length < best_length) {
                best_length = length;
                best = std::move(path);
            }
        }
        context.diagnostics().add_counter("query_bridge.parallel_rrt_attempts",
                                          static_cast<double>(safe_attempts));
        context.diagnostics().add_counter("query_bridge.parallel_rrt_successes",
                                          static_cast<double>(audited_successes));
        if (parallel_options.early_stop) {
            context.diagnostics().add_counter("query_bridge.parallel_rrt_early_stop_successes",
                                              static_cast<double>(early_successes.load(
                                                  std::memory_order_relaxed)));
            context.diagnostics().add_counter(
                local_cancel && local_cancel->load(std::memory_order_relaxed)
                    ? "query_bridge.parallel_rrt_early_stop_triggered"
                    : "query_bridge.parallel_rrt_early_stop_not_triggered");
        }
        return best;
    }

    const auto t0 = Clock::now();
    auto elapsed_ms = [&]() {
        return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    };
    for (int attempt = 0; attempt < safe_attempts; ++attempt) {
        if (context.should_stop()) {
            break;
        }
        RRTConnectConfig config =
            (attempt_configs != nullptr && !attempt_configs->empty())
                ? (*attempt_configs)[static_cast<std::size_t>(attempt) % attempt_configs->size()]
                : base_config;
        if (safe_total_ms > 0.0) {
            const double remaining_ms = safe_total_ms - elapsed_ms();
            if (remaining_ms <= 0.0) {
                break;
            }
            const int attempts_left = safe_attempts - attempt;
            config.timeout_ms = std::max(1.0, remaining_ms / static_cast<double>(attempts_left));
        }
        std::vector<Eigen::VectorXd> path =
            rrt_connect(start,
                        goal,
                        checker,
                        robot,
                        context,
                        config,
                        seed_base + attempt * std::max(1, seed_stride));
        if (path.empty()) {
            continue;
        }
        PathAuditCheck audit = audit_waypoint_path(path, checker, audit_resolution, audit_segment_step);
        if (!audit.passed) {
            continue;
        }
        const double length = path_length(path);
        if (length < best_length) {
            best_length = length;
            best = std::move(path);
        }
    }
    return best;
}

} // namespace rbf
