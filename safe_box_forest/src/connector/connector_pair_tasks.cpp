#include "connector_pair_tasks.h"

#include <SBF/connector.h>
#include <SBF/runtime.h>
#include <SBF/scene.h>

#include "connector_birrt.h"
#include "connector_internal.h"

#include <atomic>
#include <memory>
#include <utility>

namespace rbf {

namespace {

RRTConnectConfig connector_pair_rrt_config(const IslandConnectorConfig& config) {
    RRTConnectConfig pair_rrt = config.rrt;
    if (config.per_pair_timeout_ms > 0.0 &&
        (pair_rrt.timeout_ms <= 0.0 || config.per_pair_timeout_ms < pair_rrt.timeout_ms)) {
        pair_rrt.timeout_ms = config.per_pair_timeout_ms;
    }
    return pair_rrt;
}

BridgePairResult make_successful_bridge_pair_result(
    const BridgePairTask& candidate,
    std::vector<Eigen::VectorXd> path) {
    return {candidate.task_id,
            candidate.source_box_id,
            candidate.target_box_id,
            std::move(path),
            true};
}

bool record_colliding_centers(const CollisionChecker& checker,
                              const Eigen::Ref<const Eigen::VectorXd>& source_center,
                              const Eigen::Ref<const Eigen::VectorXd>& target_center,
                              StageContext& context) {
    const bool source_colliding = checker.check_config(source_center);
    const bool target_colliding = checker.check_config(target_center);
    if (!source_colliding && !target_colliding) {
        return false;
    }
    context.diagnostics().add_counter("connector.center_collision_candidates");
    if (source_colliding) {
        context.diagnostics().add_counter("connector.source_center_collisions");
    }
    if (target_colliding) {
        context.diagnostics().add_counter("connector.target_center_collisions");
    }
    return true;
}

RRTConnectConfig bridge_pair_box_rrt(const RRTConnectConfig& pair_rrt,
                                     BoxOracle& oracle,
                                     const IslandConnectorConfig& config,
                                     const Eigen::Ref<const Eigen::VectorXd>& source_center,
                                     const Eigen::Ref<const Eigen::VectorXd>& target_center) {
    RRTConnectConfig box_rrt =
        with_query_root_hull_domain(pair_rrt, oracle, source_center, target_center);
    if (config.pave.require_connected_chain) {
        box_rrt.shortcut_path = true;
    }
    return box_rrt;
}

}  // namespace

BridgePairExecutionResult run_bridge_pair_tasks(
    const std::vector<BridgePairTask>& candidates,
    const std::unordered_map<int, const BoxNode*>& map,
    BoxOracle& oracle,
    const Robot& robot,
    const CollisionChecker& checker,
    const IslandConnectorConfig& config,
    StageContext& context) {
    BridgePairExecutionResult result;
    const RRTConnectConfig pair_rrt = connector_pair_rrt_config(config);
    const bool run_parallel = context.executor().n_threads() > 1 &&
        static_cast<int>(candidates.size()) >= config.parallel_threshold;
    if (run_parallel) {
        std::vector<BridgePairResult> pair_results(candidates.size());
        auto local_cancel = std::make_shared<std::atomic<bool>>(false);
        context.executor().parallel_for(0, static_cast<int>(candidates.size()), [&](int idx) {
            if (context.should_stop() || local_cancel->load(std::memory_order_relaxed)) {
                return;
            }
            const auto& candidate = candidates[static_cast<std::size_t>(idx)];
            const BoxNode& source_box = *map.at(candidate.source_box_id);
            const BoxNode& target_box = *map.at(candidate.target_box_id);
            const auto& source_center = source_box.center();
            const auto& target_center = target_box.center();
            RRTConnectConfig box_rrt =
                bridge_pair_box_rrt(pair_rrt, oracle, config, source_center, target_center);
            auto path = closest_box_point_segment(source_box,
                                                  target_box,
                                                  checker,
                                                  box_rrt.segment_resolution,
                                                  box_rrt.segment_step);
            if (!path.empty()) {
                context.diagnostics().add_counter("connector.direct_box_segment_successes");
                context.diagnostics().add_counter("connector.rrt_successes");
                pair_results[static_cast<std::size_t>(idx)] =
                    make_successful_bridge_pair_result(candidate, std::move(path));
                if (!config.deterministic_reduce) {
                    local_cancel->store(true, std::memory_order_relaxed);
                }
                return;
            }
            if (record_colliding_centers(checker, source_center, target_center, context)) {
                return;
            }
            if (!config.enable_birrt) {
                context.diagnostics().add_counter("connector.birrt_disabled_skips");
                return;
            }
            context.diagnostics().add_counter("connector.birrt_invocations");
            auto outcome = birrt_connect_impl(
                source_center,
                target_center,
                checker,
                robot,
                box_rrt,
                candidate.source_box_id + candidate.target_box_id + candidate.task_id,
                local_cancel);
            record_birrt_stats(context.diagnostics(), outcome.stats);
            path = std::move(outcome.path);
            if (!path.empty()) {
                context.diagnostics().add_counter("connector.rrt_successes");
                pair_results[static_cast<std::size_t>(idx)] =
                    make_successful_bridge_pair_result(candidate, std::move(path));
                if (!config.deterministic_reduce) {
                    local_cancel->store(true, std::memory_order_relaxed);
                }
            } else {
                context.diagnostics().add_counter("connector.rrt_failures");
            }
        });
        result.attempted_pairs += static_cast<int>(candidates.size());
        for (auto& item : pair_results) {
            if (item.success) {
                result.successful_pairs.push_back(std::move(item));
            }
        }
        return result;
    }

    for (const auto& candidate : candidates) {
        if (context.should_stop()) {
            break;
        }
        result.attempted_pairs += 1;
        const BoxNode& source_box = *map.at(candidate.source_box_id);
        const BoxNode& target_box = *map.at(candidate.target_box_id);
        const auto& source_center = source_box.center();
        const auto& target_center = target_box.center();
        RRTConnectConfig box_rrt =
            bridge_pair_box_rrt(pair_rrt, oracle, config, source_center, target_center);
        auto path = closest_box_point_segment(source_box,
                                              target_box,
                                              checker,
                                              box_rrt.segment_resolution,
                                              box_rrt.segment_step);
        if (!path.empty()) {
            context.diagnostics().add_counter("connector.direct_box_segment_successes");
            context.diagnostics().add_counter("connector.rrt_successes");
            result.successful_pairs.push_back(
                make_successful_bridge_pair_result(candidate, std::move(path)));
            continue;
        }
        if (record_colliding_centers(checker, source_center, target_center, context)) {
            continue;
        }
        if (!config.enable_birrt) {
            context.diagnostics().add_counter("connector.birrt_disabled_skips");
            continue;
        }
        context.diagnostics().add_counter("connector.birrt_invocations");
        path = rrt_connect(
            source_center,
            target_center,
            checker,
            robot,
            context,
            box_rrt,
            candidate.source_box_id + candidate.target_box_id + candidate.task_id);
        if (path.empty()) {
            context.diagnostics().add_counter("connector.rrt_failures");
            continue;
        }
        context.diagnostics().add_counter("connector.rrt_successes");
        result.successful_pairs.push_back(
            make_successful_bridge_pair_result(candidate, std::move(path)));
    }
    return result;
}

}  // namespace rbf
