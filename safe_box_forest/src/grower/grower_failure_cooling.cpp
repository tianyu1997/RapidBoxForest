#include <SBF/grower.h>

#include <SBF/runtime.h>

#include "grower_internal.h"

#include <algorithm>

namespace rbf {

bool RrtGrower::node_in_failure_cooling(OracleNodeId node,
                                        int active_depth,
                                        int box_count,
                                        StageContext& context) {
    if (!hard_frontier_stop_loss_enabled() || node < 0) {
        return false;
    }
    auto it = failure_cooling_.find(node);
    if (it == failure_cooling_.end()) {
        return false;
    }
    auto& entry = it->second;
    if (entry.cool_until_box_count <= box_count) {
        if (entry.cool_until_box_count > 0) {
            context.diagnostics().add_counter("grower.failure_cooling_expired");
        }
        failure_cooling_.erase(it);
        return false;
    }
    if (config_.failure_cooling_retry_on_depth_raise && active_depth > entry.max_failed_depth) {
        context.diagnostics().add_counter("grower.failure_cooling_retries_after_stage_raise");
        return false;
    }
    context.diagnostics().add_counter("grower.failure_cooling_hits");
    context.diagnostics().add_counter("grower.failure_cooling_skips");
    if (config_.coverage_first_stop_loss) {
        context.diagnostics().add_counter("grower.hard_frontier_stop_loss_hits");
        context.diagnostics().add_counter("grower.hard_frontier_stop_loss_skips");
        set_grower_max_diagnostic(context,
                           "grower.hard_frontier_remaining_horizon_max",
                           static_cast<double>(entry.cool_until_box_count - box_count));
    }
    set_grower_max_diagnostic(context,
                       "grower.failure_cooling_remaining_horizon_max",
                       static_cast<double>(entry.cool_until_box_count - box_count));
    return true;
}

bool RrtGrower::seed_in_failure_cooling(const Eigen::Ref<const Eigen::VectorXd>& seed,
                                        int active_depth,
                                        int box_count,
                                        StageContext& context,
                                        OracleNodeId* domain_node) {
    OracleNodeId node = kInvalidOracleNodeId;
    if (hard_frontier_stop_loss_enabled()) {
        node = find_leaf_containing(oracle_, seed);
    }
    if (domain_node != nullptr) {
        *domain_node = node;
    }
    return node_in_failure_cooling(node, active_depth, box_count, context);
}

void RrtGrower::record_failure_cooling(const FindFreeBoxResult& result,
                                       OracleNodeId fallback_node,
                                       int active_depth,
                                       int box_count,
                                       StageContext& context) {
    if (!hard_frontier_stop_loss_enabled() || active_depth < config_.failure_cooling_min_depth) {
        return;
    }
    const bool eligible = config_.failure_cooling_unknown_only
        ? result.hit_unknown_depth_cap
        : (result.hit_unknown_depth_cap || result.seed_collision || result.fail_code == 3 || result.fail_code == 6);
    if (!eligible) {
        return;
    }
    const OracleNodeId node = result.node >= 0 ? result.node : fallback_node;
    if (node < 0) {
        return;
    }
    auto& entry = failure_cooling_[node];
    entry.fail_count += 1;
    entry.last_failed_box_count = box_count;
    entry.max_failed_depth = std::max(entry.max_failed_depth, active_depth);
    context.diagnostics().add_counter("grower.failure_cooling_recorded_failures");
    if (config_.coverage_first_stop_loss) {
        context.diagnostics().add_counter("grower.hard_frontier_recorded_failures");
        if (result.hit_unknown_depth_cap) {
            context.diagnostics().add_counter("grower.hard_frontier_unknown_depth_cap_failures");
        }
        set_grower_max_diagnostic(context,
                           "grower.hard_frontier_depth_max",
                           static_cast<double>(active_depth));
        set_grower_max_diagnostic(context,
                           "grower.hard_frontier_node_count_max",
                           static_cast<double>(failure_cooling_.size()));
        set_grower_max_diagnostic(context,
                           "grower.hard_frontier_fail_count_max",
                           static_cast<double>(entry.fail_count));
    }
    set_grower_max_diagnostic(context,
                       "grower.failure_cooling_node_count_max",
                       static_cast<double>(failure_cooling_.size()));
    set_grower_max_diagnostic(context,
                       "grower.failure_cooling_fail_count_max",
                       static_cast<double>(entry.fail_count));
    const int threshold = hard_frontier_failure_threshold();
    if (entry.fail_count >= threshold) {
        const int horizon = hard_frontier_box_horizon();
        entry.cool_until_box_count = std::max(entry.cool_until_box_count, box_count + horizon);
        entry.cooled_at_depth = active_depth;
        context.diagnostics().add_counter("grower.failure_cooling_activated");
        if (config_.coverage_first_stop_loss) {
            context.diagnostics().add_counter("grower.hard_frontier_activated");
            set_grower_max_diagnostic(context,
                               "grower.hard_frontier_cool_until_box_count_max",
                               static_cast<double>(entry.cool_until_box_count));
        }
        set_grower_max_diagnostic(context,
                           "grower.failure_cooling_cool_until_box_count_max",
                           static_cast<double>(entry.cool_until_box_count));
    }
}

void RrtGrower::record_failure_cooling_success(OracleNodeId node,
                                               StageContext& context) {
    if (!hard_frontier_stop_loss_enabled() || node < 0) {
        return;
    }
    if (failure_cooling_.erase(node) > 0) {
        context.diagnostics().add_counter("grower.failure_cooling_success_clears");
        if (config_.coverage_first_stop_loss) {
            context.diagnostics().add_counter("grower.hard_frontier_success_clears");
        }
    }
}

bool RrtGrower::hard_frontier_stop_loss_enabled() const {
    return config_.failure_cooling_enabled || config_.coverage_first_stop_loss;
}

int RrtGrower::hard_frontier_failure_threshold() const {
    if (config_.coverage_first_stop_loss) {
        return std::max(1, config_.hard_frontier_failure_threshold);
    }
    return std::max(1, config_.failure_cooling_threshold);
}

int RrtGrower::hard_frontier_box_horizon() const {
    if (config_.coverage_first_stop_loss) {
        return std::max(1, config_.hard_frontier_box_horizon);
    }
    return std::max(1, config_.failure_cooling_box_horizon);
}

}  // namespace rbf
