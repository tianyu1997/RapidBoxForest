#include <SBF/safe_box_forest.h>
#include <SBF/runtime.h>

#include <SBF/connector.h>
#include <SBF/oracle.h>

#include "planning_forest_query_bridge_pave_guard.h"
#include "../query_runtime/planning_forest_query_utils.h"

#include <algorithm>
#include <chrono>
#include <string>

namespace rbf {

int RBFPlanningForest::locate_query_bridge_box(
    const Eigen::Ref<const Eigen::VectorXd>& point) const {
    if (partition_native_mode()) {
        return locate_box_partition_first(point, config_.query.nearest_if_outside);
    }
    for (const auto& box : boxes_) {
        if (box.contains(point, config_.query.adjacency_tolerance)) {
            return box.id;
        }
    }
    if (!config_.query.nearest_if_outside) {
        return -1;
    }
    return locate_box_partition_first(point, config_.query.nearest_if_outside);
}

bool RBFPlanningForest::query_bridge_box_contains_point(
    int box_id,
    const Eigen::Ref<const Eigen::VectorXd>& point) const {
    if (box_id < 0) {
        return false;
    }
    const BoxNode* box = find_box_by_id(boxes_, box_id);
    return box != nullptr &&
           intervals_contain_point_local(box->joint_intervals,
                                         point,
                                         config_.query.adjacency_tolerance);
}

int RBFPlanningForest::refresh_query_bridge_box_or_anchor(
    int anchor_box_id,
    const Eigen::Ref<const Eigen::VectorXd>& point,
    const char* endpoint_name) {
    const int located = locate_query_bridge_box(point);
    if (located >= 0) {
        return located;
    }
    if (query_bridge_box_contains_point(anchor_box_id, point)) {
        last_build_.diagnostics[std::string("query_bridge.endpoint_anchor_keep_after_lookup_miss.") +
                                endpoint_name] += 1.0;
        return anchor_box_id;
    }
    return -1;
}

void RBFPlanningForest::sync_query_bridge_partition_boxes(
    std::size_t& partition_refresh_base,
    const char* diagnostic_prefix) {
    if (partition_native_mode() &&
        adaptive_partition_query_enabled_ &&
        adaptive_partition_ &&
        boxes_.size() > partition_refresh_base) {
        append_adaptive_partition_boxes(partition_refresh_base,
                                        &last_build_,
                                        diagnostic_prefix);
        partition_refresh_base = boxes_.size();
    }
}

std::pair<int, int> RBFPlanningForest::locate_query_bridge_boxes(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    StageContext& context) {
    using Clock = std::chrono::steady_clock;
    const auto locate_t0 = Clock::now();
    auto locate_box_linear = [&](const Eigen::Ref<const Eigen::VectorXd>& point) {
        for (const auto& box : boxes_) {
            if (intervals_contain_point_local(box.joint_intervals,
                                              point,
                                              config_.query.adjacency_tolerance)) {
                return box.id;
            }
        }
        return -1;
    };
    int source_box_id = -1;
    int target_box_id = -1;
    if (partition_native_mode()) {
        source_box_id =
            locate_box_partition_first(start, config_.query.nearest_if_outside);
        target_box_id =
            locate_box_partition_first(goal, config_.query.nearest_if_outside);
        context.diagnostics().add_counter(
            "query_bridge.locate_query_boxes_partition_first");
    } else {
        source_box_id = locate_box_linear(start);
        target_box_id = locate_box_linear(goal);
        if ((source_box_id < 0 || target_box_id < 0) && config_.query.nearest_if_outside) {
            context.diagnostics().add_counter(
                "query_bridge.locate_query_boxes_cache_fallbacks");
            invalidate_query_cache();
            source_box_id =
                locate_box_partition_first(start, config_.query.nearest_if_outside);
            target_box_id =
                locate_box_partition_first(goal, config_.query.nearest_if_outside);
        }
    }
    context.diagnostics().record_timing(
        "query_bridge.locate_query_boxes_ms",
        std::chrono::duration<double, std::milli>(Clock::now() -
                                                  locate_t0).count());
    return {source_box_id, target_box_id};
}

int RBFPlanningForest::run_query_bridge_chain_pave(
    const std::vector<Eigen::VectorXd>& waypoint_path,
    int start_box_id,
    int& next_id,
    StageContext& context,
    const ChainPaveConfig& pave_config,
    const char* partition_prefix) {
    if (!oracle_) {
        return 0;
    }
    const std::size_t boxes_before = boxes_.size();
    const int added = chain_pave_along_path(waypoint_path,
                                            start_box_id,
                                            boxes_,
                                            *oracle_,
                                            adjacency_,
                                            next_id,
                                            context,
                                            pave_config);
    if (added > 0) {
        append_adaptive_partition_boxes(boxes_before,
                                        &last_build_,
                                        partition_prefix);
        context.diagnostics().add_counter(
            "query_bridge.full_adjacency_rebuilds_avoided");
        invalidate_query_cache();
    }
    return added;
}

std::pair<int, int> RBFPlanningForest::run_query_bridge_reverse_boundary_pave(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    const std::vector<Eigen::VectorXd>& waypoint_path,
    const ChainPaveConfig& forward_config,
    int forward_added,
    int& accumulated_added,
    int& next_id,
    StageContext& context) {
    auto [source_box_id, target_box_id] =
        locate_query_bridge_boxes(start, goal, context);
    if (source_box_id >= 0 && target_box_id >= 0 &&
        box_only_path_connected_partition_first(source_box_id, target_box_id)) {
        return {source_box_id, target_box_id};
    }
    if (query_bridge_skip_graph_pave_for_partition_native(
            partition_native_mode(),
            context,
            "query_bridge.partition_graph_reverse_chain_pave_skipped")) {
        return {source_box_id, target_box_id};
    }
    const int remaining_chain = forward_config.max_chain - std::max(0, forward_added);
    if (target_box_id < 0 || remaining_chain <= 0) {
        return {source_box_id, target_box_id};
    }
    ChainPaveConfig reverse_config = forward_config;
    reverse_config.max_chain = remaining_chain;
    std::vector<Eigen::VectorXd> reverse_path(waypoint_path.rbegin(),
                                              waypoint_path.rend());
    context.diagnostics().add_counter("query_bridge.reverse_boundary_pave_attempts");
    const int reverse_added = run_query_bridge_chain_pave(
        reverse_path,
        target_box_id,
        next_id,
        context,
        reverse_config,
        "query_bridge.reverse_boundary_pave");
    if (reverse_added > 0) {
        accumulated_added += reverse_added;
        context.diagnostics().add_counter("query_bridge.reverse_boundary_pave_added",
                                          static_cast<double>(reverse_added));
    }
    return locate_query_bridge_boxes(start, goal, context);
}

} // namespace rbf
