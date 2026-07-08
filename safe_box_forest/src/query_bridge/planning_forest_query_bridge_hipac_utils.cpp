#include "planning_forest_query_bridge_hipac_utils.h"

#include <SBF/adaptive_grid_partition.h>
#include <SBF/adaptive_leaf_sweep_config.h>

#include "planning_forest_query_bridge_task.h"
#include "../query_runtime/planning_forest_query_utils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace rbf {

namespace {

double query_bridge_point_segment_distance_sq(const Eigen::VectorXd& point,
                                              const Eigen::VectorXd& a,
                                              const Eigen::VectorXd& b) {
    if (point.size() != a.size() || point.size() != b.size()) {
        return std::numeric_limits<double>::infinity();
    }
    const Eigen::VectorXd ab = b - a;
    const double denom = ab.squaredNorm();
    if (denom <= 1e-18) {
        return (point - a).squaredNorm();
    }
    const double t = std::clamp((point - a).dot(ab) / denom, 0.0, 1.0);
    return (point - (a + t * ab)).squaredNorm();
}

double query_bridge_point_polyline_distance_sq(
    const Eigen::VectorXd& point,
    const std::vector<Eigen::VectorXd>& path) {
    if (path.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t index = 1; index < path.size(); ++index) {
        best = std::min(best,
                        query_bridge_point_segment_distance_sq(point,
                                                               path[index - 1],
                                                               path[index]));
    }
    if (path.size() == 1) {
        best = (point - path.front()).squaredNorm();
    }
    return best;
}

}  // namespace

QueryBridgeHipacPrebridgeSelection query_bridge_select_hipac_prebridge_pair(
    const std::vector<AdaptiveGridPartitionComponentPair>& candidate_pairs,
    const std::vector<std::vector<int>>& components,
    int start_box_id,
    int goal_box_id,
    const std::vector<Eigen::VectorXd>& coarse_route,
    double max_pair_distance,
    double route_weight,
    double pair_weight) {
    QueryBridgeHipacPrebridgeSelection selection;
    std::unordered_map<int, int> component_by_box;
    for (std::size_t component_index = 0; component_index < components.size(); ++component_index) {
        for (int box_id : components[component_index]) {
            component_by_box.emplace(box_id, static_cast<int>(component_index));
        }
    }
    const auto start_component_it = component_by_box.find(start_box_id);
    if (start_component_it != component_by_box.end()) {
        selection.start_component = start_component_it->second;
    }
    const auto goal_component_it = component_by_box.find(goal_box_id);
    if (goal_component_it != component_by_box.end()) {
        selection.goal_component = goal_component_it->second;
    }
    const bool has_endpoint_component_target =
        selection.start_component > 0 || selection.goal_component > 0;

    double best_score = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < candidate_pairs.size(); ++index) {
        const auto& pair = candidate_pairs[index];
        if (pair.source_box_id < 0 ||
            pair.target_box_id < 0 ||
            pair.source_point.size() == 0 ||
            pair.target_point.size() == 0 ||
            pair.source_point.size() != pair.target_point.size()) {
            continue;
        }
        const bool matches_endpoint_component =
            (selection.start_component > 0 &&
             pair.source_component_index == selection.start_component) ||
            (selection.goal_component > 0 &&
             pair.source_component_index == selection.goal_component);
        if (has_endpoint_component_target && !matches_endpoint_component) {
            ++selection.endpoint_component_rejects;
            continue;
        }
        const double pair_distance = std::sqrt(std::max(0.0, pair.distance_sq));
        if (max_pair_distance > 0.0 &&
            pair_distance > max_pair_distance + 1e-12) {
            ++selection.distance_rejects;
            continue;
        }
        const Eigen::VectorXd midpoint = 0.5 * (pair.source_point + pair.target_point);
        const double route_distance =
            std::sqrt(std::max(0.0,
                               query_bridge_point_polyline_distance_sq(midpoint,
                                                                       coarse_route)));
        const bool touches_start =
            selection.start_component >= 0 &&
            (pair.source_component_index == selection.start_component ||
             pair.target_component_index == selection.start_component);
        const bool touches_goal =
            selection.goal_component >= 0 &&
            (pair.source_component_index == selection.goal_component ||
             pair.target_component_index == selection.goal_component);
        const double endpoint_bonus = (touches_start ? 0.50 : 0.0) +
                                      (touches_goal ? 0.50 : 0.0);
        const double component_size_bonus =
            0.02 * std::log1p(static_cast<double>(
                std::max(0, pair.source_component_size)));
        const double score = route_weight * route_distance +
                             pair_weight * pair_distance -
                             endpoint_bonus -
                             component_size_bonus;
        ++selection.considered;
        if (score < best_score) {
            best_score = score;
            selection.score = score;
            selection.candidate_index = static_cast<int>(index);
        }
    }
    return selection;
}

QueryBridgeHipacPrebridgeGate query_bridge_hipac_prebridge_gate(
    const AdaptiveLeafSweepConfig& config,
    bool partition_native,
    bool adaptive_partition_query_enabled,
    bool adaptive_partition_ready,
    int resolves_used) {
    QueryBridgeHipacPrebridgeGate gate;
    gate.candidate_limit = std::max(1, config.hipac_online_prebridge_candidate_limit);
    gate.max_pair_distance = std::max(0.0, config.hipac_online_prebridge_max_pair_distance);
    gate.route_weight = std::max(0.0, config.hipac_online_prebridge_route_distance_weight);
    gate.pair_weight = std::max(0.0, config.hipac_online_prebridge_pair_distance_weight);
    gate.enabled = config.hipac_online_connectivity &&
                   config.hipac_online_before_query_bridge &&
                   config.hipac_online_prebridge_portal &&
                   partition_native &&
                   adaptive_partition_query_enabled &&
                   adaptive_partition_ready &&
                   resolves_used < std::max(0, config.hipac_online_max_resolves_per_query);
    return gate;
}

QueryBridgeHipacOnlineGate query_bridge_hipac_online_gate(
    const AdaptiveLeafSweepConfig& config,
    bool partition_native,
    int candidate_path_size,
    int resolves_used) {
    QueryBridgeHipacOnlineGate gate;
    gate.resolve_cap = std::max(0, config.hipac_online_max_resolves_per_query);
    gate.candidate_max_length = std::max(0.0, config.hipac_online_candidate_max_length);
    gate.enabled = config.hipac_online_connectivity &&
                   config.hipac_online_before_query_bridge &&
                   partition_native &&
                   candidate_path_size >= 2 &&
                   resolves_used < gate.resolve_cap;
    return gate;
}

bool query_bridge_hipac_after_rrt_available(
    const AdaptiveLeafSweepConfig& config,
    const QueryBridgeSearchTask& task) {
    const int hipac_resolve_cap =
        std::max(0, config.hipac_online_max_resolves_per_query);
    const bool has_remaining_budget =
        task.hipac_online_resolves_used < hipac_resolve_cap ||
        (config.hipac_online_prebridge_portal &&
         task.hipac_prebridge_resolves_used < hipac_resolve_cap);
    return config.hipac_online_connectivity &&
           !task.waypoint_path.empty() &&
           has_remaining_budget;
}

}  // namespace rbf
