#include "planning_forest_query_bridge_hipac_utils.h"

#include <SBF/adaptive_grid_partition.h>
#include <SBF/safe_box_forest.h>

#include "planning_forest_query_bridge_task.h"
#include "planning_forest_query_utils.h"

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

QueryBridgeHipacTransitionGate query_bridge_hipac_transition_gate(
    const AdaptiveLeafSweepConfig& config,
    bool partition_native,
    bool adaptive_partition_query_enabled,
    bool adaptive_partition_ready,
    int waypoint_path_size,
    int resolves_used,
    int task_position_index,
    int query_index) {
    QueryBridgeHipacTransitionGate gate;
    const int resolve_cap = std::max(0, config.hipac_transition_max_attempts_per_query);
    gate.attempt_cap = std::max(1, config.hipac_transition_max_attempts_per_query);
    if (!config.hipac_online_connectivity ||
        !config.hipac_online_before_query_bridge ||
        !config.hipac_online_transition_portal ||
        !partition_native ||
        !adaptive_partition_query_enabled ||
        !adaptive_partition_ready ||
        waypoint_path_size < 2 ||
        resolves_used >= resolve_cap) {
        gate.disabled = true;
        return gate;
    }
    const bool target_index =
        csv_index_list_contains(config.hipac_transition_target_query_indices,
                                task_position_index) ||
        csv_index_list_contains(config.hipac_transition_target_query_indices,
                                query_index);
    if (!target_index) {
        gate.target_rejected = true;
        return gate;
    }
    gate.enabled = true;
    return gate;
}

bool query_bridge_hipac_after_rrt_available(
    const AdaptiveLeafSweepConfig& config,
    const QueryBridgeSearchTask& task) {
    const int hipac_resolve_cap =
        std::max(0, config.hipac_online_max_resolves_per_query);
    const bool has_remaining_budget =
        task.hipac_online_resolves_used < hipac_resolve_cap ||
        (config.hipac_online_transition_portal &&
         task.hipac_transition_resolves_used <
             std::max(0, config.hipac_transition_max_attempts_per_query)) ||
        (config.hipac_online_prebridge_portal &&
         task.hipac_prebridge_resolves_used < hipac_resolve_cap);
    return config.hipac_online_connectivity &&
           !task.waypoint_path.empty() &&
           has_remaining_budget;
}

QueryBridgeHipacTransitionCandidateSet query_bridge_select_hipac_transition_candidates(
    const AdaptiveGridPartition& partition,
    const std::vector<Eigen::VectorXd>& waypoint_path,
    int stride,
    int candidate_limit,
    int min_predicted_edges,
    double max_pair_distance,
    double sample_step,
    bool allow_same_component) {
    QueryBridgeHipacTransitionCandidateSet set;
    if (waypoint_path.size() < 2) {
        return set;
    }

    const auto components = partition.component_box_ids_with_overlay();
    std::unordered_map<int, int> component_by_box;
    for (std::size_t component_index = 0; component_index < components.size(); ++component_index) {
        for (int box_id : components[component_index]) {
            component_by_box.emplace(box_id, static_cast<int>(component_index));
        }
    }

    const int effective_stride = std::max(1, stride);
    const int effective_candidate_limit = std::max(1, candidate_limit);
    const int effective_min_edges = std::max(0, min_predicted_edges);
    const double effective_max_pair_distance = std::max(0.0, max_pair_distance);
    const double effective_sample_step = std::max(1.0e-9, sample_step);

    set.candidates.reserve(static_cast<std::size_t>(effective_candidate_limit));
    for (std::size_t begin = 0; begin + 1 < waypoint_path.size(); ++begin) {
        const std::size_t end =
            std::min(waypoint_path.size() - 1,
                     begin + static_cast<std::size_t>(effective_stride));
        if (end <= begin) {
            continue;
        }
        const auto source_nearest = partition.nearest_boxes(waypoint_path[begin], {}, 1);
        const auto target_nearest = partition.nearest_boxes(waypoint_path[end], {}, 1);
        if (source_nearest.empty() || target_nearest.empty()) {
            ++set.gated;
            continue;
        }
        const auto& source = source_nearest.front();
        const auto& target = target_nearest.front();
        if (source.box_id < 0 ||
            target.box_id < 0 ||
            source.box_id == target.box_id ||
            source.closest_point.size() == 0 ||
            target.closest_point.size() == 0 ||
            source.closest_point.size() != target.closest_point.size()) {
            ++set.gated;
            continue;
        }
        const auto source_component_it = component_by_box.find(source.box_id);
        const int source_component =
            source_component_it == component_by_box.end() ? -1 : source_component_it->second;
        const auto target_component_it = component_by_box.find(target.box_id);
        const int target_component =
            target_component_it == component_by_box.end() ? -1 : target_component_it->second;
        if (!allow_same_component &&
            source_component >= 0 &&
            source_component == target_component) {
            ++set.same_component_gated;
            continue;
        }
        const double pair_distance = (target.closest_point - source.closest_point).norm();
        if (effective_max_pair_distance > 0.0 &&
            pair_distance > effective_max_pair_distance + 1e-12) {
            ++set.distance_gated;
            continue;
        }
        double local_length = 0.0;
        for (std::size_t index = begin + 1; index <= end; ++index) {
            local_length += (waypoint_path[index] - waypoint_path[index - 1]).norm();
        }
        const int predicted_edges =
            static_cast<int>(std::ceil(local_length / effective_sample_step));
        if (predicted_edges < effective_min_edges) {
            ++set.edge_gated;
            continue;
        }

        QueryBridgeHipacTransitionCandidate candidate;
        candidate.source_box_id = source.box_id;
        candidate.target_box_id = target.box_id;
        candidate.source_component = source_component;
        candidate.target_component = target_component;
        candidate.first_waypoint = static_cast<int>(begin);
        candidate.last_waypoint = static_cast<int>(end);
        candidate.source_point = source.closest_point;
        candidate.target_point = target.closest_point;
        candidate.pair_distance = pair_distance;
        candidate.local_length = local_length;
        candidate.predicted_bridge_edges = predicted_edges;
        candidate.local_path.reserve(end - begin + 2);
        candidate.local_path.push_back(candidate.source_point);
        for (std::size_t index = begin + 1; index < end; ++index) {
            candidate.local_path.push_back(waypoint_path[index]);
        }
        candidate.local_path.push_back(candidate.target_point);
        candidate.score =
            static_cast<double>(predicted_edges) -
            0.25 * pair_distance -
            0.05 * static_cast<double>(std::abs(target_component - source_component));
        set.candidates.push_back(std::move(candidate));
    }

    std::sort(set.candidates.begin(),
              set.candidates.end(),
              [](const QueryBridgeHipacTransitionCandidate& lhs,
                 const QueryBridgeHipacTransitionCandidate& rhs) {
        if (std::abs(lhs.score - rhs.score) > 1e-12) {
            return lhs.score > rhs.score;
        }
        if (std::abs(lhs.local_length - rhs.local_length) > 1e-12) {
            return lhs.local_length > rhs.local_length;
        }
        return lhs.first_waypoint < rhs.first_waypoint;
    });
    if (static_cast<int>(set.candidates.size()) > effective_candidate_limit) {
        set.candidates.resize(static_cast<std::size_t>(effective_candidate_limit));
    }
    return set;
}

}  // namespace rbf
