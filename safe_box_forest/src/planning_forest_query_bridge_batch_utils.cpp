#include "planning_forest_query_bridge_batch_utils.h"

#include "planning_forest_audit.h"
#include "planning_forest_query_utils.h"

#include <SBF/adaptive_grid_partition.h>
#include <SBF/box_graph.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <random>
#include <string>
#include <unordered_map>
#include <utility>

namespace rbf {

void add_query_bridge_oracle_counter_delta(BuildProfile& profile,
                                           const OracleCounters& before,
                                           const OracleCounters& after) {
    auto add_counter_delta = [&](const std::string& key, auto after_value, auto before_value) {
        profile.diagnostics[key] += static_cast<double>(after_value - before_value);
    };
    add_counter_delta("query_bridge.oracle_node_validations",
                      after.node_validations,
                      before.node_validations);
    add_counter_delta("query_bridge.oracle_validation_cache_hits",
                      after.validation_cache_hits,
                      before.validation_cache_hits);
    add_counter_delta("query_bridge.oracle_validation_cache_misses",
                      after.validation_cache_misses,
                      before.validation_cache_misses);
    add_counter_delta("query_bridge.oracle_materializations",
                      after.materializations,
                      before.materializations);
    add_counter_delta("query_bridge.oracle_external_exact_hits",
                      after.materialization_external_exact_hits,
                      before.materialization_external_exact_hits);
    add_counter_delta("query_bridge.oracle_external_exact_misses",
                      after.materialization_external_exact_misses,
                      before.materialization_external_exact_misses);
    add_counter_delta("query_bridge.oracle_interval_replay_compatibility_checks",
                      after.interval_replay_compatibility_checks,
                      before.interval_replay_compatibility_checks);
    add_counter_delta("query_bridge.oracle_interval_replay_compatible",
                      after.interval_replay_compatible,
                      before.interval_replay_compatible);
    add_counter_delta("query_bridge.oracle_interval_replay_incompatible",
                      after.interval_replay_incompatible,
                      before.interval_replay_incompatible);
    add_counter_delta("query_bridge.oracle_interval_replay_direct_exact_hits",
                      after.interval_replay_direct_exact_hits,
                      before.interval_replay_direct_exact_hits);
    add_counter_delta("query_bridge.oracle_interval_replay_key_only_blocked",
                      after.interval_replay_key_only_blocked,
                      before.interval_replay_key_only_blocked);
    add_counter_delta("query_bridge.oracle_shared_endpoint_cache_hits",
                      after.materialization_reused_shared_endpoint_cache,
                      before.materialization_reused_shared_endpoint_cache);
    add_counter_delta("query_bridge.oracle_endpoint_path_ms",
                      after.validate_node_endpoint_path_time_us * 1.0e-3,
                      before.validate_node_endpoint_path_time_us * 1.0e-3);
    add_counter_delta("query_bridge.oracle_classify_ms",
                      after.validate_node_classify_time_us * 1.0e-3,
                      before.validate_node_classify_time_us * 1.0e-3);
    add_counter_delta("query_bridge.oracle_validate_total_ms",
                      after.validate_node_total_time_us * 1.0e-3,
                      before.validate_node_total_time_us * 1.0e-3);
    add_counter_delta("query_bridge.oracle_materialization_endpoint_ms",
                      after.materialization_endpoint_time_us * 1.0e-3,
                      before.materialization_endpoint_time_us * 1.0e-3);
    add_counter_delta("query_bridge.oracle_materialization_envelope_ms",
                      after.materialization_envelope_time_us * 1.0e-3,
                      before.materialization_envelope_time_us * 1.0e-3);
    add_counter_delta("query_bridge.oracle_envelope_collision_queries",
                      after.envelope_collision_queries,
                      before.envelope_collision_queries);
    add_counter_delta("query_bridge.oracle_envelope_gjk_tests",
                      after.envelope_collision_gjk_tests,
                      before.envelope_collision_gjk_tests);
}

std::string query_bridge_task_key(std::size_t index, const std::string& suffix) {
    return "query_bridge.batch_task." + std::to_string(index) + "." + suffix;
}

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
    config.optimize_after_first_iters = options.rrt_optimize_after_first_iters;
    const int effective_fixed_iters =
        override_fixed_iters > 0 ? override_fixed_iters : options.rrt_fixed_iters;
    if (effective_fixed_iters > 0) {
        config.max_iters = effective_fixed_iters;
        config.timeout_ms = options.rrt_fixed_timeout_ms;
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

bool query_bridge_should_check_current_query(
    const QueryBridgeSearchTask& task,
    bool respect_forced,
    const QueryBridgeIndexOptions& index_options,
    const QueryBridgeRetryOptions& retry_options) {
    if (query_bridge_index_segment_only(index_options, task.index)) {
        return false;
    }
    if (respect_forced && query_bridge_index_forced(index_options, task.index)) {
        return false;
    }
    return retry_options.skip_deferred_short_edges;
}

void query_bridge_mark_task_skip(BuildProfile& profile,
                                 std::size_t index,
                                 double code,
                                 const char* reason) {
    profile.diagnostics[query_bridge_task_key(index, "skip_reason_code")] = code;
    if (reason != nullptr && reason[0] != '\0') {
        profile.diagnostics[std::string("query_bridge.batch_task_skip.") + reason] += 1.0;
    }
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
    plan.partition_path_first =
        task.waypoint_path_from_partition_query && !task.waypoint_path.empty();
    plan.effective_attempts =
        plan.partition_path_first ? 0 : plan.base_attempts;
    if (plan.effective_attempts > 0 &&
        !options.local_radius_schedule.empty() &&
        options.local_radius_append_unrestricted_attempt) {
        plan.effective_attempts =
            std::max(plan.effective_attempts,
                     static_cast<int>(options.local_radius_schedule.size()) + 1);
    }
    return plan;
}

std::vector<Eigen::VectorXd> query_bridge_direct_line_fallback_path(
    const QueryBridgeSearchTask& task,
    const Robot& audit_robot,
    const Scene& scene,
    const QueryConfig& query_config,
    const QueryBridgeDirectLineFallbackOptions& options,
    StageContext& context) {
    if (!options.enabled) {
        return {};
    }
    context.diagnostics().add_counter("query_bridge.direct_line_on_no_path_attempts");
    CollisionChecker checker = make_audit_checker(audit_robot, scene, query_config);
    std::vector<Eigen::VectorXd> path{task.start, task.goal};
    const PathAuditCheck audit =
        audit_waypoint_path(path,
                            checker,
                            query_config.audit_resolution,
                            query_config.audit_segment_step);
    if (!audit.passed) {
        context.diagnostics().add_counter("query_bridge.direct_line_on_no_path_rejects");
        return {};
    }
    context.diagnostics().add_counter("query_bridge.direct_line_on_no_path_successes");
    return path;
}

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

std::vector<Eigen::VectorXd> query_bridge_deterministic_detour_fallback_path(
    const QueryBridgeSearchTask& task,
    const Robot& audit_robot,
    const Scene& scene,
    const QueryConfig& query_config,
    const std::vector<Interval>& planning_domain,
    const QueryBridgeDetourOptions& options,
    int rng_seed_base,
    StageContext& context) {
    if (!options.enabled ||
        task.start.size() != task.goal.size() ||
        task.start.size() <= 0 ||
        static_cast<int>(planning_domain.size()) != task.start.size()) {
        return {};
    }
    CollisionChecker checker = make_audit_checker(audit_robot, scene, query_config);
    const Eigen::VectorXd delta = task.goal - task.start;
    const double direct_length = delta.norm();
    if (direct_length <= 1e-9) {
        return {};
    }
    std::vector<int> dims(static_cast<std::size_t>(task.start.size()));
    std::iota(dims.begin(), dims.end(), 0);
    std::sort(dims.begin(), dims.end(), [&](int lhs, int rhs) {
        const double lhs_width = std::max(1e-9, planning_domain[static_cast<std::size_t>(lhs)].width());
        const double rhs_width = std::max(1e-9, planning_domain[static_cast<std::size_t>(rhs)].width());
        const double lhs_along = std::abs(delta[lhs]) / lhs_width;
        const double rhs_along = std::abs(delta[rhs]) / rhs_width;
        if (std::abs(lhs_along - rhs_along) > 1e-12) {
            return lhs_along < rhs_along;
        }
        return lhs < rhs;
    });
    const int dim_limit = std::min<int>(options.dims, static_cast<int>(dims.size()));
    const int rounds = options.rounds;
    const int max_candidates = options.max_candidates;
    const bool multi_axis_detour = options.multi_axis;
    const int random_candidates = options.random_candidates;
    const double base_offset = options.offset;
    const double two_bend_alpha = options.two_bend_alpha;
    const Eigen::VectorXd mid = 0.5 * (task.start + task.goal);
    double best_length = std::numeric_limits<double>::infinity();
    std::vector<Eigen::VectorXd> best_path;
    int candidates = 0;
    auto clamp_to_domain = [&](Eigen::VectorXd point) {
        for (int dim = 0; dim < point.size(); ++dim) {
            point[dim] = std::min(planning_domain[static_cast<std::size_t>(dim)].hi,
                                  std::max(planning_domain[static_cast<std::size_t>(dim)].lo,
                                           point[dim]));
        }
        return point;
    };
    auto try_path = [&](std::vector<Eigen::VectorXd> path) {
        if (candidates >= max_candidates) {
            return;
        }
        ++candidates;
        context.diagnostics().add_counter("query_bridge.detour_on_no_path_candidates");
        double length = path_length(path);
        if (!std::isfinite(length) || length + 1e-12 >= best_length) {
            return;
        }
        const PathAuditCheck audit =
            audit_waypoint_path(path,
                                checker,
                                query_config.audit_resolution,
                                query_config.audit_segment_step);
        if (!audit.passed) {
            context.diagnostics().add_counter("query_bridge.detour_on_no_path_rejects");
            return;
        }
        best_length = length;
        best_path = std::move(path);
    };
    for (int item = 0; item < dim_limit && candidates < max_candidates; ++item) {
        const int dim = dims[static_cast<std::size_t>(item)];
        const double width = planning_domain[static_cast<std::size_t>(dim)].width();
        for (int round = 1; round <= rounds && candidates < max_candidates; ++round) {
            const double magnitude = std::min(0.45 * std::max(0.0, width),
                                              base_offset * static_cast<double>(round));
            if (magnitude <= 1e-9) {
                continue;
            }
            for (double sign : {1.0, -1.0}) {
                Eigen::VectorXd single = mid;
                single[dim] += sign * magnitude;
                single = clamp_to_domain(std::move(single));
                if ((single - mid).norm() > 1e-9) {
                    try_path({task.start, single, task.goal});
                }
                if (candidates >= max_candidates) {
                    break;
                }
                Eigen::VectorXd first = task.start + two_bend_alpha * delta;
                Eigen::VectorXd second = task.start + (1.0 - two_bend_alpha) * delta;
                first[dim] += sign * magnitude;
                second[dim] += sign * magnitude;
                first = clamp_to_domain(std::move(first));
                second = clamp_to_domain(std::move(second));
                if ((first - (task.start + two_bend_alpha * delta)).norm() > 1e-9 ||
                    (second - (task.start + (1.0 - two_bend_alpha) * delta)).norm() > 1e-9) {
                    try_path({task.start, first, second, task.goal});
                }
                if (candidates >= max_candidates) {
                    break;
                }
            }
        }
    }
    if (multi_axis_detour && dim_limit >= 2) {
        for (int first_item = 0; first_item < dim_limit && candidates < max_candidates; ++first_item) {
            const int first_dim = dims[static_cast<std::size_t>(first_item)];
            const double first_width = planning_domain[static_cast<std::size_t>(first_dim)].width();
            for (int second_item = first_item + 1;
                 second_item < dim_limit && candidates < max_candidates;
                 ++second_item) {
                const int second_dim = dims[static_cast<std::size_t>(second_item)];
                const double second_width = planning_domain[static_cast<std::size_t>(second_dim)].width();
                for (int round = 1; round <= rounds && candidates < max_candidates; ++round) {
                    const double first_mag = std::min(0.35 * std::max(0.0, first_width),
                                                      base_offset * static_cast<double>(round));
                    const double second_mag = std::min(0.35 * std::max(0.0, second_width),
                                                       base_offset * static_cast<double>(round));
                    if (first_mag <= 1e-9 || second_mag <= 1e-9) {
                        continue;
                    }
                    for (double first_sign : {1.0, -1.0}) {
                        for (double second_sign : {1.0, -1.0}) {
                            Eigen::VectorXd single = mid;
                            single[first_dim] += first_sign * first_mag;
                            single[second_dim] += second_sign * second_mag;
                            single = clamp_to_domain(std::move(single));
                            if ((single - mid).norm() > 1e-9) {
                                try_path({task.start, single, task.goal});
                            }
                            if (candidates >= max_candidates) {
                                break;
                            }
                            Eigen::VectorXd first = task.start + two_bend_alpha * delta;
                            Eigen::VectorXd second = task.start + (1.0 - two_bend_alpha) * delta;
                            first[first_dim] += first_sign * first_mag;
                            first[second_dim] += second_sign * second_mag;
                            second[first_dim] += first_sign * first_mag;
                            second[second_dim] += second_sign * second_mag;
                            first = clamp_to_domain(std::move(first));
                            second = clamp_to_domain(std::move(second));
                            try_path({task.start, first, second, task.goal});
                            if (candidates >= max_candidates) {
                                break;
                            }
                        }
                        if (candidates >= max_candidates) {
                            break;
                        }
                    }
                }
            }
        }
    }
    if (random_candidates > 0 && dim_limit > 0 && candidates < max_candidates) {
        const int random_budget = std::min(random_candidates, max_candidates - candidates);
        std::mt19937 rng(static_cast<std::uint32_t>(
            derived_planner_seed(rng_seed_base,
                                 kSeedBatchBridgeOffset,
                                 static_cast<int>(task.index),
                                 task.query_index,
                                 41443)));
        std::uniform_int_distribution<int> dim_pick(0, dim_limit - 1);
        std::uniform_real_distribution<double> unit(-1.0, 1.0);
        const double max_scale = std::max(1.0, static_cast<double>(rounds));
        for (int sample = 0; sample < random_budget && candidates < max_candidates; ++sample) {
            const int first_dim = dims[static_cast<std::size_t>(dim_pick(rng))];
            int second_dim = first_dim;
            if (dim_limit > 1) {
                for (int guard = 0; guard < 4 && second_dim == first_dim; ++guard) {
                    second_dim = dims[static_cast<std::size_t>(dim_pick(rng))];
                }
            }
            Eigen::VectorXd offset = Eigen::VectorXd::Zero(task.start.size());
            auto apply_random_dim = [&](int dim) {
                const double width = planning_domain[static_cast<std::size_t>(dim)].width();
                const double limit = std::min(0.35 * std::max(0.0, width),
                                              base_offset * max_scale);
                if (limit > 1e-9) {
                    offset[dim] += unit(rng) * limit;
                }
            };
            apply_random_dim(first_dim);
            if (second_dim != first_dim) {
                apply_random_dim(second_dim);
            }
            if (offset.norm() <= 1e-9) {
                continue;
            }
            if ((sample & 1) == 0) {
                Eigen::VectorXd single = clamp_to_domain(mid + offset);
                try_path({task.start, single, task.goal});
            } else {
                Eigen::VectorXd first = clamp_to_domain(task.start + two_bend_alpha * delta + offset);
                Eigen::VectorXd second = clamp_to_domain(task.start + (1.0 - two_bend_alpha) * delta + offset);
                try_path({task.start, first, second, task.goal});
            }
        }
    }
    context.diagnostics().add_counter("query_bridge.detour_on_no_path_attempts");
    if (!best_path.empty()) {
        context.diagnostics().add_counter("query_bridge.detour_on_no_path_successes");
    }
    return best_path;
}

bool query_bridge_result_acceptable(const QueryResult& current,
                                    const Eigen::VectorXd& start,
                                    const Eigen::VectorXd& goal,
                                    const QueryBridgeAcceptanceThresholds& thresholds) {
    if (!current.success || !current.audit_passed) {
        return false;
    }
    const double raw_length =
        current.raw_path_length > 1e-12 ? current.raw_path_length : current.path_length;
    const double segment_fraction =
        raw_length > 1e-12 ? current.segment_edge_length / raw_length
                           : std::numeric_limits<double>::infinity();
    if (!(segment_fraction <= thresholds.max_segment_fraction)) {
        return false;
    }
    const double direct = (goal - start).norm();
    return direct <= 1e-9 ||
           current.path_length <= std::max(direct * thresholds.path_ratio,
                                            direct + thresholds.path_additive) ||
           current.path_length <= thresholds.max_path_length;
}

void accumulate_query_bridge_direct_corridor_totals(const BuildProfile& profile,
                                                    StageContext& context,
                                                    std::size_t task_index) {
    auto add = [&](const std::string& suffix, const std::string& total_key) {
        const auto it = profile.diagnostics.find(query_bridge_task_key(task_index, suffix));
        if (it != profile.diagnostics.end()) {
            context.diagnostics().add_counter(total_key, it->second);
        }
    };
    add("direct_corridor_ms", "query_bridge.direct_corridor_ms_total");
    add("direct_corridor_samples", "query_bridge.direct_corridor_samples_total");
    add("direct_corridor_ffb_calls", "query_bridge.direct_corridor_ffb_calls_total");
    add("direct_corridor_all_ffb_calls", "query_bridge.direct_corridor_all_ffb_calls_total");
    add("direct_corridor_direct_ffb_ms", "query_bridge.direct_corridor_direct_ffb_ms");
    add("direct_corridor_repair_ffb_ms", "query_bridge.direct_corridor_repair_ffb_ms");
    add("direct_corridor_adaptive_repair_ffb_ms",
        "query_bridge.direct_corridor_adaptive_repair_ffb_ms");
    add("direct_corridor_lateral_repair_ffb_ms",
        "query_bridge.direct_corridor_lateral_repair_ffb_ms");
    add("direct_corridor_segment_audit_ms",
        "query_bridge.direct_corridor_segment_audit_ms");
    add("direct_corridor_added", "query_bridge.direct_corridor_added_total");
    add("direct_corridor_repair_calls", "query_bridge.direct_corridor_repair_calls_total");
    add("direct_corridor_repair_added", "query_bridge.direct_corridor_repair_added_total");
    add("direct_corridor_adaptive_repair_calls",
        "query_bridge.direct_corridor_adaptive_repair_calls_total");
    add("direct_corridor_adaptive_repair_added",
        "query_bridge.direct_corridor_adaptive_repair_added_total");
    add("direct_corridor_lateral_repair_calls",
        "query_bridge.direct_corridor_lateral_repair_calls_total");
    add("direct_corridor_lateral_repair_added",
        "query_bridge.direct_corridor_lateral_repair_added_total");
    add("direct_corridor_bad_initial", "query_bridge.direct_corridor_bad_initial_total");
    add("direct_corridor_bad_final", "query_bridge.direct_corridor_bad_final_total");
    add("direct_corridor_segment_edges", "query_bridge.direct_corridor_segment_edges_total");
    add("direct_corridor_ffb_find_calls", "query_bridge.direct_corridor_ffb_find_calls_total");
    add("direct_corridor_ffb_binary_requested",
        "query_bridge.direct_corridor_ffb_binary_requested_total");
    add("direct_corridor_ffb_virtual_sparse_binary_attempts",
        "query_bridge.direct_corridor_ffb_virtual_sparse_binary_attempts_total");
    add("direct_corridor_ffb_virtual_sparse_binary_successes",
        "query_bridge.direct_corridor_ffb_virtual_sparse_binary_successes_total");
    add("direct_corridor_ffb_virtual_sparse_binary_probes",
        "query_bridge.direct_corridor_ffb_virtual_sparse_binary_probes_total");
    add("direct_corridor_ffb_binary_materialized_fallback_calls",
        "query_bridge.direct_corridor_ffb_binary_materialized_fallback_calls_total");
    add("direct_corridor_ffb_binary_blocked_adaptive_depths",
        "query_bridge.direct_corridor_ffb_binary_blocked_adaptive_depths_total");
    add("direct_corridor_ffb_binary_virtual_unsupported",
        "query_bridge.direct_corridor_ffb_binary_virtual_unsupported_total");
    add("direct_corridor_ffb_linear_descent_calls",
        "query_bridge.direct_corridor_ffb_linear_descent_calls_total");
    add("direct_corridor_transition_connected_ms",
        "query_bridge.direct_corridor_transition_connected_ms");
    add("direct_corridor_transition_connected_calls",
        "query_bridge.direct_corridor_transition_connected_calls");
    add("direct_corridor_bad_transitions_ms",
        "query_bridge.direct_corridor_bad_transitions_ms");
    add("direct_corridor_bad_transitions_calls",
        "query_bridge.direct_corridor_bad_transitions_calls");
    add("direct_corridor_current_cover_ms", "query_bridge.direct_corridor_current_cover_ms");
    add("direct_corridor_current_cover_calls", "query_bridge.direct_corridor_current_cover_calls");
    add("direct_corridor_current_cover_partition_ms",
        "query_bridge.direct_corridor_current_cover_partition_ms");
    add("direct_corridor_current_cover_corridor_scan_ms",
        "query_bridge.direct_corridor_current_cover_corridor_scan_ms");
    add("direct_corridor_current_cover_direct_index_ms",
        "query_bridge.direct_corridor_current_cover_direct_index_ms");
    add("direct_corridor_duplicate_lookup_ms", "query_bridge.direct_corridor_duplicate_lookup_ms");
    add("direct_corridor_duplicate_lookup_calls",
        "query_bridge.direct_corridor_duplicate_lookup_calls");
    add("direct_corridor_commit_total_ms", "query_bridge.direct_corridor_commit_total_ms");
    add("direct_corridor_commit_calls", "query_bridge.direct_corridor_commit_calls");
    add("direct_corridor_commit_dynamic_policy_ms",
        "query_bridge.direct_corridor_commit_dynamic_policy_ms");
    add("direct_corridor_commit_partition_append_ms",
        "query_bridge.direct_corridor_commit_partition_append_ms");
    add("direct_corridor_partition_append_calls",
        "query_bridge.direct_corridor_partition_append_calls");
    add("direct_corridor_partition_append_boxes",
        "query_bridge.direct_corridor_partition_append_boxes");
    add("direct_corridor_assimilate_calls", "query_bridge.direct_corridor_assimilate_calls");
    add("direct_corridor_assimilate_sample_scan_ms",
        "query_bridge.direct_corridor_assimilate_sample_scan_ms");
    add("direct_corridor_assimilate_local_hits",
        "query_bridge.direct_corridor_assimilate_local_hits");
    add("direct_corridor_assimilate_full_scan_fallbacks",
        "query_bridge.direct_corridor_assimilate_full_scan_fallbacks");
    add("direct_corridor_assimilate_local_sample_tests",
        "query_bridge.direct_corridor_assimilate_local_sample_tests");
    add("direct_corridor_assimilate_candidate_build_ms",
        "query_bridge.direct_corridor_assimilate_candidate_build_ms");
    add("direct_corridor_assimilate_adjacency_ms",
        "query_bridge.direct_corridor_assimilate_adjacency_ms");
    add("direct_corridor_segment_insert_ms", "query_bridge.direct_corridor_segment_insert_ms");
    add("direct_corridor_segment_insert_calls",
        "query_bridge.direct_corridor_segment_insert_calls");
    add("direct_corridor_direct_task_build_ms",
        "query_bridge.direct_corridor_direct_task_build_ms");
    add("direct_corridor_direct_loop_ms", "query_bridge.direct_corridor_direct_loop_ms");
    add("direct_corridor_repair_loop_ms", "query_bridge.direct_corridor_repair_loop_ms");
    add("direct_corridor_adaptive_loop_ms", "query_bridge.direct_corridor_adaptive_loop_ms");
    add("direct_corridor_lateral_loop_ms", "query_bridge.direct_corridor_lateral_loop_ms");
    add("direct_corridor_residual_segment_loop_ms",
        "query_bridge.direct_corridor_residual_segment_loop_ms");
}

}  // namespace rbf
