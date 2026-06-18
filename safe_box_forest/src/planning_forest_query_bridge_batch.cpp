#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>
#include <SBF/connector.h>

#include "planning_forest_audit.h"
#include "planning_forest_diagnostics.h"
#include "planning_forest_qroot_helpers.h"
#include "planning_forest_query_bridge_batch_utils.h"
#include "planning_forest_query_bridge_corridor_utils.h"
#include "planning_forest_query_utils.h"
#include "virtual_sparse_ffb.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace rbf {

std::vector<int> RBFPlanningForest::bridge_queries(const std::vector<Eigen::VectorXd>& starts,
                                                   const std::vector<Eigen::VectorXd>& goals) {
    if (starts.size() != goals.size()) {
        throw std::invalid_argument("bridge_queries requires starts/goals with matching sizes");
    }
    std::vector<int> added_by_query(starts.size(), 0);
    std::size_t partition_refresh_base = boxes_.size();
    const std::size_t segment_edges_before_partition_refresh = segment_edges_.size();
    OracleCounters oracle_counters_before;
    bool oracle_counters_before_valid = false;
    auto finish_batch_bridge = [&]() {
        if (oracle_counters_before_valid && oracle_) {
            const auto after = oracle_->counters();
            add_query_bridge_oracle_counter_delta(last_build_, oracle_counters_before, after);
        }
        const bool changed = boxes_.size() != partition_refresh_base ||
                             segment_edges_.size() != segment_edges_before_partition_refresh ||
                             std::any_of(added_by_query.begin(),
                                         added_by_query.end(),
                                         [](int added) { return added > 0; });
        if (boxes_.size() > partition_refresh_base) {
            append_adaptive_partition_boxes(partition_refresh_base,
                                            &last_build_,
                                            "query_bridge.batch");
            partition_refresh_base = boxes_.size();
        } else if (changed) {
            sync_adaptive_partition_segment_edges(&last_build_, "query_bridge.batch");
            refresh_adaptive_partition_diagnostics(&last_build_);
        }
        return added_by_query;
    };
    if (starts.empty() || !oracle_) {
        return added_by_query;
    }
    oracle_counters_before = oracle_->counters();
    oracle_counters_before_valid = true;

    const QueryBridgeAcceptanceThresholds bridge_acceptance =
        query_bridge_acceptance_thresholds_from_env();
    auto query_result_good = [&](const QueryResult& current,
                                 const Eigen::VectorXd& start,
                                 const Eigen::VectorXd& goal) {
        return query_bridge_result_acceptable(current, start, goal, bridge_acceptance);
    };
    const QueryBridgeIndexOptions index_options = query_bridge_index_options_from_env();
    const QueryBridgePartitionPathFirstOptions partition_path_first_options =
        query_bridge_partition_path_first_options_from_env(partition_native_mode());

    std::vector<QueryBridgeSearchTask> tasks;
    tasks.reserve(starts.size());
    for (std::size_t index = 0; index < starts.size(); ++index) {
        if (starts[index].size() != goals[index].size()) {
            throw std::invalid_argument("bridge_queries received a start/goal dimension mismatch");
        }
        const bool forced_task = query_bridge_index_forced(index_options, index);
        QueryResult initial_query;
        bool has_initial_query = false;
        if (!forced_task || partition_path_first_options.enabled) {
            initial_query = query(starts[index], goals[index]);
            has_initial_query = true;
            if (!forced_task && query_result_good(initial_query, starts[index], goals[index])) {
                query_bridge_mark_task_skip(last_build_, index, 1.0, "initial_good");
                continue;
            }
        }
        int start_box_id = locate_query_bridge_box(starts[index]);
        if (start_box_id < 0) {
            start_box_id = anchor_query_endpoint_box_with_diagnostics(starts[index]);
        }
        if (start_box_id < 0) {
            query_bridge_mark_task_skip(last_build_, index, 2.0, "start_anchor_failed");
            continue;
        }
        int goal_box_id = locate_query_bridge_box(goals[index]);
        if (goal_box_id < 0) {
            goal_box_id = anchor_query_endpoint_box_with_diagnostics(goals[index]);
        }
        sync_query_bridge_partition_boxes(partition_refresh_base,
                                          "query_bridge.endpoint_anchor");
        if (start_box_id >= 0) {
            start_box_id = refresh_query_bridge_box_or_anchor(start_box_id,
                                                              starts[index],
                                                              "start");
        }
        if (goal_box_id >= 0) {
            goal_box_id = refresh_query_bridge_box_or_anchor(goal_box_id,
                                                             goals[index],
                                                             "goal");
        }
        if (goal_box_id < 0 || goal_box_id == start_box_id) {
            query_bridge_mark_task_skip(last_build_,
                                        index,
                                        goal_box_id < 0 ? 3.0 : 4.0,
                                        goal_box_id < 0 ? "goal_anchor_failed" : "same_box");
            continue;
        }

        QueryBridgeSearchTask task;
        task.index = index;
        task.query_index = query_bridge_index_global(index_options,
                                                     index,
                                                     static_cast<int>(index));
        last_build_.diagnostics["query_bridge.batch_task." +
                                std::to_string(index) +
                                ".global_index"] = static_cast<double>(task.query_index);
        task.start = starts[index];
        task.goal = goals[index];
        if (last_adaptive_partition_config_.hipac_online_connectivity &&
            has_initial_query &&
            initial_query.success &&
            initial_query.audit_passed &&
            !initial_query.path.empty()) {
            task.hipac_candidate_path = initial_query.path;
        }
        task.bridge_rrt = with_query_root_hull_domain(config_.connector.rrt,
                                                      *oracle_,
                                                      task.start,
                                                      task.goal);
        task.bridge_rrt.segment_resolution =
            std::max(task.bridge_rrt.segment_resolution, config_.query.audit_resolution);
        if (partition_path_first_options.enabled &&
            has_initial_query &&
            initial_query.success &&
            initial_query.audit_passed &&
            !initial_query.path.empty()) {
            last_build_.diagnostics["query_bridge.partition_path_first_initial_success"] += 1.0;
            const double direct = (task.goal - task.start).norm();
            const double raw_length = initial_query.raw_path_length > 1e-12
                ? initial_query.raw_path_length
                : initial_query.path_length;
            const double segment_fraction =
                raw_length > 1e-12
                    ? initial_query.segment_edge_length / raw_length
                    : std::numeric_limits<double>::infinity();
            const bool segment_reasonable =
                std::isfinite(segment_fraction) &&
                segment_fraction <= partition_path_first_options.max_segment_fraction;
            const bool length_reasonable =
                direct <= 1e-9 ||
                initial_query.path_length <= std::max(direct * bridge_acceptance.path_ratio,
                                                      direct + bridge_acceptance.path_additive) ||
                initial_query.path_length <= bridge_acceptance.max_path_length;
            if (!segment_reasonable) {
                last_build_.diagnostics["query_bridge.partition_path_first_reject_segment"] += 1.0;
            }
            if (!length_reasonable) {
                last_build_.diagnostics["query_bridge.partition_path_first_reject_length"] += 1.0;
            }
            if (segment_reasonable &&
                (length_reasonable || partition_path_first_options.allow_long)) {
                task.waypoint_path = initial_query.path;
                task.waypoint_path_from_partition_query = true;
                if (task.hipac_candidate_path.empty()) {
                    task.hipac_candidate_path = initial_query.path;
                }
                last_build_.diagnostics["query_bridge.partition_path_first_accepted"] += 1.0;
            }
        }
        const double bridge_distance = (task.goal - task.start).norm();
        task.short_local_bridge = query_bridge_short_local_distance(bridge_distance);
        if (task.short_local_bridge) {
            query_bridge_configure_short_local_profiles(task.bridge_rrt,
                                                        task.short_local_profiles);
        }
        task.attempts = std::max(1, config_.connector.max_pairs_per_gap);
        tasks.push_back(std::move(task));
    }

    std::stable_sort(tasks.begin(), tasks.end(), [](const QueryBridgeSearchTask& lhs,
                                                    const QueryBridgeSearchTask& rhs) {
        const bool lhs_short = lhs.short_local_bridge;
        const bool rhs_short = rhs.short_local_bridge;
        if (lhs_short != rhs_short) {
            return !lhs_short && rhs_short;
        }
        return lhs.index < rhs.index;
    });

    if (tasks.empty()) {
        return finish_batch_bridge();
    }
    using Clock = std::chrono::steady_clock;
    const auto batch_t0 = Clock::now();
    StageContext batch_context = StageContext::from_runtime(config_.runtime);
    const QueryBridgeEdgeRuntimeOptions edge_options = query_bridge_edge_runtime_options();
    batch_context.diagnostics().set_value("query_bridge.scene_reusable_edges",
                                          edge_options.scene_reusable_edges ? 1.0 : 0.0);
    ScopedStageDiagnosticsFlush batch_diagnostics_flush(last_build_, batch_context);
    auto elapsed_ms_since = [](Clock::time_point t0) {
        return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    };
    auto edge_query_index_for = [&](const QueryBridgeSearchTask& task) {
        return edge_options.scene_reusable_edges ? -1 : task.query_index;
    };
    const bool direct_start_goal_segment =
        edge_options.direct_segment_after_rrt &&
        edge_options.direct_start_goal_segment &&
        config_.connector.segment_edges_enabled &&
        config_.connector.rrt_segment_edges;
    const bool fast_direct_segment_after_rrt =
        edge_options.direct_segment_after_rrt &&
        edge_options.fast_direct_segment_after_rrt &&
        config_.connector.segment_edges_enabled &&
        config_.connector.rrt_segment_edges;
    const double fast_direct_segment_after_rrt_min_length =
        edge_options.direct_segment_after_rrt_min_length;
    batch_context.diagnostics().set_value(
        "query_bridge.direct_segment_after_rrt",
        edge_options.direct_segment_after_rrt ? 1.0 : 0.0);
    batch_context.diagnostics().set_value(
        "query_bridge.direct_segment_after_rrt_min_length",
        edge_options.direct_segment_after_rrt_min_length);
    batch_context.diagnostics().set_value(
        "query_bridge.direct_start_goal_segment",
        direct_start_goal_segment ? 1.0 : 0.0);
    batch_context.diagnostics().set_value(
        "query_bridge.fast_direct_segment_after_rrt",
        fast_direct_segment_after_rrt ? 1.0 : 0.0);
    auto try_direct_start_goal_segment = [&](QueryBridgeSearchTask& task) -> int {
        if (!direct_start_goal_segment || task.direct_start_goal_satisfied) {
            return 0;
        }
        const int source_box_id =
            locate_query_bridge_box(task.start);
        const int target_box_id =
            locate_query_bridge_box(task.goal);
        if (source_box_id < 0 || target_box_id < 0 || source_box_id == target_box_id) {
            batch_context.diagnostics().add_counter(
                "query_bridge.direct_start_goal_segment_missing_endpoint");
            return 0;
        }
        std::vector<Eigen::VectorXd> direct_path{task.start, task.goal};
        batch_context.diagnostics().add_counter(
            "query_bridge.direct_start_goal_segment_attempts");
        batch_context.diagnostics().add_counter(
            query_bridge_task_key(task.index, "direct_start_goal_segment_attempts"));
        CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
        const PathAuditCheck audit =
            audit_waypoint_path(direct_path,
                                checker,
                                config_.query.audit_resolution,
                                config_.query.audit_segment_step);
        if (!audit.passed) {
            batch_context.diagnostics().add_counter(
                "query_bridge.direct_start_goal_segment_audit_rejects");
            batch_context.diagnostics().add_counter(
                query_bridge_task_key(task.index, "direct_start_goal_segment_audit_rejects"));
            return 0;
        }
        const int edge_id = add_segment_edge_partition_first(
            source_box_id,
            target_box_id,
            direct_path,
            SegmentEdgeType::QueryBridge,
            config_.query.audit_resolution,
            SegmentEdgeValidation::CollisionChecked,
            true,
            edge_query_index_for(task));
        if (edge_id < 0) {
            batch_context.diagnostics().add_counter(
                "query_bridge.direct_start_goal_segment_add_fail");
            batch_context.diagnostics().add_counter(
                query_bridge_task_key(task.index, "direct_start_goal_segment_add_fail"));
            return 0;
        }
        task.direct_start_goal_satisfied = true;
        batch_context.diagnostics().add_counter(
            "query_bridge.direct_start_goal_segment_edges");
        batch_context.diagnostics().add_counter(
            query_bridge_task_key(task.index, "direct_start_goal_segment_edges"));
        invalidate_query_cache();
        sync_adaptive_partition_segment_edges(&last_build_,
                                               "query_bridge.direct_start_goal_segment");
        refresh_adaptive_partition_diagnostics(&last_build_);
        return 1;
    };
    auto try_fast_direct_segment_after_rrt = [&](QueryBridgeSearchTask& task) -> int {
        if (!fast_direct_segment_after_rrt || task.waypoint_path.empty()) {
            return 0;
        }
        std::vector<std::vector<Eigen::VectorXd>> candidate_paths;
        candidate_paths.push_back(task.waypoint_path);
        if (edge_options.fast_direct_shortcut && task.waypoint_path.size() > 2) {
            const double before_length = path_length(task.waypoint_path);
            CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
            std::vector<Eigen::VectorXd> shortened =
                collision_shortcut_path(task.waypoint_path,
                                        checker,
                                        collision_shortcut_resolution(config_.query));
            const double after_length = path_length(shortened);
            batch_context.diagnostics().add_counter(
                "query_bridge.fast_direct_segment_after_rrt_shortcut_attempts");
            batch_context.diagnostics().add_counter(
                query_bridge_task_key(task.index, "fast_direct_segment_after_rrt_shortcut_attempts"));
            if (!shortened.empty() && after_length + 1e-12 < before_length) {
                candidate_paths.push_back(std::move(shortened));
                batch_context.diagnostics().add_counter(
                    "query_bridge.fast_direct_segment_after_rrt_shortcut_accepts");
                batch_context.diagnostics().add_counter(
                    "query_bridge.fast_direct_segment_after_rrt_shortcut_delta",
                    before_length - after_length);
                batch_context.diagnostics().add_counter(
                    query_bridge_task_key(task.index, "fast_direct_segment_after_rrt_shortcut_accepts"));
                batch_context.diagnostics().add_counter(
                    query_bridge_task_key(task.index, "fast_direct_segment_after_rrt_shortcut_delta"),
                    before_length - after_length);
            }
            const int random_shortcut_iters = edge_options.fast_direct_random_shortcut_iters;
            const auto& random_source = candidate_paths.back();
            if (random_shortcut_iters > 0 && random_source.size() > 2U) {
                const double random_before_length = path_length(random_source);
                const std::uint32_t shortcut_seed =
                    static_cast<std::uint32_t>(0x9e3779b9U ^
                                               ((static_cast<std::uint32_t>(task.query_index) + 1U) * 2654435761U) ^
                                               (static_cast<std::uint32_t>(task.index + 1U) * 2246822519U) ^
                                               static_cast<std::uint32_t>(random_source.size()));
                std::vector<Eigen::VectorXd> random_shortened =
                    random_collision_shortcut_path(random_source,
                                                   checker,
                                                   collision_shortcut_resolution(config_.query),
                                                   random_shortcut_iters,
                                                   shortcut_seed);
                const double random_after_length = path_length(random_shortened);
                batch_context.diagnostics().add_counter(
                    "query_bridge.fast_direct_segment_after_rrt_random_shortcut_attempts");
                batch_context.diagnostics().add_counter(
                    query_bridge_task_key(task.index, "fast_direct_segment_after_rrt_random_shortcut_attempts"));
                batch_context.diagnostics().add_counter(
                    "query_bridge.fast_direct_segment_after_rrt_random_shortcut_iters",
                    static_cast<double>(random_shortcut_iters));
                if (!random_shortened.empty() &&
                    random_after_length + 1e-12 < random_before_length) {
                    candidate_paths.push_back(std::move(random_shortened));
                    batch_context.diagnostics().add_counter(
                        "query_bridge.fast_direct_segment_after_rrt_random_shortcut_accepts");
                    batch_context.diagnostics().add_counter(
                        "query_bridge.fast_direct_segment_after_rrt_random_shortcut_delta",
                        random_before_length - random_after_length);
                    batch_context.diagnostics().add_counter(
                        query_bridge_task_key(task.index, "fast_direct_segment_after_rrt_random_shortcut_accepts"));
                    batch_context.diagnostics().add_counter(
                        query_bridge_task_key(task.index, "fast_direct_segment_after_rrt_random_shortcut_delta"),
                        random_before_length - random_after_length);
                }
            }
        }
        std::sort(candidate_paths.begin(),
                  candidate_paths.end(),
                  [](const auto& lhs, const auto& rhs) {
                      return path_length(lhs) < path_length(rhs);
                  });
        candidate_paths.erase(std::unique(candidate_paths.begin(),
                                          candidate_paths.end(),
                                          [](const auto& lhs, const auto& rhs) {
                                              if (lhs.size() != rhs.size()) {
                                                  return false;
                                              }
                                              for (std::size_t index = 0; index < lhs.size(); ++index) {
                                                  if ((lhs[index] - rhs[index]).norm() > 1e-12) {
                                                      return false;
                                                  }
                                              }
                                              return true;
                                          }),
                              candidate_paths.end());
        if (candidate_paths.empty() ||
            !(path_length(candidate_paths.front()) >= fast_direct_segment_after_rrt_min_length)) {
            batch_context.diagnostics().add_counter(
                "query_bridge.fast_direct_segment_after_rrt_length_rejects");
            batch_context.diagnostics().add_counter(
                query_bridge_task_key(task.index, "fast_direct_segment_after_rrt_length_rejects"));
            return 0;
        }
        const int source_box_id = locate_query_bridge_box(task.start);
        const int target_box_id = locate_query_bridge_box(task.goal);
        if (source_box_id < 0 || target_box_id < 0 || source_box_id == target_box_id) {
            batch_context.diagnostics().add_counter(
                "query_bridge.fast_direct_segment_after_rrt_missing_endpoint");
            batch_context.diagnostics().add_counter(
                query_bridge_task_key(task.index, "fast_direct_segment_after_rrt_missing_endpoint"));
            return 0;
        }
        CollisionChecker strict_checker = make_audit_checker(audit_robot_, scene_, config_.query);
        int edge_id = -1;
        double added_length = std::numeric_limits<double>::infinity();
        for (std::size_t candidate_index = 0; candidate_index < candidate_paths.size(); ++candidate_index) {
            const auto& candidate_path = candidate_paths[candidate_index];
            if (path_length(candidate_path) + 1e-12 < fast_direct_segment_after_rrt_min_length) {
                continue;
            }
            batch_context.diagnostics().add_counter(
                "query_bridge.fast_direct_segment_after_rrt_add_candidates");
            const PathAuditCheck candidate_audit =
                audit_waypoint_path(candidate_path,
                                    strict_checker,
                                    config_.query.audit_resolution,
                                    config_.query.audit_segment_step);
            if (!candidate_audit.passed) {
                batch_context.diagnostics().add_counter(
                    "query_bridge.fast_direct_segment_after_rrt_candidate_audit_rejects");
                batch_context.diagnostics().add_counter(
                    query_bridge_task_key(task.index, "fast_direct_segment_after_rrt_candidate_audit_rejects"));
                continue;
            }
            edge_id = add_segment_edge_partition_first(
                source_box_id,
                target_box_id,
                candidate_path,
                SegmentEdgeType::QueryBridge,
                task.bridge_rrt.segment_resolution,
                SegmentEdgeValidation::CollisionChecked,
                true,
                edge_query_index_for(task));
            if (edge_id >= 0) {
                added_length = path_length(candidate_path);
                if (candidate_index > 0) {
                    batch_context.diagnostics().add_counter(
                        "query_bridge.fast_direct_segment_after_rrt_fallback_candidate_success");
                }
                break;
            }
            batch_context.diagnostics().add_counter(
                "query_bridge.fast_direct_segment_after_rrt_add_candidate_fail");
        }
        if (edge_id < 0) {
            batch_context.diagnostics().add_counter(
                "query_bridge.fast_direct_segment_after_rrt_add_fail");
            batch_context.diagnostics().add_counter(
                query_bridge_task_key(task.index, "fast_direct_segment_after_rrt_add_fail"));
            return 0;
        }
        invalidate_query_cache();
        batch_context.diagnostics().add_counter(
            "query_bridge.fast_direct_segment_after_rrt_edges");
        batch_context.diagnostics().add_counter(
            query_bridge_task_key(task.index, "fast_direct_segment_after_rrt_edges"));
        batch_context.diagnostics().set_value(
            query_bridge_task_key(task.index, "fast_direct_segment_after_rrt_length"),
            added_length);
        return 1;
    };
	    auto try_hipac_prebridge_portal = [&](QueryBridgeSearchTask& task) -> int {
	        const QueryBridgeHipacPrebridgeGate prebridge_gate =
	            query_bridge_hipac_prebridge_gate(last_adaptive_partition_config_,
	                                             partition_native_mode(),
	                                             adaptive_partition_query_enabled_,
	                                             adaptive_partition_ && !adaptive_partition_->empty(),
	                                             task.hipac_prebridge_resolves_used);
	        if (!prebridge_gate.enabled) {
	            return 0;
	        }
	        std::vector<Eigen::VectorXd> coarse_route = task.hipac_candidate_path;
	        if (coarse_route.size() < 2) {
	            coarse_route = {task.start, task.goal};
	            batch_context.diagnostics().add_counter(
	                "query_bridge.hipac_prebridge_direct_query_route");
	        }
	        if (coarse_route.size() < 2) {
	            return 0;
	        }
	        const auto prebridge_t0 = Clock::now();
	        batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_attempts");
	        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_prebridge_attempt"),
	                                              1.0);
	        const auto candidate_pairs =
	            adaptive_partition_->nearest_component_pairs_to_largest(1,
	                                                                    prebridge_gate.candidate_limit);
	        batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_candidates",
	                                                static_cast<double>(candidate_pairs.size()));
	        if (candidate_pairs.empty()) {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_no_candidates");
	            return 0;
	        }

	        const auto components = adaptive_partition_->component_box_ids_with_overlay();
	        const int start_box_id = locate_box_partition_first(task.start, false);
	        const int goal_box_id = locate_box_partition_first(task.goal, false);

	        const QueryBridgeHipacPrebridgeSelection prebridge_selection =
	            query_bridge_select_hipac_prebridge_pair(candidate_pairs,
	                                                     components,
	                                                     start_box_id,
	                                                     goal_box_id,
	                                                     coarse_route,
	                                                     prebridge_gate.max_pair_distance,
	                                                     prebridge_gate.route_weight,
	                                                     prebridge_gate.pair_weight);
	        batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_considered",
	                                                static_cast<double>(prebridge_selection.considered));
	        batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_distance_rejects",
	                                                static_cast<double>(prebridge_selection.distance_rejects));
	        batch_context.diagnostics().add_counter(
	            "query_bridge.hipac_prebridge_endpoint_component_rejects",
	            static_cast<double>(prebridge_selection.endpoint_component_rejects));
	        if (prebridge_selection.candidate_index < 0 ||
	            prebridge_selection.candidate_index >= static_cast<int>(candidate_pairs.size())) {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_no_candidate_after_filter");
	            return 0;
	        }
	        const AdaptiveGridPartitionComponentPair& best_pair =
	            candidate_pairs[static_cast<std::size_t>(prebridge_selection.candidate_index)];

	        task.hipac_prebridge_resolves_used += 1;
	        batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_portal_attempts");
	        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_prebridge_score"),
	                                              prebridge_selection.score);
	        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_prebridge_pair_distance"),
	                                              std::sqrt(std::max(0.0, best_pair.distance_sq)));
	        std::vector<Eigen::VectorXd> local_path{best_pair.source_point, best_pair.target_point};
	        const int added = add_partition_portal_corridor_overlay(best_pair.source_point,
	                                                                best_pair.target_point,
	                                                                local_path,
	                                                                "query_bridge.hipac_online_prebridge",
	                                                                false,
	                                                                true,
	                                                                edge_query_index_for(task),
	                                                                &last_build_);
	        const double prebridge_ms = elapsed_ms_since(prebridge_t0);
	        batch_context.diagnostics().record_timing("query_bridge.hipac_prebridge_ms_total",
	                                                  prebridge_ms);
	        batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_ms_total",
	                                                prebridge_ms);
	        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_prebridge_ms"),
	                                              prebridge_ms);
	        if (added <= 0) {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_failures");
	            return 0;
	        }
	        batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_added",
	                                                static_cast<double>(added));
	        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_prebridge_added"),
	                                              static_cast<double>(added));
	        added_by_query[task.index] += added;
	        const QueryResult probe_after_prebridge = query(task.start, task.goal);
	        if (query_result_good(probe_after_prebridge, task.start, task.goal)) {
	            task.hipac_online_satisfied = true;
	            batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_satisfied");
	            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_prebridge_satisfied"),
	                                                  1.0);
	        } else {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_not_sufficient");
	        }
	        return added;
	    };
	    auto try_hipac_online_bridge = [&](QueryBridgeSearchTask& task) -> int {
	        const QueryBridgeHipacOnlineGate hipac_online_gate =
	            query_bridge_hipac_online_gate(
	                last_adaptive_partition_config_,
	                partition_native_mode(),
	                static_cast<int>(task.hipac_candidate_path.size()),
	                task.hipac_online_resolves_used);
	        if (!hipac_online_gate.enabled) {
	            return 0;
	        }
	        task.hipac_online_resolves_used += 1;
	        const auto hipac_t0 = Clock::now();
	        batch_context.diagnostics().add_counter("query_bridge.hipac_online_attempts");
	        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_online_attempt"),
	                                              1.0);
	        std::vector<Eigen::VectorXd> hipac_path = task.hipac_candidate_path;
	        if (hipac_path.size() > 2) {
	            CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
	            const double before_length = path_length(hipac_path);
	            std::vector<Eigen::VectorXd> shortened =
	                collision_shortcut_path(hipac_path,
	                                        checker,
	                                        collision_shortcut_resolution(config_.query));
	            if (shortened.size() >= 2 &&
	                path_length(shortened) <= before_length + 1e-12) {
	                const PathAuditCheck audit =
	                    audit_waypoint_path(shortened,
	                                        checker,
	                                        config_.query.audit_resolution,
	                                        config_.query.audit_segment_step);
	                if (audit.passed) {
	                    batch_context.diagnostics().add_counter(
	                        "query_bridge.hipac_online_shortcut_accepts");
	                    batch_context.diagnostics().add_counter(
	                        "query_bridge.hipac_online_shortcut_delta",
	                        std::max(0.0, before_length - path_length(shortened)));
	                    hipac_path = std::move(shortened);
	                } else {
	                    batch_context.diagnostics().add_counter(
	                        "query_bridge.hipac_online_shortcut_audit_rejects");
	                }
	            }
	        }
	        const double hipac_candidate_length = path_length(hipac_path);
	        batch_context.diagnostics().add_counter("query_bridge.hipac_online_candidate_length",
	                                                hipac_candidate_length);
	        if (hipac_online_gate.candidate_max_length > 0.0 &&
	            hipac_candidate_length > hipac_online_gate.candidate_max_length + 1e-12) {
	            batch_context.diagnostics().add_counter(
	                "query_bridge.hipac_online_candidate_length_rejects");
	            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_online_length_reject"),
	                                                  1.0);
	            const double hipac_ms = elapsed_ms_since(hipac_t0);
	            batch_context.diagnostics().record_timing("query_bridge.hipac_online_ms_total",
	                                                      hipac_ms);
	            batch_context.diagnostics().add_counter("query_bridge.hipac_online_ms_total",
	                                                    hipac_ms);
	            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_online_ms"),
	                                                  hipac_ms);
	            batch_context.diagnostics().add_counter("query_bridge.hipac_online_failures");
	            return 0;
	        }
	        int added = add_partition_box_corridor_overlay(task.start,
	                                                       task.goal,
	                                                       hipac_path,
	                                                       "query_bridge.hipac_online",
	                                                       true,
	                                                       false,
	                                                       edge_query_index_for(task),
	                                                       &last_build_);
	        if (added <= 0 &&
	            last_adaptive_partition_config_.hipac_online_ffb_portal_fallback) {
	            added = add_partition_portal_corridor_overlay(task.start,
	                                                          task.goal,
	                                                          hipac_path,
	                                                          "query_bridge.hipac_online",
	                                                          true,
	                                                          false,
	                                                          edge_query_index_for(task),
	                                                          &last_build_);
	        }
	        const double hipac_ms = elapsed_ms_since(hipac_t0);
	        batch_context.diagnostics().record_timing("query_bridge.hipac_online_ms_total",
	                                                  hipac_ms);
	        batch_context.diagnostics().add_counter("query_bridge.hipac_online_ms_total",
	                                                hipac_ms);
	        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_online_ms"),
	                                              hipac_ms);
	        if (added <= 0) {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_online_failures");
	            return 0;
	        }
	        batch_context.diagnostics().add_counter("query_bridge.hipac_online_added",
	                                                static_cast<double>(added));
	        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_online_added"),
	                                              static_cast<double>(added));
	        added_by_query[task.index] += added;
	        const QueryResult probe_after_hipac = query(task.start, task.goal);
	        if (query_result_good(probe_after_hipac, task.start, task.goal)) {
	            task.hipac_online_satisfied = true;
	            batch_context.diagnostics().add_counter("query_bridge.hipac_online_satisfied");
	            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_online_satisfied"),
	                                                  1.0);
	        } else {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_online_not_sufficient");
	        }
	        return added;
	    };
	    auto try_hipac_transition_portal = [&](QueryBridgeSearchTask& task) -> int {
	        const QueryBridgeHipacTransitionGate transition_gate =
	            query_bridge_hipac_transition_gate(
	                last_adaptive_partition_config_,
	                partition_native_mode(),
	                adaptive_partition_query_enabled_,
	                adaptive_partition_ && !adaptive_partition_->empty(),
	                static_cast<int>(task.waypoint_path.size()),
	                task.hipac_transition_resolves_used,
	                static_cast<int>(task.index),
	                task.query_index);
	        if (transition_gate.disabled) {
	            return 0;
	        }
	        if (transition_gate.target_rejected) {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_transition_target_rejects");
	            return 0;
	        }

	        const auto transition_t0 = Clock::now();
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_attempts");
	        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_transition_attempt"),
	                                              1.0);
	        const int stride =
	            std::max(1, last_adaptive_partition_config_.hipac_transition_window_stride);
	        const int candidate_limit =
	            std::max(1, last_adaptive_partition_config_.hipac_transition_candidate_limit);
	        const int min_predicted_edges =
	            std::max(0, last_adaptive_partition_config_.hipac_transition_min_predicted_bridge_edges);
	        const double max_pair_distance =
	            std::max(0.0, last_adaptive_partition_config_.hipac_transition_max_pair_distance);
	        const double sample_step =
	            query_bridge_direct_corridor_runtime_options(
	                task.query_index,
	                config_.query.audit_segment_step).sample_step;
	        const QueryBridgeHipacTransitionCandidateSet transition_candidates =
	            query_bridge_select_hipac_transition_candidates(
	                *adaptive_partition_,
	                task.waypoint_path,
	                stride,
	                candidate_limit,
	                min_predicted_edges,
	                max_pair_distance,
	                sample_step,
	                last_adaptive_partition_config_.hipac_transition_allow_same_component);
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_candidates",
	                                                static_cast<double>(transition_candidates.candidates.size()));
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_gated",
	                                                static_cast<double>(transition_candidates.gated +
	                                                                    transition_candidates.same_component_gated +
	                                                                    transition_candidates.distance_gated +
	                                                                    transition_candidates.edge_gated));
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_gated_same_component",
	                                                static_cast<double>(transition_candidates.same_component_gated));
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_gated_distance",
	                                                static_cast<double>(transition_candidates.distance_gated));
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_gated_edges",
	                                                static_cast<double>(transition_candidates.edge_gated));
	        if (transition_candidates.candidates.empty()) {
	            return 0;
	        }

	        int total_added = 0;
	        int attempts = 0;
	        const int attempt_cap = transition_gate.attempt_cap;
	        for (const auto& candidate : transition_candidates.candidates) {
	            if (attempts >= attempt_cap ||
	                task.hipac_transition_resolves_used >= attempt_cap) {
	                break;
	            }
	            ++attempts;
	            task.hipac_transition_resolves_used += 1;
	            batch_context.diagnostics().add_counter("query_bridge.hipac_transition_portal_attempts");
	            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_transition_predicted_edges"),
	                                                  static_cast<double>(candidate.predicted_bridge_edges));
	            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_transition_pair_distance"),
	                                                  candidate.pair_distance);
	            const int added = add_partition_portal_corridor_overlay(candidate.source_point,
	                                                                    candidate.target_point,
	                                                                    candidate.local_path,
	                                                                    "query_bridge.hipac_online_transition",
	                                                                    false,
	                                                                    false,
	                                                                    edge_query_index_for(task),
	                                                                    &last_build_);
	            if (added <= 0) {
	                batch_context.diagnostics().add_counter("query_bridge.hipac_transition_failures");
	                continue;
	            }
	            total_added += added;
	            batch_context.diagnostics().add_counter("query_bridge.hipac_transition_added",
	                                                    static_cast<double>(added));
	            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_transition_added"),
	                                                  static_cast<double>(added));
	            added_by_query[task.index] += added;
	            const QueryResult probe_after_transition = query(task.start, task.goal);
	            if (probe_after_transition.success &&
	                probe_after_transition.audit_passed &&
	                !probe_after_transition.path.empty()) {
	                task.waypoint_path = probe_after_transition.path;
	                task.hipac_candidate_path = probe_after_transition.path;
	                batch_context.diagnostics().add_counter(
	                    "query_bridge.hipac_transition_probe_path_adopted");
	                batch_context.diagnostics().set_value(
	                    query_bridge_task_key(task.index, "hipac_transition_probe_path_length"),
	                    probe_after_transition.path_length);
	            }
	            if (query_result_good(probe_after_transition, task.start, task.goal)) {
	                task.hipac_online_satisfied = true;
	                batch_context.diagnostics().add_counter("query_bridge.hipac_transition_satisfied");
	                batch_context.diagnostics().set_value(query_bridge_task_key(task.index,
	                                                               "hipac_transition_satisfied"),
	                                                      1.0);
	                break;
	            }
	            batch_context.diagnostics().add_counter("query_bridge.hipac_transition_not_sufficient");
	        }
	        const double transition_ms = elapsed_ms_since(transition_t0);
	        batch_context.diagnostics().record_timing("query_bridge.hipac_transition_ms_total",
	                                                  transition_ms);
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_ms_total",
	                                                transition_ms);
	        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_transition_ms"),
	                                              transition_ms);
	        return total_added;
	    };
	    auto maybe_promote_query_repair = [&](const QueryBridgeSearchTask& task,
	                                          int bridge_added) -> int {
	        if (!last_adaptive_partition_config_.hipac_online_connectivity ||
	            !last_adaptive_partition_config_.hipac_promote_query_repairs ||
	            !partition_native_mode() ||
	            bridge_added <= 0 ||
	            task.waypoint_path.size() < 2) {
	            return 0;
	        }
	        const auto promote_t0 = Clock::now();
	        batch_context.diagnostics().add_counter("query_bridge.hipac_promote_attempts");
	        const int promoted = add_partition_portal_corridor_overlay(task.start,
	                                                                   task.goal,
	                                                                   task.waypoint_path,
	                                                                   "query_bridge.hipac_promote",
	                                                                   true,
	                                                                   false,
	                                                                   edge_query_index_for(task),
	                                                                   &last_build_);
	        const double promote_ms = elapsed_ms_since(promote_t0);
	        batch_context.diagnostics().record_timing("query_bridge.hipac_promote_ms_total",
	                                                  promote_ms);
	        batch_context.diagnostics().add_counter("query_bridge.hipac_promote_ms_total",
	                                                promote_ms);
	        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_promote_ms"),
	                                              promote_ms);
	        if (promoted > 0) {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_promote_added",
	                                                    static_cast<double>(promoted));
	            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_promote_added"),
	                                                  static_cast<double>(promoted));
	            added_by_query[task.index] += promoted;
	        } else {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_promote_failures");
	        }
	        return promoted;
	    };
    auto try_hipac_online_sequence = [&](QueryBridgeSearchTask& task) {
        try_hipac_online_bridge(task);
        if (!task.hipac_online_satisfied) {
            try_hipac_transition_portal(task);
        }
        if (!task.hipac_online_satisfied) {
            try_hipac_prebridge_portal(task);
        }
        return task.hipac_online_satisfied;
    };
    auto mark_hipac_after_rrt_skip = [&](const QueryBridgeSearchTask& task,
                                         double total_ms) {
        batch_context.diagnostics().add_counter(
            "query_bridge.batch_tasks_skipped_by_hipac_after_rrt");
        batch_context.diagnostics().set_value(
            query_bridge_task_key(task.index, "skipped_by_hipac_after_rrt"),
            1.0);
        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "total_ms"),
                                              total_ms);
    };
    batch_context.diagnostics().set_value("query_bridge.batch_tasks_initial",
                                          static_cast<double>(tasks.size()));
    if (direct_start_goal_segment) {
        for (auto& task : tasks) {
            const int added = try_direct_start_goal_segment(task);
            if (added > 0) {
                added_by_query[task.index] += added;
            }
        }
    }
    const QueryBridgeRetryOptions retry_options = query_bridge_retry_options_from_env();
    record_query_bridge_retry_diagnostics(batch_context, retry_options);
    const QueryBridgeBatchExecutionOptions batch_execution_options =
        query_bridge_batch_execution_options_from_env();
    record_query_bridge_batch_execution_diagnostics(batch_context, batch_execution_options);
    const QueryBridgeParallelRrtOptions parallel_rrt_options =
        query_bridge_parallel_rrt_options_from_env();
    record_query_bridge_parallel_rrt_diagnostics(batch_context, parallel_rrt_options);
    record_query_bridge_acceptance_diagnostics(batch_context, bridge_acceptance);
    record_query_bridge_partition_path_first_diagnostics(batch_context,
                                                        partition_path_first_options);
    auto rrt_path_good_enough_for_task = [&](const QueryBridgeSearchTask& task,
                                             const std::vector<Eigen::VectorXd>& path) {
        return query_bridge_parallel_rrt_path_good_enough(task.start,
                                                          task.goal,
                                                          path,
                                                          parallel_rrt_options);
    };
    auto query_bridge_forced = [&](const QueryBridgeSearchTask& task) {
        return query_bridge_index_forced(index_options, task.index);
    };
    auto current_query_good = [&](const QueryBridgeSearchTask& task, bool respect_forced) {
        if (query_bridge_index_segment_only(index_options, task.index)) {
            return false;
        }
        if (respect_forced && query_bridge_forced(task)) {
            return false;
        }
        if (!retry_options.skip_deferred_short_edges) {
            return false;
        }
        return query_result_good(query(task.start, task.goal), task.start, task.goal);
    };
    auto run_task_attempt = [&](const QueryBridgeSearchTask& task,
                                int attempt,
                                int override_fixed_iters,
                                std::shared_ptr<std::atomic<bool>> cancel_override =
                                    std::shared_ptr<std::atomic<bool>>{}) {
        const int scheduled_attempt = attempt + retry_options.attempt_offset;
        Robot bridge_robot = make_sbf_clearance_robot(audit_robot_, retry_options.rrt_clearance);
        CollisionChecker checker =
            retry_options.rrt_clearance > 0.0
                ? CollisionChecker(bridge_robot, scene_)
                : make_audit_checker(audit_robot_, scene_, config_.query);
        RRTConnectConfig config =
            task.short_local_profiles.empty()
                ? task.bridge_rrt
                : task.short_local_profiles[
                      static_cast<std::size_t>(scheduled_attempt) % task.short_local_profiles.size()];
        if (!retry_options.local_radius_schedule.empty() &&
            attempt >= 0 &&
            static_cast<std::size_t>(attempt) < retry_options.local_radius_schedule.size()) {
            const double scheduled_radius =
                retry_options.local_radius_schedule[static_cast<std::size_t>(attempt)];
            if (scheduled_radius >= 0.0) {
                config.local_sampling_radius = scheduled_radius;
            }
        }
        config.optimize_after_first_iters = retry_options.rrt_optimize_after_first_iters;
        const int effective_fixed_iters =
            override_fixed_iters > 0 ? override_fixed_iters : retry_options.rrt_fixed_iters;
        if (effective_fixed_iters > 0) {
            config.max_iters = effective_fixed_iters;
            config.timeout_ms = retry_options.rrt_fixed_timeout_ms;
        } else {
            config.timeout_ms = std::max(1.0, config_.connector.per_pair_timeout_ms);
        }
        std::vector<Eigen::VectorXd> path = rrt_connect(
            task.start,
            task.goal,
            checker,
            bridge_robot,
            config,
                derived_planner_seed(config_.grower.rng_seed,
                                     kSeedBatchBridgeOffset,
                                     scheduled_attempt,
                                     task.query_index,
                                     task.short_local_bridge ? 0 : kSeedAttemptStride),
            cancel_override ? cancel_override : batch_context.native_cancel_flag());
        if (path.empty()) {
            return std::vector<Eigen::VectorXd>{};
        }
        const PathAuditCheck audit =
            audit_waypoint_path(path, checker, config_.query.audit_resolution, config_.query.audit_segment_step);
        if (!audit.passed) {
            return std::vector<Eigen::VectorXd>{};
        }
        return path;
    };
    const QueryBridgeDirectLineFallbackOptions direct_line_options =
        query_bridge_direct_line_fallback_options_from_env();
    record_query_bridge_direct_line_fallback_diagnostics(batch_context, direct_line_options);
    auto direct_line_fallback_path = [&](const QueryBridgeSearchTask& task) {
        return query_bridge_direct_line_fallback_path(task,
                                                      audit_robot_,
                                                      scene_,
                                                      config_.query,
                                                      direct_line_options,
                                                      batch_context);
    };
    const QueryBridgeDetourOptions detour_options = query_bridge_detour_options_from_env();
    record_query_bridge_detour_diagnostics(batch_context, detour_options);
    const auto detour_planning_domain = oracle_->planning_intervals();
    auto deterministic_detour_fallback_path = [&](const QueryBridgeSearchTask& task) {
        return query_bridge_deterministic_detour_fallback_path(task,
                                                              audit_robot_,
                                                              scene_,
                                                              config_.query,
                                                              detour_planning_domain,
                                                              detour_options,
                                                              config_.grower.rng_seed,
                                                              batch_context);
    };
    auto maybe_apply_detour_path = [&](const QueryBridgeSearchTask& task,
                                       double& best_length,
        std::vector<Eigen::VectorXd>& waypoint_path) {
        if (!waypoint_path.empty() && !detour_options.candidate) {
            return false;
        }
        auto detour_path = deterministic_detour_fallback_path(task);
        if (detour_path.empty()) {
            return false;
        }
        const double detour_length = path_length(detour_path);
        if (!waypoint_path.empty() &&
            detour_length > best_length * detour_options.replace_factor + 1e-12) {
            batch_context.diagnostics().add_counter(
                "query_bridge.detour_candidate_not_shorter");
            return false;
        }
        best_length = detour_length;
        waypoint_path = std::move(detour_path);
        batch_context.diagnostics().add_counter(
            "query_bridge.detour_candidate_selected");
        return true;
    };
    const QueryBridgeWaypointQualityRetryOptions quality_retry_options =
        query_bridge_waypoint_quality_retry_options_from_env();
    record_query_bridge_waypoint_quality_retry_diagnostics(batch_context,
                                                           quality_retry_options);
    auto improve_waypoint_if_needed = [&](QueryBridgeSearchTask& task,
                                          int attempts_already_used,
                                          double& best_length,
                                          std::vector<Eigen::VectorXd>& waypoint_path) {
        if (!quality_retry_options.enabled ||
            quality_retry_options.attempts <= 0 ||
            waypoint_path.empty()) {
            return;
        }
        if (!query_bridge_waypoint_quality_retry_needed(task.start,
                                                        task.goal,
                                                        best_length,
                                                        quality_retry_options)) {
            return;
        }
        const double direct = (task.goal - task.start).norm();
        const double limit = std::max(direct * quality_retry_options.max_ratio,
                                      direct + quality_retry_options.max_additive);
        batch_context.diagnostics().add_counter(
            "query_bridge.waypoint_quality_retry_tasks");
        const auto retry_t0 = Clock::now();
        int retry_successes = 0;
        std::vector<std::vector<Eigen::VectorXd>> retry_paths(
            static_cast<std::size_t>(quality_retry_options.attempts));
        if (batch_context.executor().n_threads() > 1 &&
            quality_retry_options.attempts > 1) {
            batch_context.executor().parallel_for(
                0,
                quality_retry_options.attempts,
                [&](int retry) {
                    retry_paths[static_cast<std::size_t>(retry)] =
                        run_task_attempt(task,
                                         attempts_already_used + retry,
                                         quality_retry_options.iters);
                });
        } else {
            for (int retry = 0; retry < quality_retry_options.attempts; ++retry) {
                retry_paths[static_cast<std::size_t>(retry)] =
                    run_task_attempt(task,
                                     attempts_already_used + retry,
                                     quality_retry_options.iters);
            }
        }
        for (auto& retry_path : retry_paths) {
            if (retry_path.empty()) {
                continue;
            }
            retry_successes += 1;
            const double length = path_length(retry_path);
            if (length < best_length) {
                if (!waypoint_path.empty() &&
                    task.waypoint_fallback_paths.size() < 4) {
                    task.waypoint_fallback_paths.push_back(waypoint_path);
                }
                best_length = length;
                waypoint_path = std::move(retry_path);
            }
            if (best_length <= limit) {
                break;
            }
        }
        if (batch_context.executor().n_threads() > 1 &&
            quality_retry_options.attempts > 1) {
            batch_context.diagnostics().add_counter(
                "query_bridge.waypoint_quality_retry_parallel_batches");
        }
        batch_context.diagnostics().add_counter(
            "query_bridge.waypoint_quality_retry_attempts",
            static_cast<double>(quality_retry_options.attempts));
        batch_context.diagnostics().add_counter(
            "query_bridge.waypoint_quality_retry_successes",
            static_cast<double>(retry_successes));
        if (best_length <= limit) {
            batch_context.diagnostics().add_counter(
                "query_bridge.waypoint_quality_retry_fixed");
        }
        batch_context.diagnostics().record_timing(
            "query_bridge.waypoint_quality_retry_ms_total",
            elapsed_ms_since(retry_t0));
    };
    const QueryBridgeHybridizeAttemptOptions hybrid_options =
        query_bridge_hybridize_attempt_options_from_env();
    auto select_attempt_paths = [&](QueryBridgeSearchTask& task,
                                    std::vector<std::vector<Eigen::VectorXd>>& attempt_paths,
                                    double& best_length) {
        std::vector<std::pair<double, std::size_t>> valid_paths;
        valid_paths.reserve(attempt_paths.size());
        for (std::size_t index = 0; index < attempt_paths.size(); ++index) {
            if (attempt_paths[index].empty()) {
                continue;
            }
            const double length = path_length(attempt_paths[index]);
            if (!std::isfinite(length)) {
                continue;
            }
            valid_paths.emplace_back(length, index);
        }
        if (valid_paths.empty()) {
            return;
        }
        if (hybrid_options.enabled && valid_paths.size() >= 2U) {
            CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
            const double best_input_length =
                std::min(best_length,
                         std::min_element(valid_paths.begin(),
                                          valid_paths.end(),
                                          [](const auto& lhs, const auto& rhs) {
                                              return lhs.first < rhs.first;
                                          })
                             ->first);
            std::vector<Eigen::VectorXd> hybrid =
                hybridize_collision_free_paths(attempt_paths,
                                               checker,
                                               collision_shortcut_resolution(config_.query),
                                               hybrid_options.max_paths,
                                               hybrid_options.max_vertices,
                                               hybrid_options.max_cross_checks);
            batch_context.diagnostics().add_counter(
                "query_bridge.hybridize_attempt_paths_tasks");
            if (!hybrid.empty()) {
                const double hybrid_length = path_length(hybrid);
                batch_context.diagnostics().add_counter(
                    "query_bridge.hybridize_attempt_paths_candidates");
                batch_context.diagnostics().add_counter(
                    query_bridge_task_key(task.index, "hybridize_attempt_paths_candidates"));
                if (hybrid_length + 1e-12 < best_input_length) {
                    const PathAuditCheck audit =
                        audit_waypoint_path(hybrid,
                                            checker,
                                            config_.query.audit_resolution,
                                            config_.query.audit_segment_step);
                    if (audit.passed) {
                        const std::size_t index = attempt_paths.size();
                        attempt_paths.push_back(std::move(hybrid));
                        valid_paths.emplace_back(hybrid_length, index);
                        batch_context.diagnostics().add_counter(
                            "query_bridge.hybridize_attempt_paths_accepts");
                        batch_context.diagnostics().add_counter(
                            "query_bridge.hybridize_attempt_paths_delta",
                            best_input_length - hybrid_length);
                        batch_context.diagnostics().add_counter(
                            query_bridge_task_key(task.index, "hybridize_attempt_paths_accepts"));
                    } else {
                        batch_context.diagnostics().add_counter(
                            "query_bridge.hybridize_attempt_paths_audit_rejects");
                    }
                }
            }
        }
        std::sort(valid_paths.begin(), valid_paths.end(),
                  [](const auto& lhs, const auto& rhs) {
                      if (std::abs(lhs.first - rhs.first) > 1e-12) {
                          return lhs.first < rhs.first;
                      }
                      return lhs.second < rhs.second;
                  });
        std::size_t selected_index = std::numeric_limits<std::size_t>::max();
        if (task.waypoint_path.empty() || valid_paths.front().first < best_length) {
            selected_index = valid_paths.front().second;
            if (!task.waypoint_path.empty() &&
                retry_options.attempt_fallback_paths > 0 &&
                task.waypoint_fallback_paths.size() <
                    static_cast<std::size_t>(retry_options.attempt_fallback_paths)) {
                task.waypoint_fallback_paths.push_back(std::move(task.waypoint_path));
                batch_context.diagnostics().add_counter(
                    "query_bridge.attempt_fallback_paths_stored");
            }
            best_length = valid_paths.front().first;
            task.waypoint_path = std::move(attempt_paths[selected_index]);
        }
        for (const auto& [length, index] : valid_paths) {
            (void)length;
            if (index == selected_index || attempt_paths[index].empty()) {
                continue;
            }
            if (retry_options.attempt_fallback_paths <= 0 ||
                task.waypoint_fallback_paths.size() >=
                    static_cast<std::size_t>(retry_options.attempt_fallback_paths)) {
                break;
            }
            task.waypoint_fallback_paths.push_back(std::move(attempt_paths[index]));
            batch_context.diagnostics().add_counter(
                "query_bridge.attempt_fallback_paths_stored");
            batch_context.diagnostics().add_counter(
                query_bridge_task_key(task.index, "attempt_fallback_paths_stored"));
        }
    };
    if (last_adaptive_partition_config_.hipac_online_connectivity &&
        last_adaptive_partition_config_.hipac_online_before_query_bridge) {
        for (auto& task : tasks) {
            try_hipac_online_sequence(task);
        }
    }

    auto bridge_query_with_waypoint_fallbacks =
        [&](QueryBridgeSearchTask& task,
            int& added_accumulator) -> int {
        std::vector<const std::vector<Eigen::VectorXd>*> candidate_paths;
        if (!task.waypoint_path.empty()) {
            candidate_paths.push_back(&task.waypoint_path);
        }
        for (const auto& fallback : task.waypoint_fallback_paths) {
            if (!fallback.empty()) {
                candidate_paths.push_back(&fallback);
            }
        }
        int total_added = 0;
        for (std::size_t candidate_index = 0;
             candidate_index < candidate_paths.size();
             ++candidate_index) {
            const auto& candidate_path = *candidate_paths[candidate_index];
            if (candidate_path.empty()) {
                continue;
            }
            if (candidate_index > 0) {
                batch_context.diagnostics().add_counter(
                    "query_bridge.waypoint_quality_fallback_attempts");
                batch_context.diagnostics().add_counter(
                    query_bridge_task_key(task.index, "waypoint_quality_fallback_attempts"));
            }
            const int bridge_added =
                bridge_query_with_waypoint_path(task.start,
                                                task.goal,
                                                candidate_path,
                                                task.short_local_bridge,
                                                task.bridge_rrt,
                                                task.query_index);
            total_added += bridge_added;
            added_accumulator += bridge_added;
            maybe_promote_query_repair(task, bridge_added);
            accumulate_query_bridge_direct_corridor_totals(last_build_,
                                                           batch_context,
                                                           task.index);
            if (candidate_index > 0) {
                batch_context.diagnostics().add_counter(
                    "query_bridge.waypoint_quality_fallback_added",
                    static_cast<double>(bridge_added));
            }
            if (current_query_good(task, false)) {
                if (candidate_index > 0) {
                    batch_context.diagnostics().add_counter(
                        "query_bridge.waypoint_quality_fallback_successes");
                    batch_context.diagnostics().set_value(
                        query_bridge_task_key(task.index, "waypoint_quality_fallback_success"),
                        1.0);
                    task.waypoint_path = candidate_path;
                }
                if (!batch_execution_options.evaluate_all_fallback_paths) {
                    break;
                }
            }
        }
        return total_added;
    };
    auto mark_batch_task_already_satisfied = [&](const QueryBridgeSearchTask& task,
                                                 Clock::time_point probe_t0) {
        batch_context.diagnostics().add_counter("query_bridge.batch_tasks_skipped");
        batch_context.diagnostics().record_timing("query_bridge.batch_probe_ms_total",
                                                  elapsed_ms_since(probe_t0));
        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "skipped"),
                                              1.0);
        if (task.hipac_online_satisfied) {
            batch_context.diagnostics().set_value(
                query_bridge_task_key(task.index, "skipped_by_hipac_online"),
                1.0);
        }
        if (task.direct_start_goal_satisfied) {
            batch_context.diagnostics().set_value(
                query_bridge_task_key(task.index, "skipped_by_direct_start_goal_segment"),
                1.0);
        }
    };
    auto mark_batch_task_skipped_after_rrt = [&](const QueryBridgeSearchTask& task,
                                                 bool forced_task,
                                                 Clock::time_point probe_t0,
                                                 double total_ms) {
        batch_context.diagnostics().add_counter("query_bridge.batch_tasks_skipped_after_rrt");
        if (forced_task) {
            batch_context.diagnostics().add_counter(
                "query_bridge.batch_forced_tasks_skipped_after_rrt");
        }
        batch_context.diagnostics().record_timing("query_bridge.batch_probe_ms_total",
                                                  elapsed_ms_since(probe_t0));
        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "skipped_after_rrt"),
                                              1.0);
        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "total_ms"),
                                              total_ms);
    };
    auto mark_partition_path_first_task = [&](const QueryBridgeSearchTask& task) {
        batch_context.diagnostics().add_counter(
            "query_bridge.partition_path_first_tasks");
        batch_context.diagnostics().set_value(
            query_bridge_task_key(task.index, "partition_path_first"),
            1.0);
    };
    auto mark_partition_path_first_rrt_skipped = [&](const QueryBridgeSearchTask& task) {
        batch_context.diagnostics().add_counter(
            "query_bridge.partition_path_first_rrt_skipped");
        batch_context.diagnostics().set_value(
            query_bridge_task_key(task.index, "waypoint_from_partition_path"),
            1.0);
    };
    auto mark_batch_task_no_path = [&](const QueryBridgeSearchTask& task,
                                       double total_ms) {
        batch_context.diagnostics().add_counter("query_bridge.batch_tasks_no_path");
        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "no_path"),
                                              1.0);
        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "total_ms"),
                                              total_ms);
    };
    auto task_already_satisfied = [&](const QueryBridgeSearchTask& task) {
        return task.hipac_online_satisfied ||
               task.direct_start_goal_satisfied ||
               current_query_good(task, true);
    };
    auto record_forced_and_attempts = [&](const QueryBridgeSearchTask& task,
                                          bool forced_task,
                                          int attempts) {
        if (forced_task) {
            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "forced"),
                                                  1.0);
        }
        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "attempts"),
                                              static_cast<double>(attempts));
    };
    auto prepare_task_attempts = [&](QueryBridgeSearchTask& task) {
        QueryBridgeAttemptPlan plan =
            query_bridge_attempt_plan(task, query_bridge_forced(task), retry_options);
        if (plan.partition_path_first) {
            mark_partition_path_first_task(task);
        }
        record_forced_and_attempts(task, plan.forced, plan.effective_attempts);
        return plan;
    };
    auto adopt_waypoint_after_rrt =
        [&](QueryBridgeSearchTask& task,
            std::vector<std::vector<Eigen::VectorXd>>& attempt_paths_for_task,
            int improve_attempts,
            double& best_length) {
            select_attempt_paths(task, attempt_paths_for_task, best_length);
            if (task.waypoint_path.empty()) {
                auto direct_path = direct_line_fallback_path(task);
                if (!direct_path.empty()) {
                    best_length = path_length(direct_path);
                    task.waypoint_path = std::move(direct_path);
                    batch_context.diagnostics().set_value(
                        query_bridge_task_key(task.index, "direct_line_on_no_path"),
                        1.0);
                }
            }
            if (maybe_apply_detour_path(task, best_length, task.waypoint_path)) {
                batch_context.diagnostics().set_value(
                    query_bridge_task_key(task.index, "detour_on_no_path"),
                    1.0);
            }
            improve_waypoint_if_needed(task,
                                       improve_attempts,
                                       best_length,
                                       task.waypoint_path);
        };
    auto adopt_retry_path_if_better = [&](QueryBridgeSearchTask& task,
                                          std::vector<Eigen::VectorXd> retry_path,
                                          double& best_length,
                                          int& retry_successes) {
        if (retry_path.empty()) {
            return;
        }
        retry_successes += 1;
        const double length = path_length(retry_path);
        if (length < best_length) {
            best_length = length;
            task.waypoint_path = std::move(retry_path);
        }
    };
    auto run_segment_only_retry = [&](QueryBridgeSearchTask& task,
                                      int first_attempt,
                                      double& best_length) {
        if (!task.waypoint_path.empty() ||
            retry_options.segment_only_retry_attempts <= 0) {
            return;
        }
        const auto retry_t0 = Clock::now();
        int retry_successes = 0;
        for (int retry = 0; retry < retry_options.segment_only_retry_attempts; ++retry) {
            adopt_retry_path_if_better(task,
                                       run_task_attempt(task, first_attempt + retry, 0),
                                       best_length,
                                       retry_successes);
            if (retry_successes > 0 && retry_options.no_path_retry_stop_on_first_success) {
                break;
            }
        }
        const double retry_ms = elapsed_ms_since(retry_t0);
        batch_context.diagnostics().record_timing(
            "query_bridge.batch_segment_only_retry_ms_total",
            retry_ms);
        batch_context.diagnostics().add_counter(
            "query_bridge.batch_segment_only_retry_attempts",
            static_cast<double>(retry_options.segment_only_retry_attempts));
        batch_context.diagnostics().add_counter(
            "query_bridge.batch_segment_only_retry_successes",
            static_cast<double>(retry_successes));
        batch_context.diagnostics().set_value(
            query_bridge_task_key(task.index, "segment_only_retry_attempts"),
            static_cast<double>(retry_options.segment_only_retry_attempts));
        batch_context.diagnostics().set_value(
            query_bridge_task_key(task.index, "segment_only_retry_ms"),
            retry_ms);
        batch_context.diagnostics().set_value(
            query_bridge_task_key(task.index, "segment_only_retry_successes"),
            static_cast<double>(retry_successes));
    };
    auto run_no_path_retries = [&](QueryBridgeSearchTask& task,
                                   int first_attempt,
                                   double& best_length) {
        if (!task.waypoint_path.empty() ||
            (retry_options.no_path_retry_attempts <= 0 &&
             retry_options.no_path_retry_budget_stages == 0)) {
            return;
        }
        int retry_attempt_offset = first_attempt;
        int retry_attempts_total = 0;
        int retry_successes_total = 0;
        double retry_ms_total = 0.0;
        auto run_no_path_retry_stage = [&](int stage_index,
                                           int stage_attempts,
                                           int stage_fixed_iters,
                                           bool adaptive_stage) {
            const int effective_stage_attempts = std::max(0, stage_attempts);
            if (effective_stage_attempts == 0 || !task.waypoint_path.empty()) {
                return;
            }
            const auto retry_t0 = Clock::now();
            int retry_successes = 0;
            int retry_attempts_run = 0;
            for (int retry = 0; retry < effective_stage_attempts; ++retry) {
                adopt_retry_path_if_better(
                    task,
                    run_task_attempt(task,
                                     retry_attempt_offset + retry,
                                     stage_fixed_iters),
                    best_length,
                    retry_successes);
                retry_attempts_run += 1;
                if (retry_successes > 0 &&
                    retry_options.no_path_retry_stop_on_first_success) {
                    break;
                }
            }
            retry_attempt_offset += effective_stage_attempts;
            retry_attempts_total += retry_attempts_run;
            retry_successes_total += retry_successes;
            const double retry_ms = elapsed_ms_since(retry_t0);
            retry_ms_total += retry_ms;
            const std::string key_prefix =
                stage_index == 0
                    ? query_bridge_task_key(task.index, "no_path_retry_")
                    : query_bridge_task_key(
                          task.index,
                          "no_path_retry_stage." + std::to_string(stage_index) + ".");
            batch_context.diagnostics().set_value(
                key_prefix + "attempts",
                static_cast<double>(effective_stage_attempts));
            batch_context.diagnostics().set_value(
                key_prefix + "attempts_run",
                static_cast<double>(retry_attempts_run));
            batch_context.diagnostics().set_value(
                key_prefix + "successes",
                static_cast<double>(retry_successes));
            batch_context.diagnostics().set_value(
                key_prefix + "fixed_iters",
                static_cast<double>(stage_fixed_iters));
            batch_context.diagnostics().set_value(key_prefix + "ms", retry_ms);
            if (adaptive_stage) {
                batch_context.diagnostics().add_counter(
                    "query_bridge.batch_no_path_retry_adaptive_attempts",
                    static_cast<double>(retry_attempts_run));
                batch_context.diagnostics().add_counter(
                    "query_bridge.batch_no_path_retry_adaptive_successes",
                    static_cast<double>(retry_successes));
                batch_context.diagnostics().record_timing(
                    "query_bridge.batch_no_path_retry_adaptive_ms_total",
                    retry_ms);
            }
        };
        run_no_path_retry_stage(0, retry_options.no_path_retry_attempts, 0, false);
        for (std::size_t stage = 0;
             task.waypoint_path.empty() && stage < retry_options.no_path_retry_budget_stages;
             ++stage) {
            run_no_path_retry_stage(
                static_cast<int>(stage) + 1,
                retry_options.no_path_retry_budget_attempts[stage],
                retry_options.no_path_retry_budget_iters[stage],
                true);
        }
        batch_context.diagnostics().record_timing(
            "query_bridge.batch_no_path_retry_ms_total",
            retry_ms_total);
        batch_context.diagnostics().add_counter(
            "query_bridge.batch_no_path_retry_attempts",
            static_cast<double>(retry_attempts_total));
        batch_context.diagnostics().add_counter(
            "query_bridge.batch_no_path_retry_successes",
            static_cast<double>(retry_successes_total));
        batch_context.diagnostics().set_value(
            query_bridge_task_key(task.index, "no_path_retry_attempts"),
            static_cast<double>(retry_attempts_total));
        batch_context.diagnostics().set_value(
            query_bridge_task_key(task.index, "no_path_retry_ms"),
            retry_ms_total);
        batch_context.diagnostics().set_value(
            query_bridge_task_key(task.index, "no_path_retry_successes"),
            static_cast<double>(retry_successes_total));
    };
    auto try_commit_segment_only_task = [&](QueryBridgeSearchTask& task) {
        const int source_box_id = locate_box_partition_first(
            task.start,
            config_.query.nearest_if_outside);
        const int target_box_id = locate_box_partition_first(
            task.goal,
            config_.query.nearest_if_outside);
        int edge_id = -1;
        if (source_box_id >= 0 && target_box_id >= 0) {
            edge_id = add_segment_edge_partition_first(
                source_box_id,
                target_box_id,
                task.waypoint_path,
                SegmentEdgeType::QueryBridge,
                task.bridge_rrt.segment_resolution,
                SegmentEdgeValidation::CollisionChecked,
                true,
                edge_query_index_for(task));
        }
        if (edge_id >= 0) {
            added_by_query[task.index] += 1;
            invalidate_query_cache();
            batch_context.diagnostics().add_counter(
                "query_bridge.batch_tasks_segment_only");
            batch_context.diagnostics().set_value(
                query_bridge_task_key(task.index, "segment_only"),
                1.0);
            batch_context.diagnostics().set_value(
                query_bridge_task_key(task.index, "added"),
                1.0);
            return true;
        }
        batch_context.diagnostics().add_counter(
            "query_bridge.batch_tasks_segment_only_failures");
        batch_context.diagnostics().set_value(
            query_bridge_task_key(task.index, "segment_only_failure"),
            1.0);
        return false;
    };
    auto finish_ready_waypoint_task =
        [&](QueryBridgeSearchTask& task,
            bool forced_task,
            bool segment_only_task,
            double best_length,
            auto&& task_elapsed_ms) {
            batch_context.diagnostics().set_value(
                query_bridge_task_key(task.index, "waypoint_length"),
                best_length);
            const auto second_probe_t0 = Clock::now();
            if (current_query_good(task, !retry_options.post_rrt_skip_forced)) {
                mark_batch_task_skipped_after_rrt(task,
                                                  forced_task,
                                                  second_probe_t0,
                                                  task_elapsed_ms());
                return;
            }
            batch_context.diagnostics().record_timing(
                "query_bridge.batch_probe_ms_total",
                elapsed_ms_since(second_probe_t0));
            if (query_bridge_hipac_after_rrt_available(last_adaptive_partition_config_,
                                                       task)) {
                task.hipac_candidate_path = task.waypoint_path;
                try_hipac_online_sequence(task);
                if (task.hipac_online_satisfied) {
                    mark_hipac_after_rrt_skip(task, task_elapsed_ms());
                    return;
                }
            }
            const int fast_direct_added = try_fast_direct_segment_after_rrt(task);
            if (fast_direct_added > 0) {
                added_by_query[task.index] += fast_direct_added;
                batch_context.diagnostics().set_value(
                    query_bridge_task_key(task.index, "fast_direct_segment_after_rrt"),
                    1.0);
                batch_context.diagnostics().set_value(
                    query_bridge_task_key(task.index, "added"),
                    static_cast<double>(added_by_query[task.index]));
                batch_context.diagnostics().set_value(
                    query_bridge_task_key(task.index, "total_ms"),
                    task_elapsed_ms());
                return;
            }
            if (segment_only_task) {
                try_commit_segment_only_task(task);
                batch_context.diagnostics().set_value(
                    query_bridge_task_key(task.index, "total_ms"),
                    task_elapsed_ms());
                return;
            }
            const auto pave_t0 = Clock::now();
            bridge_query_with_waypoint_fallbacks(task, added_by_query[task.index]);
            const double pave_ms = elapsed_ms_since(pave_t0);
            batch_context.diagnostics().record_timing("query_bridge.batch_pave_ms_total",
                                                      pave_ms);
            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "pave_ms"),
                                                  pave_ms);
            batch_context.diagnostics().set_value(
                query_bridge_task_key(task.index, "added"),
                static_cast<double>(added_by_query[task.index]));
            batch_context.diagnostics().set_value(
                query_bridge_task_key(task.index, "total_ms"),
                task_elapsed_ms());
        };
    auto make_parallel_rrt_cancel_flag = [&]() {
        return parallel_rrt_options.early_stop
            ? std::make_shared<std::atomic<bool>>(false)
            : batch_context.native_cancel_flag();
    };
    auto maybe_stop_parallel_rrt_after_success =
        [&](const QueryBridgeSearchTask& task,
            const std::vector<Eigen::VectorXd>& path,
            std::atomic<int>& early_successes,
            const std::shared_ptr<std::atomic<bool>>& local_cancel) {
            if (!parallel_rrt_options.early_stop ||
                !rrt_path_good_enough_for_task(task, path)) {
                return;
            }
            const int successes =
                early_successes.fetch_add(1, std::memory_order_relaxed) + 1;
            if (successes >= parallel_rrt_options.early_stop_min_successes &&
                local_cancel) {
                local_cancel->store(true, std::memory_order_relaxed);
            }
        };
    auto record_parallel_rrt_early_stop =
        [&](const std::shared_ptr<std::atomic<bool>>& local_cancel,
            const std::atomic<int>& early_successes) {
            if (!parallel_rrt_options.early_stop) {
                return;
            }
            batch_context.diagnostics().add_counter(
                "query_bridge.parallel_rrt_early_stop_successes",
                static_cast<double>(early_successes.load(std::memory_order_relaxed)));
            batch_context.diagnostics().add_counter(
                local_cancel && local_cancel->load(std::memory_order_relaxed)
                    ? "query_bridge.parallel_rrt_early_stop_triggered"
                    : "query_bridge.parallel_rrt_early_stop_not_triggered");
        };
    auto run_attempts_for_task =
        [&](QueryBridgeSearchTask& task,
            int effective_attempts,
            std::vector<std::vector<Eigen::VectorXd>>& attempt_paths) {
            if (batch_context.executor().n_threads() > 1 && effective_attempts > 1) {
                std::shared_ptr<std::atomic<bool>> local_cancel =
                    make_parallel_rrt_cancel_flag();
                std::atomic<int> early_successes{0};
                batch_context.executor().parallel_for(0, effective_attempts, [&](int attempt) {
                    if (local_cancel && local_cancel->load(std::memory_order_relaxed)) {
                        return;
                    }
                    auto path = run_task_attempt(task, attempt, 0, local_cancel);
                    maybe_stop_parallel_rrt_after_success(task,
                                                          path,
                                                          early_successes,
                                                          local_cancel);
                    attempt_paths[static_cast<std::size_t>(attempt)] = std::move(path);
                });
                record_parallel_rrt_early_stop(local_cancel, early_successes);
                return;
            }
            for (int attempt = 0; attempt < effective_attempts; ++attempt) {
                attempt_paths[static_cast<std::size_t>(attempt)] =
                    run_task_attempt(task, attempt, 0);
            }
        };

    batch_context.diagnostics().set_value("query_bridge.attempt_offset",
                                          static_cast<double>(retry_options.attempt_offset));
    const bool has_segment_only_task =
        std::any_of(tasks.begin(), tasks.end(), [&](const QueryBridgeSearchTask& task) {
            return query_bridge_index_segment_only(index_options, task.index);
        });
    if (batch_execution_options.parallel_task_rrt && !has_segment_only_task &&
        retry_options.no_path_retry_attempts == 0 &&
        retry_options.no_path_retry_budget_stages == 0) {
        struct PreparedTask {
            bool skipped = false;
            bool forced = false;
            int attempts = 1;
            double task_start_ms = 0.0;
        };
        std::vector<PreparedTask> prepared(tasks.size());
        std::vector<QueryBridgeSearchJob> jobs;
        for (std::size_t task_offset = 0; task_offset < tasks.size(); ++task_offset) {
            auto& task = tasks[task_offset];
            prepared[task_offset].task_start_ms = elapsed_ms_since(batch_t0);
            const auto probe_t0 = Clock::now();
            if (task_already_satisfied(task)) {
                prepared[task_offset].skipped = true;
                mark_batch_task_already_satisfied(task, probe_t0);
                continue;
            }
            batch_context.diagnostics().record_timing("query_bridge.batch_probe_ms_total",
                                                      elapsed_ms_since(probe_t0));
            batch_context.diagnostics().add_counter("query_bridge.batch_tasks_attempted");
            const QueryBridgeAttemptPlan attempt_plan = prepare_task_attempts(task);
            prepared[task_offset].forced = attempt_plan.forced;
            prepared[task_offset].attempts = attempt_plan.effective_attempts;
            for (int attempt = 0; attempt < prepared[task_offset].attempts; ++attempt) {
                jobs.push_back({task_offset, attempt});
            }
        }

        std::vector<std::vector<std::vector<Eigen::VectorXd>>> attempt_paths(tasks.size());
        for (std::size_t task_offset = 0; task_offset < tasks.size(); ++task_offset) {
            attempt_paths[task_offset].resize(
                static_cast<std::size_t>(std::max(0, prepared[task_offset].attempts)));
        }
        const auto rrt_t0 = Clock::now();
        if (batch_context.executor().n_threads() > 1 && jobs.size() > 1) {
            std::shared_ptr<std::atomic<bool>> local_cancel =
                make_parallel_rrt_cancel_flag();
            std::atomic<int> early_successes{0};
            batch_context.executor().parallel_for(0,
                                                  static_cast<int>(jobs.size()),
                                                  [&](int job_index) {
                if (local_cancel && local_cancel->load(std::memory_order_relaxed)) {
                    return;
                }
                const QueryBridgeSearchJob& job = jobs[static_cast<std::size_t>(job_index)];
                auto path = run_task_attempt(tasks[job.task_index],
                                             job.attempt,
                                             0,
                                             local_cancel);
                maybe_stop_parallel_rrt_after_success(tasks[job.task_index],
                                                      path,
                                                      early_successes,
                                                      local_cancel);
                attempt_paths[job.task_index][static_cast<std::size_t>(job.attempt)] =
                    std::move(path);
            });
            record_parallel_rrt_early_stop(local_cancel, early_successes);
        } else {
            for (const QueryBridgeSearchJob& job : jobs) {
                attempt_paths[job.task_index][static_cast<std::size_t>(job.attempt)] =
                    run_task_attempt(tasks[job.task_index], job.attempt, 0);
            }
        }
        const double rrt_ms = elapsed_ms_since(rrt_t0);
        batch_context.diagnostics().record_timing("query_bridge.batch_rrt_ms_total",
                                                  rrt_ms);
        batch_context.diagnostics().set_value("query_bridge.parallel_task_rrt",
                                              1.0);
        batch_context.diagnostics().set_value("query_bridge.parallel_task_rrt_jobs",
                                              static_cast<double>(jobs.size()));

        for (std::size_t task_offset = 0; task_offset < tasks.size(); ++task_offset) {
            auto& task = tasks[task_offset];
            if (prepared[task_offset].skipped) {
                batch_context.diagnostics().set_value(
                    query_bridge_task_key(task.index, "total_ms"),
                    elapsed_ms_since(batch_t0) - prepared[task_offset].task_start_ms);
                continue;
            }
            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "rrt_ms"),
                                                  rrt_ms);
            double best_length = std::numeric_limits<double>::infinity();
            if (task.waypoint_path_from_partition_query && !task.waypoint_path.empty()) {
                best_length = path_length(task.waypoint_path);
                mark_partition_path_first_rrt_skipped(task);
            }
            adopt_waypoint_after_rrt(task,
                                     attempt_paths[task_offset],
                                     prepared[task_offset].attempts,
                                     best_length);
            if (task.waypoint_path.empty()) {
                mark_batch_task_no_path(
                    task,
                    elapsed_ms_since(batch_t0) - prepared[task_offset].task_start_ms);
                continue;
            }
            finish_ready_waypoint_task(
                task,
                prepared[task_offset].forced,
                false,
                best_length,
                [&]() {
                    return elapsed_ms_since(batch_t0) -
                           prepared[task_offset].task_start_ms;
                });
        }

        batch_context.diagnostics().set_value("query_bridge.batch_total_ms",
                                              elapsed_ms_since(batch_t0));
        return finish_batch_bridge();
    }

    for (auto& task : tasks) {
        const auto task_t0 = Clock::now();
        const auto probe_t0 = Clock::now();
        if (task_already_satisfied(task)) {
            mark_batch_task_already_satisfied(task, probe_t0);
            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "total_ms"),
                                                  elapsed_ms_since(task_t0));
            continue;
        }
        batch_context.diagnostics().record_timing("query_bridge.batch_probe_ms_total",
                                                  elapsed_ms_since(probe_t0));
        batch_context.diagnostics().add_counter("query_bridge.batch_tasks_attempted");
        const QueryBridgeAttemptPlan attempt_plan = prepare_task_attempts(task);
        if (attempt_plan.partition_path_first) {
            mark_partition_path_first_rrt_skipped(task);
        }
        std::vector<std::vector<Eigen::VectorXd>> attempt_paths(
            static_cast<std::size_t>(attempt_plan.effective_attempts));
        const auto rrt_t0 = Clock::now();
        run_attempts_for_task(task, attempt_plan.effective_attempts, attempt_paths);
        const double rrt_ms = elapsed_ms_since(rrt_t0);
        batch_context.diagnostics().record_timing("query_bridge.batch_rrt_ms_total",
                                                  rrt_ms);
        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "rrt_ms"),
                                              rrt_ms);
        double best_length = std::numeric_limits<double>::infinity();
        if (task.waypoint_path_from_partition_query && !task.waypoint_path.empty()) {
            best_length = path_length(task.waypoint_path);
        }
        adopt_waypoint_after_rrt(task,
                                 attempt_paths,
                                 attempt_plan.base_attempts,
                                 best_length);
        const bool segment_only_task =
            query_bridge_index_segment_only(index_options, task.index);
        if (segment_only_task) {
            run_segment_only_retry(task, attempt_plan.base_attempts, best_length);
        } else {
            run_no_path_retries(task, attempt_plan.base_attempts, best_length);
        }
        if (task.waypoint_path.empty()) {
            mark_batch_task_no_path(task, elapsed_ms_since(task_t0));
            continue;
        }
        finish_ready_waypoint_task(task,
                                   attempt_plan.forced,
                                   segment_only_task,
                                   best_length,
                                   [&]() { return elapsed_ms_since(task_t0); });
    }

    batch_context.diagnostics().set_value("query_bridge.batch_total_ms",
                                          elapsed_ms_since(batch_t0));
    return finish_batch_bridge();
}

} // namespace rbf
