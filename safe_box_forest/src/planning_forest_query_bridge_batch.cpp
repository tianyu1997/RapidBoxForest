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
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
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
    auto locate_existing_box_for_query_bridge = [&](const Eigen::Ref<const Eigen::VectorXd>& point) {
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
    };
    auto box_id_contains_query_point = [&](int box_id,
                                           const Eigen::Ref<const Eigen::VectorXd>& point) {
        if (box_id < 0) {
            return false;
        }
        const BoxNode* box = find_box_by_id(boxes_, box_id);
        return box != nullptr &&
               intervals_contain_point_local(box->joint_intervals,
                                             point,
                                             config_.query.adjacency_tolerance);
    };
    auto refresh_located_or_keep_anchor = [&](int anchor_box_id,
                                              const Eigen::Ref<const Eigen::VectorXd>& point,
                                              const char* endpoint_name) {
        const int located = locate_existing_box_for_query_bridge(point);
        if (located >= 0) {
            return located;
        }
        if (box_id_contains_query_point(anchor_box_id, point)) {
            last_build_.diagnostics[std::string("query_bridge.endpoint_anchor_keep_after_lookup_miss.") +
                                    endpoint_name] += 1.0;
            return anchor_box_id;
        }
        return -1;
    };
    auto catch_up_query_bridge_partition = [&](const char* diagnostic_prefix) {
        if (partition_native_mode() &&
            adaptive_partition_query_enabled_ &&
            adaptive_partition_ &&
            boxes_.size() > partition_refresh_base) {
            append_adaptive_partition_boxes(partition_refresh_base,
                                            &last_build_,
                                            diagnostic_prefix);
            partition_refresh_base = boxes_.size();
        }
    };
    const QueryBridgeIndexOptions index_options = query_bridge_index_options_from_env();
    auto query_bridge_forced_index = [&](std::size_t index) {
        return query_bridge_index_forced(index_options, index);
    };
    const QueryBridgePartitionPathFirstOptions partition_path_first_options =
        query_bridge_partition_path_first_options_from_env(partition_native_mode());

    std::vector<QueryBridgeSearchTask> tasks;
    tasks.reserve(starts.size());
    for (std::size_t index = 0; index < starts.size(); ++index) {
        if (starts[index].size() != goals[index].size()) {
            throw std::invalid_argument("bridge_queries received a start/goal dimension mismatch");
        }
        const bool forced_task = query_bridge_forced_index(index);
        auto mark_task_skip = [&](double code, const char* reason) {
            last_build_.diagnostics["query_bridge.batch_task." +
                                    std::to_string(index) +
                                    ".skip_reason_code"] = code;
            if (reason != nullptr && reason[0] != '\0') {
                last_build_.diagnostics[std::string("query_bridge.batch_task_skip.") + reason] += 1.0;
            }
        };
        QueryResult initial_query;
        bool has_initial_query = false;
        if (!forced_task || partition_path_first_options.enabled) {
            initial_query = query(starts[index], goals[index]);
            has_initial_query = true;
            if (!forced_task && query_result_good(initial_query, starts[index], goals[index])) {
                mark_task_skip(1.0, "initial_good");
                continue;
            }
        }
        int start_box_id = locate_existing_box_for_query_bridge(starts[index]);
        if (start_box_id < 0) {
            StageContext anchor_context = StageContext::from_runtime(config_.runtime);
            start_box_id = anchor_query_endpoint_box(starts[index], anchor_context);
            merge_diagnostic_snapshot(last_build_.diagnostics, anchor_context.diagnostics().snapshot());
        }
        if (start_box_id < 0) {
            mark_task_skip(2.0, "start_anchor_failed");
            continue;
        }
        int goal_box_id = locate_existing_box_for_query_bridge(goals[index]);
        if (goal_box_id < 0) {
            StageContext anchor_context = StageContext::from_runtime(config_.runtime);
            goal_box_id = anchor_query_endpoint_box(goals[index], anchor_context);
            merge_diagnostic_snapshot(last_build_.diagnostics, anchor_context.diagnostics().snapshot());
        }
        catch_up_query_bridge_partition("query_bridge.endpoint_anchor");
        if (start_box_id >= 0) {
            start_box_id = refresh_located_or_keep_anchor(start_box_id,
                                                          starts[index],
                                                          "start");
        }
        if (goal_box_id >= 0) {
            goal_box_id = refresh_located_or_keep_anchor(goal_box_id,
                                                         goals[index],
                                                         "goal");
        }
        if (goal_box_id < 0 || goal_box_id == start_box_id) {
            mark_task_skip(goal_box_id < 0 ? 3.0 : 4.0,
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
        task.short_local_bridge = bridge_distance > 0.55 && bridge_distance < 0.85;
        if (task.short_local_bridge) {
            task.bridge_rrt.step_size = std::min(task.bridge_rrt.step_size, 0.25);
            task.bridge_rrt.goal_bias = 0.08;
            task.bridge_rrt.local_sampling_radius =
                task.bridge_rrt.local_sampling_radius > 0.0
                    ? std::min(task.bridge_rrt.local_sampling_radius, 0.85)
                    : 0.85;
            auto add_profile = [&](double step_size, double goal_bias, double radius) {
                RRTConnectConfig profile = task.bridge_rrt;
                profile.step_size = step_size;
                profile.goal_bias = goal_bias;
                profile.local_sampling_radius = radius;
                profile.shortcut_path = true;
                task.short_local_profiles.push_back(std::move(profile));
            };
            add_profile(0.25, 0.08, 0.90);
            add_profile(0.50, 0.20, 1.00);
            add_profile(0.35, 0.10, 1.00);
            add_profile(0.25, 0.08, 0.45);
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
    struct BatchBridgeDiagnosticsFlush {
        BuildProfile& profile;
        StageContext& context;
        ~BatchBridgeDiagnosticsFlush() {
            for (const auto& [key, value] : context.diagnostics().snapshot()) {
                profile.diagnostics[key] = value;
            }
        }
    } batch_diagnostics_flush{last_build_, batch_context};
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
            locate_existing_box_for_query_bridge(task.start);
        const int target_box_id =
            locate_existing_box_for_query_bridge(task.goal);
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
        const int source_box_id = locate_existing_box_for_query_bridge(task.start);
        const int target_box_id = locate_existing_box_for_query_bridge(task.goal);
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
	        if (!last_adaptive_partition_config_.hipac_online_connectivity ||
	            !last_adaptive_partition_config_.hipac_online_before_query_bridge ||
	            !last_adaptive_partition_config_.hipac_online_prebridge_portal ||
	            !partition_native_mode() ||
	            !adaptive_partition_query_enabled_ ||
	            !adaptive_partition_ ||
		            adaptive_partition_->empty() ||
		            task.hipac_prebridge_resolves_used >=
		                std::max(0, last_adaptive_partition_config_.hipac_online_max_resolves_per_query)) {
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
	        const int candidate_limit =
	            std::max(1, last_adaptive_partition_config_.hipac_online_prebridge_candidate_limit);
	        const auto candidate_pairs =
	            adaptive_partition_->nearest_component_pairs_to_largest(1, candidate_limit);
	        batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_candidates",
	                                                static_cast<double>(candidate_pairs.size()));
	        if (candidate_pairs.empty()) {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_no_candidates");
	            return 0;
	        }

	        std::unordered_map<int, int> component_by_box;
	        const auto components = adaptive_partition_->component_box_ids_with_overlay();
	        for (std::size_t component_index = 0; component_index < components.size(); ++component_index) {
	            for (int box_id : components[component_index]) {
	                component_by_box.emplace(box_id, static_cast<int>(component_index));
	            }
	        }
	        const int start_box_id = locate_box_partition_first(task.start, false);
	        const int goal_box_id = locate_box_partition_first(task.goal, false);
	        const int start_component =
	            component_by_box.count(start_box_id) > 0 ? component_by_box[start_box_id] : -1;
	        const int goal_component =
	            component_by_box.count(goal_box_id) > 0 ? component_by_box[goal_box_id] : -1;
	        const bool has_endpoint_component_target =
	            start_component > 0 || goal_component > 0;

	        const double max_pair_distance =
	            std::max(0.0, last_adaptive_partition_config_.hipac_online_prebridge_max_pair_distance);
	        const double route_weight =
	            std::max(0.0,
	                     last_adaptive_partition_config_.hipac_online_prebridge_route_distance_weight);
	        const double pair_weight =
	            std::max(0.0,
	                     last_adaptive_partition_config_.hipac_online_prebridge_pair_distance_weight);
	        const AdaptiveGridPartitionComponentPair* best_pair = nullptr;
	        double best_score = std::numeric_limits<double>::infinity();
	        int considered = 0;
	        int distance_rejects = 0;
	        int endpoint_component_rejects = 0;
	        for (const auto& pair : candidate_pairs) {
	            if (pair.source_box_id < 0 ||
	                pair.target_box_id < 0 ||
	                pair.source_point.size() == 0 ||
	                pair.target_point.size() == 0 ||
	                pair.source_point.size() != pair.target_point.size()) {
	                continue;
	            }
	            const bool matches_endpoint_component =
	                (start_component > 0 && pair.source_component_index == start_component) ||
	                (goal_component > 0 && pair.source_component_index == goal_component);
	            if (has_endpoint_component_target && !matches_endpoint_component) {
	                ++endpoint_component_rejects;
	                continue;
	            }
	            const double pair_distance = std::sqrt(std::max(0.0, pair.distance_sq));
	            if (max_pair_distance > 0.0 &&
	                pair_distance > max_pair_distance + 1e-12) {
	                ++distance_rejects;
	                continue;
	            }
	            const Eigen::VectorXd midpoint = 0.5 * (pair.source_point + pair.target_point);
		            const double route_distance =
		                std::sqrt(std::max(0.0,
		                                   query_bridge_point_polyline_distance_sq(midpoint,
		                                                              coarse_route)));
	            const bool touches_start =
	                start_component >= 0 &&
	                (pair.source_component_index == start_component ||
	                 pair.target_component_index == start_component);
	            const bool touches_goal =
	                goal_component >= 0 &&
	                (pair.source_component_index == goal_component ||
	                 pair.target_component_index == goal_component);
	            const double endpoint_bonus = (touches_start ? 0.50 : 0.0) +
	                                          (touches_goal ? 0.50 : 0.0);
	            const double component_size_bonus =
	                0.02 * std::log1p(static_cast<double>(
	                    std::max(0, pair.source_component_size)));
	            const double score = route_weight * route_distance +
	                                 pair_weight * pair_distance -
	                                 endpoint_bonus -
	                                 component_size_bonus;
	            ++considered;
	            if (score < best_score) {
	                best_score = score;
	                best_pair = &pair;
	            }
	        }
	        batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_considered",
	                                                static_cast<double>(considered));
	        batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_distance_rejects",
	                                                static_cast<double>(distance_rejects));
	        batch_context.diagnostics().add_counter(
	            "query_bridge.hipac_prebridge_endpoint_component_rejects",
	            static_cast<double>(endpoint_component_rejects));
	        if (best_pair == nullptr) {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_no_candidate_after_filter");
	            return 0;
	        }

	        task.hipac_prebridge_resolves_used += 1;
	        batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_portal_attempts");
	        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_prebridge_score"),
	                                              best_score);
	        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_prebridge_pair_distance"),
	                                              std::sqrt(std::max(0.0, best_pair->distance_sq)));
	        std::vector<Eigen::VectorXd> local_path{best_pair->source_point, best_pair->target_point};
	        const int added = add_partition_portal_corridor_overlay(best_pair->source_point,
	                                                                best_pair->target_point,
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
	        if (!last_adaptive_partition_config_.hipac_online_connectivity ||
	            !last_adaptive_partition_config_.hipac_online_before_query_bridge ||
	            !partition_native_mode() ||
	            task.hipac_candidate_path.size() < 2 ||
	            task.hipac_online_resolves_used >=
	                std::max(0, last_adaptive_partition_config_.hipac_online_max_resolves_per_query)) {
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
	        const double hipac_candidate_max_length =
	            std::max(0.0, last_adaptive_partition_config_.hipac_online_candidate_max_length);
	        batch_context.diagnostics().add_counter("query_bridge.hipac_online_candidate_length",
	                                                hipac_candidate_length);
	        if (hipac_candidate_max_length > 0.0 &&
	            hipac_candidate_length > hipac_candidate_max_length + 1e-12) {
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
	        if (!last_adaptive_partition_config_.hipac_online_connectivity ||
	            !last_adaptive_partition_config_.hipac_online_before_query_bridge ||
	            !last_adaptive_partition_config_.hipac_online_transition_portal ||
	            !partition_native_mode() ||
	            !adaptive_partition_query_enabled_ ||
	            !adaptive_partition_ ||
	            adaptive_partition_->empty() ||
	            task.waypoint_path.size() < 2 ||
	            task.hipac_transition_resolves_used >=
	                std::max(0, last_adaptive_partition_config_.hipac_transition_max_attempts_per_query)) {
	            return 0;
	        }
	        const bool target_index =
	            csv_index_list_contains(last_adaptive_partition_config_.hipac_transition_target_query_indices,
	                                    static_cast<int>(task.index)) ||
	            csv_index_list_contains(last_adaptive_partition_config_.hipac_transition_target_query_indices,
	                                    task.query_index);
	        if (!target_index) {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_transition_target_rejects");
	            return 0;
	        }

	        const auto transition_t0 = Clock::now();
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_attempts");
	        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_transition_attempt"),
	                                              1.0);
	        struct TransitionCandidate {
	            int source_box_id = -1;
	            int target_box_id = -1;
	            int source_component = -1;
	            int target_component = -1;
	            int first_waypoint = 0;
	            int last_waypoint = 0;
	            Eigen::VectorXd source_point;
	            Eigen::VectorXd target_point;
	            std::vector<Eigen::VectorXd> local_path;
	            double pair_distance = 0.0;
	            double local_length = 0.0;
	            int predicted_bridge_edges = 0;
	            double score = -std::numeric_limits<double>::infinity();
	        };
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
	        std::unordered_map<int, int> component_by_box;
	        const auto components = adaptive_partition_->component_box_ids_with_overlay();
	        for (std::size_t component_index = 0; component_index < components.size(); ++component_index) {
	            for (int box_id : components[component_index]) {
	                component_by_box.emplace(box_id, static_cast<int>(component_index));
	            }
	        }

	        std::vector<TransitionCandidate> candidates;
	        candidates.reserve(static_cast<std::size_t>(candidate_limit));
	        int gated = 0;
	        int same_component_gated = 0;
	        int distance_gated = 0;
	        int edge_gated = 0;
	        for (std::size_t begin = 0; begin + 1 < task.waypoint_path.size(); ++begin) {
	            const std::size_t end =
	                std::min(task.waypoint_path.size() - 1,
	                         begin + static_cast<std::size_t>(stride));
	            if (end <= begin) {
	                continue;
	            }
	            const auto source_nearest =
	                adaptive_partition_->nearest_boxes(task.waypoint_path[begin], {}, 1);
	            const auto target_nearest =
	                adaptive_partition_->nearest_boxes(task.waypoint_path[end], {}, 1);
	            if (source_nearest.empty() || target_nearest.empty()) {
	                ++gated;
	                continue;
	            }
	            const auto& source = source_nearest.front();
	            const auto& target = target_nearest.front();
	            if (source.box_id < 0 || target.box_id < 0 ||
	                source.box_id == target.box_id ||
	                source.closest_point.size() == 0 ||
	                target.closest_point.size() == 0 ||
	                source.closest_point.size() != target.closest_point.size()) {
	                ++gated;
	                continue;
	            }
	            const int source_component =
	                component_by_box.count(source.box_id) > 0 ? component_by_box[source.box_id] : -1;
	            const int target_component =
	                component_by_box.count(target.box_id) > 0 ? component_by_box[target.box_id] : -1;
	            if (!last_adaptive_partition_config_.hipac_transition_allow_same_component &&
	                source_component >= 0 &&
	                source_component == target_component) {
	                ++same_component_gated;
	                continue;
	            }
	            const double pair_distance =
	                (target.closest_point - source.closest_point).norm();
	            if (max_pair_distance > 0.0 &&
	                pair_distance > max_pair_distance + 1e-12) {
	                ++distance_gated;
	                continue;
	            }
	            double local_length = 0.0;
	            for (std::size_t index = begin + 1; index <= end; ++index) {
	                local_length += (task.waypoint_path[index] - task.waypoint_path[index - 1]).norm();
	            }
	            const int predicted_edges =
	                static_cast<int>(std::ceil(local_length / sample_step));
	            if (predicted_edges < min_predicted_edges) {
	                ++edge_gated;
	                continue;
	            }
	            TransitionCandidate candidate;
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
	                candidate.local_path.push_back(task.waypoint_path[index]);
	            }
	            candidate.local_path.push_back(candidate.target_point);
	            candidate.score =
	                static_cast<double>(predicted_edges) -
	                0.25 * pair_distance -
	                0.05 * static_cast<double>(std::abs(target_component - source_component));
	            candidates.push_back(std::move(candidate));
	        }
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_candidates",
	                                                static_cast<double>(candidates.size()));
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_gated",
	                                                static_cast<double>(gated + same_component_gated +
	                                                                    distance_gated + edge_gated));
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_gated_same_component",
	                                                static_cast<double>(same_component_gated));
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_gated_distance",
	                                                static_cast<double>(distance_gated));
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_gated_edges",
	                                                static_cast<double>(edge_gated));
	        if (candidates.empty()) {
	            return 0;
	        }
	        std::sort(candidates.begin(),
	                  candidates.end(),
	                  [](const TransitionCandidate& lhs, const TransitionCandidate& rhs) {
	            if (std::abs(lhs.score - rhs.score) > 1e-12) {
	                return lhs.score > rhs.score;
	            }
	            if (std::abs(lhs.local_length - rhs.local_length) > 1e-12) {
	                return lhs.local_length > rhs.local_length;
	            }
	            return lhs.first_waypoint < rhs.first_waypoint;
	        });
	        if (static_cast<int>(candidates.size()) > candidate_limit) {
	            candidates.resize(static_cast<std::size_t>(candidate_limit));
	        }

	        int total_added = 0;
	        int attempts = 0;
	        const int attempt_cap =
	            std::max(1, last_adaptive_partition_config_.hipac_transition_max_attempts_per_query);
	        for (const auto& candidate : candidates) {
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
    batch_context.diagnostics().set_value(
        "query_bridge.accept_segment_fraction",
        bridge_acceptance.max_segment_fraction);
    batch_context.diagnostics().set_value(
        "query_bridge.accept_path_ratio",
        bridge_acceptance.path_ratio);
    batch_context.diagnostics().set_value(
        "query_bridge.accept_path_additive",
        bridge_acceptance.path_additive);
    batch_context.diagnostics().set_value(
        "query_bridge.accept_max_path_length",
        bridge_acceptance.max_path_length);
    batch_context.diagnostics().set_value(
        "query_bridge.partition_path_first",
        partition_path_first_options.enabled ? 1.0 : 0.0);
    batch_context.diagnostics().set_value(
        "query_bridge.partition_path_first_allow_long",
        partition_path_first_options.allow_long ? 1.0 : 0.0);
    batch_context.diagnostics().set_value(
        "query_bridge.partition_path_first_max_segment_fraction",
        partition_path_first_options.max_segment_fraction);
    auto rrt_path_good_enough_for_task = [&](const QueryBridgeSearchTask& task,
                                             const std::vector<Eigen::VectorXd>& path) {
        return query_bridge_parallel_rrt_path_good_enough(task.start,
                                                          task.goal,
                                                          path,
                                                          parallel_rrt_options);
    };
    auto query_bridge_forced = [&](const QueryBridgeSearchTask& task) {
        return query_bridge_forced_index(task.index);
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
        if (!direct_line_options.enabled) {
            return std::vector<Eigen::VectorXd>{};
        }
        batch_context.diagnostics().add_counter(
            "query_bridge.direct_line_on_no_path_attempts");
        CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
        std::vector<Eigen::VectorXd> path{task.start, task.goal};
        const PathAuditCheck audit =
            audit_waypoint_path(path,
                                checker,
                                config_.query.audit_resolution,
                                config_.query.audit_segment_step);
        if (!audit.passed) {
            batch_context.diagnostics().add_counter(
                "query_bridge.direct_line_on_no_path_rejects");
            return std::vector<Eigen::VectorXd>{};
        }
        batch_context.diagnostics().add_counter(
            "query_bridge.direct_line_on_no_path_successes");
        return path;
    };
    const QueryBridgeDetourOptions detour_options = query_bridge_detour_options_from_env();
    record_query_bridge_detour_diagnostics(batch_context, detour_options);
    auto deterministic_detour_fallback_path = [&](const QueryBridgeSearchTask& task) {
        if (!detour_options.enabled ||
            task.start.size() != task.goal.size() ||
            task.start.size() <= 0) {
            return std::vector<Eigen::VectorXd>{};
        }
        const auto domain = oracle_->planning_intervals();
        if (static_cast<int>(domain.size()) != task.start.size()) {
            return std::vector<Eigen::VectorXd>{};
        }
        CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
        const Eigen::VectorXd delta = task.goal - task.start;
        const double direct_length = delta.norm();
        if (direct_length <= 1e-9) {
            return std::vector<Eigen::VectorXd>{};
        }
        std::vector<int> dims(static_cast<std::size_t>(task.start.size()));
        std::iota(dims.begin(), dims.end(), 0);
        std::sort(dims.begin(), dims.end(), [&](int lhs, int rhs) {
            const double lhs_width = std::max(1e-9, domain[static_cast<std::size_t>(lhs)].width());
            const double rhs_width = std::max(1e-9, domain[static_cast<std::size_t>(rhs)].width());
            const double lhs_along = std::abs(delta[lhs]) / lhs_width;
            const double rhs_along = std::abs(delta[rhs]) / rhs_width;
            if (std::abs(lhs_along - rhs_along) > 1e-12) {
                return lhs_along < rhs_along;
            }
            return lhs < rhs;
        });
        const int dim_limit = std::min<int>(detour_options.dims,
                                            static_cast<int>(dims.size()));
        const int rounds = detour_options.rounds;
        const int max_candidates = detour_options.max_candidates;
        const bool multi_axis_detour = detour_options.multi_axis;
        const int random_candidates = detour_options.random_candidates;
        const double base_offset = detour_options.offset;
        const double two_bend_alpha = detour_options.two_bend_alpha;
        const Eigen::VectorXd mid = 0.5 * (task.start + task.goal);
        double best_length = std::numeric_limits<double>::infinity();
        std::vector<Eigen::VectorXd> best_path;
        int candidates = 0;
        auto clamp_to_domain = [&](Eigen::VectorXd point) {
            for (int dim = 0; dim < point.size(); ++dim) {
                point[dim] = std::min(domain[static_cast<std::size_t>(dim)].hi,
                                      std::max(domain[static_cast<std::size_t>(dim)].lo,
                                               point[dim]));
            }
            return point;
        };
        auto try_path = [&](std::vector<Eigen::VectorXd> path) {
            if (candidates >= max_candidates) {
                return;
            }
            ++candidates;
            batch_context.diagnostics().add_counter(
                "query_bridge.detour_on_no_path_candidates");
            double length = path_length(path);
            if (!std::isfinite(length) || length + 1e-12 >= best_length) {
                return;
            }
            const PathAuditCheck audit =
                audit_waypoint_path(path,
                                    checker,
                                    config_.query.audit_resolution,
                                    config_.query.audit_segment_step);
            if (!audit.passed) {
                batch_context.diagnostics().add_counter(
                    "query_bridge.detour_on_no_path_rejects");
                return;
            }
            best_length = length;
            best_path = std::move(path);
        };
        for (int item = 0; item < dim_limit && candidates < max_candidates; ++item) {
            const int dim = dims[static_cast<std::size_t>(item)];
            const double width = domain[static_cast<std::size_t>(dim)].width();
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
                const double first_width = domain[static_cast<std::size_t>(first_dim)].width();
                for (int second_item = first_item + 1;
                     second_item < dim_limit && candidates < max_candidates;
                     ++second_item) {
                    const int second_dim = dims[static_cast<std::size_t>(second_item)];
                    const double second_width = domain[static_cast<std::size_t>(second_dim)].width();
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
                derived_planner_seed(config_.grower.rng_seed,
                                     kSeedBatchBridgeOffset,
                                     task.index,
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
                    const double width = domain[static_cast<std::size_t>(dim)].width();
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
        batch_context.diagnostics().add_counter(
            "query_bridge.detour_on_no_path_attempts");
        if (!best_path.empty()) {
            batch_context.diagnostics().add_counter(
                "query_bridge.detour_on_no_path_successes");
        }
        return best_path;
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
	            try_hipac_online_bridge(task);
	            if (!task.hipac_online_satisfied) {
	                try_hipac_transition_portal(task);
	            }
	            if (!task.hipac_online_satisfied) {
	                try_hipac_prebridge_portal(task);
	            }
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
        struct PreparedJob {
            std::size_t task_offset = 0;
            int attempt = 0;
        };
        std::vector<PreparedTask> prepared(tasks.size());
        std::vector<PreparedJob> jobs;
        for (std::size_t task_offset = 0; task_offset < tasks.size(); ++task_offset) {
            auto& task = tasks[task_offset];
            prepared[task_offset].task_start_ms = elapsed_ms_since(batch_t0);
            const auto probe_t0 = Clock::now();
            if (task.hipac_online_satisfied ||
                task.direct_start_goal_satisfied ||
                current_query_good(task, true)) {
                prepared[task_offset].skipped = true;
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
                continue;
            }
            batch_context.diagnostics().record_timing("query_bridge.batch_probe_ms_total",
                                                      elapsed_ms_since(probe_t0));
            batch_context.diagnostics().add_counter("query_bridge.batch_tasks_attempted");
            prepared[task_offset].forced = query_bridge_forced(task);
            prepared[task_offset].attempts = prepared[task_offset].forced
                ? std::max(std::max(1, task.attempts), retry_options.forced_attempts)
                : std::max(1, task.attempts);
            if (task.waypoint_path_from_partition_query && !task.waypoint_path.empty()) {
                prepared[task_offset].attempts = 0;
                batch_context.diagnostics().add_counter(
                    "query_bridge.partition_path_first_tasks");
                batch_context.diagnostics().set_value(
                    query_bridge_task_key(task.index, "partition_path_first"),
                    1.0);
            }
            if (prepared[task_offset].attempts > 0 &&
                !retry_options.local_radius_schedule.empty() &&
                retry_options.local_radius_append_unrestricted_attempt) {
                prepared[task_offset].attempts = std::max(
                    prepared[task_offset].attempts,
                    static_cast<int>(retry_options.local_radius_schedule.size()) + 1);
            }
            if (prepared[task_offset].forced) {
                batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "forced"),
                                                      1.0);
            }
            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "attempts"),
                                                  static_cast<double>(prepared[task_offset].attempts));
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
                parallel_rrt_options.early_stop ? std::make_shared<std::atomic<bool>>(false)
                                                : batch_context.native_cancel_flag();
            std::atomic<int> early_successes{0};
            batch_context.executor().parallel_for(0,
                                                  static_cast<int>(jobs.size()),
                                                  [&](int job_index) {
                if (local_cancel && local_cancel->load(std::memory_order_relaxed)) {
                    return;
                }
                const PreparedJob& job = jobs[static_cast<std::size_t>(job_index)];
                auto path = run_task_attempt(tasks[job.task_offset],
                                             job.attempt,
                                             0,
                                             local_cancel);
                if (parallel_rrt_options.early_stop &&
                    rrt_path_good_enough_for_task(tasks[job.task_offset], path)) {
                    const int successes =
                        early_successes.fetch_add(1, std::memory_order_relaxed) + 1;
                    if (successes >= parallel_rrt_options.early_stop_min_successes &&
                        local_cancel) {
                        local_cancel->store(true, std::memory_order_relaxed);
                    }
                }
                attempt_paths[job.task_offset][static_cast<std::size_t>(job.attempt)] =
                    std::move(path);
            });
            if (parallel_rrt_options.early_stop) {
                batch_context.diagnostics().add_counter(
                    "query_bridge.parallel_rrt_early_stop_successes",
                    static_cast<double>(early_successes.load(std::memory_order_relaxed)));
                batch_context.diagnostics().add_counter(
                    local_cancel && local_cancel->load(std::memory_order_relaxed)
                        ? "query_bridge.parallel_rrt_early_stop_triggered"
                        : "query_bridge.parallel_rrt_early_stop_not_triggered");
            }
        } else {
            for (const PreparedJob& job : jobs) {
                attempt_paths[job.task_offset][static_cast<std::size_t>(job.attempt)] =
                    run_task_attempt(tasks[job.task_offset], job.attempt, 0);
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
                batch_context.diagnostics().add_counter(
                    "query_bridge.partition_path_first_rrt_skipped");
                batch_context.diagnostics().set_value(
                    query_bridge_task_key(task.index, "waypoint_from_partition_path"),
                    1.0);
            }
            select_attempt_paths(task, attempt_paths[task_offset], best_length);
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
                                       prepared[task_offset].attempts,
                                       best_length,
                                       task.waypoint_path);
            if (task.waypoint_path.empty()) {
                batch_context.diagnostics().add_counter("query_bridge.batch_tasks_no_path");
                batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "no_path"),
                                                      1.0);
                batch_context.diagnostics().set_value(
                    query_bridge_task_key(task.index, "total_ms"),
                    elapsed_ms_since(batch_t0) - prepared[task_offset].task_start_ms);
                continue;
            }
            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "waypoint_length"),
                                                  best_length);
            const auto second_probe_t0 = Clock::now();
            if (current_query_good(task, !retry_options.post_rrt_skip_forced)) {
                batch_context.diagnostics().add_counter("query_bridge.batch_tasks_skipped_after_rrt");
                if (prepared[task_offset].forced) {
                    batch_context.diagnostics().add_counter(
                        "query_bridge.batch_forced_tasks_skipped_after_rrt");
                }
                batch_context.diagnostics().record_timing("query_bridge.batch_probe_ms_total",
                                                          elapsed_ms_since(second_probe_t0));
                batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "skipped_after_rrt"),
                                                      1.0);
                batch_context.diagnostics().set_value(
                    query_bridge_task_key(task.index, "total_ms"),
                    elapsed_ms_since(batch_t0) - prepared[task_offset].task_start_ms);
                continue;
            }
            batch_context.diagnostics().record_timing("query_bridge.batch_probe_ms_total",
                                                      elapsed_ms_since(second_probe_t0));
            const int hipac_resolve_cap =
                std::max(0, last_adaptive_partition_config_.hipac_online_max_resolves_per_query);
            const bool can_run_hipac_online =
                task.hipac_online_resolves_used < hipac_resolve_cap ||
                (last_adaptive_partition_config_.hipac_online_transition_portal &&
                 task.hipac_transition_resolves_used <
                     std::max(0, last_adaptive_partition_config_.hipac_transition_max_attempts_per_query)) ||
                (last_adaptive_partition_config_.hipac_online_prebridge_portal &&
                 task.hipac_prebridge_resolves_used < hipac_resolve_cap);
            if (last_adaptive_partition_config_.hipac_online_connectivity &&
                !task.waypoint_path.empty() &&
                can_run_hipac_online) {
                task.hipac_candidate_path = task.waypoint_path;
                try_hipac_online_bridge(task);
                if (!task.hipac_online_satisfied) {
                    try_hipac_transition_portal(task);
                }
                if (!task.hipac_online_satisfied) {
                    try_hipac_prebridge_portal(task);
                }
                if (task.hipac_online_satisfied) {
                    batch_context.diagnostics().add_counter(
                        "query_bridge.batch_tasks_skipped_by_hipac_after_rrt");
                    batch_context.diagnostics().set_value(
                        query_bridge_task_key(task.index, "skipped_by_hipac_after_rrt"),
                        1.0);
                    batch_context.diagnostics().set_value(
                        query_bridge_task_key(task.index, "total_ms"),
                        elapsed_ms_since(batch_t0) - prepared[task_offset].task_start_ms);
                    continue;
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
                    elapsed_ms_since(batch_t0) - prepared[task_offset].task_start_ms);
                continue;
            }
            const auto pave_t0 = Clock::now();
            bridge_query_with_waypoint_fallbacks(task, added_by_query[task.index]);
            const double pave_ms = elapsed_ms_since(pave_t0);
            batch_context.diagnostics().record_timing("query_bridge.batch_pave_ms_total",
                                                      pave_ms);
            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "pave_ms"),
                                                  pave_ms);
            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "added"),
                                                  static_cast<double>(added_by_query[task.index]));
            batch_context.diagnostics().set_value(
                query_bridge_task_key(task.index, "total_ms"),
                elapsed_ms_since(batch_t0) - prepared[task_offset].task_start_ms);
        }

        batch_context.diagnostics().set_value("query_bridge.batch_total_ms",
                                              elapsed_ms_since(batch_t0));
        return finish_batch_bridge();
    }

	    for (auto& task : tasks) {
	        const auto task_t0 = Clock::now();
	        const auto probe_t0 = Clock::now();
	        if (task.hipac_online_satisfied ||
                task.direct_start_goal_satisfied ||
                current_query_good(task, true)) {
	            batch_context.diagnostics().add_counter("query_bridge.batch_tasks_skipped");
	            batch_context.diagnostics().record_timing("query_bridge.batch_probe_ms_total",
	                                                      elapsed_ms_since(probe_t0));
	            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "skipped"),
	                                                  1.0);
	            if (task.hipac_online_satisfied) {
	                batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "skipped_by_hipac_online"),
	                                                      1.0);
	            }
                if (task.direct_start_goal_satisfied) {
                    batch_context.diagnostics().set_value(
                        query_bridge_task_key(task.index, "skipped_by_direct_start_goal_segment"),
                        1.0);
                }
	            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "total_ms"),
	                                                  elapsed_ms_since(task_t0));
	            continue;
        }
        batch_context.diagnostics().record_timing("query_bridge.batch_probe_ms_total",
                                                  elapsed_ms_since(probe_t0));
        batch_context.diagnostics().add_counter("query_bridge.batch_tasks_attempted");
        const bool forced_task = query_bridge_forced(task);
        const int attempts = forced_task
            ? std::max(std::max(1, task.attempts), retry_options.forced_attempts)
            : std::max(1, task.attempts);
        int effective_attempts =
            task.waypoint_path_from_partition_query && !task.waypoint_path.empty()
                ? 0
                : attempts;
        if (effective_attempts > 0 &&
            !retry_options.local_radius_schedule.empty() &&
            retry_options.local_radius_append_unrestricted_attempt) {
            effective_attempts = std::max(
                effective_attempts,
                static_cast<int>(retry_options.local_radius_schedule.size()) + 1);
        }
        if (forced_task) {
            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "forced"),
                                                  1.0);
        }
        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "attempts"),
                                              static_cast<double>(effective_attempts));
        if (task.waypoint_path_from_partition_query && !task.waypoint_path.empty()) {
            batch_context.diagnostics().add_counter(
                "query_bridge.partition_path_first_tasks");
            batch_context.diagnostics().add_counter(
                "query_bridge.partition_path_first_rrt_skipped");
            batch_context.diagnostics().set_value(
                query_bridge_task_key(task.index, "partition_path_first"),
                1.0);
            batch_context.diagnostics().set_value(
                query_bridge_task_key(task.index, "waypoint_from_partition_path"),
                1.0);
        }
        std::vector<std::vector<Eigen::VectorXd>> attempt_paths(static_cast<std::size_t>(effective_attempts));
        const auto rrt_t0 = Clock::now();
        if (batch_context.executor().n_threads() > 1 && effective_attempts > 1) {
            std::shared_ptr<std::atomic<bool>> local_cancel =
                parallel_rrt_options.early_stop ? std::make_shared<std::atomic<bool>>(false)
                                                : batch_context.native_cancel_flag();
            std::atomic<int> early_successes{0};
            batch_context.executor().parallel_for(0, effective_attempts, [&](int attempt) {
                if (local_cancel && local_cancel->load(std::memory_order_relaxed)) {
                    return;
                }
                auto path = run_task_attempt(task, attempt, 0, local_cancel);
                if (parallel_rrt_options.early_stop &&
                    rrt_path_good_enough_for_task(task, path)) {
                    const int successes =
                        early_successes.fetch_add(1, std::memory_order_relaxed) + 1;
                    if (successes >= parallel_rrt_options.early_stop_min_successes &&
                        local_cancel) {
                        local_cancel->store(true, std::memory_order_relaxed);
                    }
                }
                attempt_paths[static_cast<std::size_t>(attempt)] = std::move(path);
            });
            if (parallel_rrt_options.early_stop) {
                batch_context.diagnostics().add_counter(
                    "query_bridge.parallel_rrt_early_stop_successes",
                    static_cast<double>(early_successes.load(std::memory_order_relaxed)));
                batch_context.diagnostics().add_counter(
                    local_cancel && local_cancel->load(std::memory_order_relaxed)
                        ? "query_bridge.parallel_rrt_early_stop_triggered"
                        : "query_bridge.parallel_rrt_early_stop_not_triggered");
            }
        } else {
            for (int attempt = 0; attempt < effective_attempts; ++attempt) {
                attempt_paths[static_cast<std::size_t>(attempt)] = run_task_attempt(task, attempt, 0);
            }
        }
        const double rrt_ms = elapsed_ms_since(rrt_t0);
        batch_context.diagnostics().record_timing("query_bridge.batch_rrt_ms_total",
                                                  rrt_ms);
        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "rrt_ms"),
                                              rrt_ms);
        double best_length = std::numeric_limits<double>::infinity();
        if (task.waypoint_path_from_partition_query && !task.waypoint_path.empty()) {
            best_length = path_length(task.waypoint_path);
        }
        select_attempt_paths(task, attempt_paths, best_length);
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
                                   attempts,
                                   best_length,
                                   task.waypoint_path);
        const bool segment_only_task =
            query_bridge_index_segment_only(index_options, task.index);
        if (task.waypoint_path.empty() && segment_only_task &&
            retry_options.segment_only_retry_attempts > 0) {
            const auto retry_t0 = Clock::now();
            int retry_successes = 0;
            for (int retry = 0; retry < retry_options.segment_only_retry_attempts; ++retry) {
                auto retry_path = run_task_attempt(task, attempts + retry, 0);
                if (retry_path.empty()) {
                    continue;
                }
                retry_successes += 1;
                const double length = path_length(retry_path);
                if (length < best_length) {
                    best_length = length;
                    task.waypoint_path = std::move(retry_path);
                }
                if (retry_options.no_path_retry_stop_on_first_success) {
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
        }
        if (task.waypoint_path.empty() && !segment_only_task &&
            (retry_options.no_path_retry_attempts > 0 || retry_options.no_path_retry_budget_stages > 0)) {
            int retry_attempt_offset = attempts;
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
                    auto retry_path =
                        run_task_attempt(task, retry_attempt_offset + retry, stage_fixed_iters);
                    retry_attempts_run += 1;
                    if (retry_path.empty()) {
                        continue;
                    }
                    retry_successes += 1;
                    const double length = path_length(retry_path);
                    if (length < best_length) {
                        best_length = length;
                        task.waypoint_path = std::move(retry_path);
                    }
                    if (retry_options.no_path_retry_stop_on_first_success) {
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
                        : query_bridge_task_key(task.index,
                                   "no_path_retry_stage." + std::to_string(stage_index) + ".");
                batch_context.diagnostics().set_value(key_prefix + "attempts",
                                                      static_cast<double>(effective_stage_attempts));
                batch_context.diagnostics().set_value(key_prefix + "attempts_run",
                                                      static_cast<double>(retry_attempts_run));
                batch_context.diagnostics().set_value(key_prefix + "successes",
                                                      static_cast<double>(retry_successes));
                batch_context.diagnostics().set_value(key_prefix + "fixed_iters",
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
                run_no_path_retry_stage(static_cast<int>(stage) + 1,
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
        }
        if (task.waypoint_path.empty()) {
            batch_context.diagnostics().add_counter("query_bridge.batch_tasks_no_path");
            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "no_path"),
                                                  1.0);
            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "total_ms"),
                                                  elapsed_ms_since(task_t0));
            continue;
        }
        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "waypoint_length"),
                                              best_length);
        const auto second_probe_t0 = Clock::now();
        if (current_query_good(task, !retry_options.post_rrt_skip_forced)) {
            batch_context.diagnostics().add_counter("query_bridge.batch_tasks_skipped_after_rrt");
            if (forced_task) {
                batch_context.diagnostics().add_counter(
                    "query_bridge.batch_forced_tasks_skipped_after_rrt");
            }
            batch_context.diagnostics().record_timing("query_bridge.batch_probe_ms_total",
                                                      elapsed_ms_since(second_probe_t0));
            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "skipped_after_rrt"),
                                                  1.0);
            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "total_ms"),
                                                  elapsed_ms_since(task_t0));
            continue;
        }
        batch_context.diagnostics().record_timing("query_bridge.batch_probe_ms_total",
                                                  elapsed_ms_since(second_probe_t0));
        const int hipac_resolve_cap =
            std::max(0, last_adaptive_partition_config_.hipac_online_max_resolves_per_query);
        const bool can_run_hipac_online =
            task.hipac_online_resolves_used < hipac_resolve_cap ||
            (last_adaptive_partition_config_.hipac_online_transition_portal &&
             task.hipac_transition_resolves_used <
                 std::max(0, last_adaptive_partition_config_.hipac_transition_max_attempts_per_query)) ||
            (last_adaptive_partition_config_.hipac_online_prebridge_portal &&
             task.hipac_prebridge_resolves_used < hipac_resolve_cap);
        if (last_adaptive_partition_config_.hipac_online_connectivity &&
            !task.waypoint_path.empty() &&
            can_run_hipac_online) {
            task.hipac_candidate_path = task.waypoint_path;
            try_hipac_online_bridge(task);
            if (!task.hipac_online_satisfied) {
                try_hipac_transition_portal(task);
            }
            if (!task.hipac_online_satisfied) {
                try_hipac_prebridge_portal(task);
            }
            if (task.hipac_online_satisfied) {
                batch_context.diagnostics().add_counter(
                    "query_bridge.batch_tasks_skipped_by_hipac_after_rrt");
                batch_context.diagnostics().set_value(
                    query_bridge_task_key(task.index, "skipped_by_hipac_after_rrt"),
                    1.0);
                batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "total_ms"),
                                                      elapsed_ms_since(task_t0));
                continue;
            }
        }
        const int fast_direct_added = try_fast_direct_segment_after_rrt(task);
        if (fast_direct_added > 0) {
            added_by_query[task.index] += fast_direct_added;
            batch_context.diagnostics().set_value(
                query_bridge_task_key(task.index, "fast_direct_segment_after_rrt"),
                1.0);
            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "added"),
                                                  static_cast<double>(added_by_query[task.index]));
            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "total_ms"),
                                                  elapsed_ms_since(task_t0));
            continue;
        }
        if (segment_only_task) {
            const int source_box_id = locate_box_partition_first(task.start,
                                                                 config_.query.nearest_if_outside);
            const int target_box_id = locate_box_partition_first(task.goal,
                                                                 config_.query.nearest_if_outside);
            int edge_id = -1;
            if (source_box_id >= 0 && target_box_id >= 0) {
                edge_id = add_segment_edge_partition_first(                                           source_box_id,
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
                batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "segment_only"),
                                                      1.0);
                batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "added"),
                                                      1.0);
            } else {
                batch_context.diagnostics().add_counter(
                    "query_bridge.batch_tasks_segment_only_failures");
                batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "segment_only_failure"),
                                                      1.0);
            }
            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "total_ms"),
                                                  elapsed_ms_since(task_t0));
            continue;
        }
        const auto pave_t0 = Clock::now();
        bridge_query_with_waypoint_fallbacks(task, added_by_query[task.index]);
        const double pave_ms = elapsed_ms_since(pave_t0);
        batch_context.diagnostics().record_timing("query_bridge.batch_pave_ms_total",
                                                  pave_ms);
        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "pave_ms"),
                                              pave_ms);
        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "added"),
                                              static_cast<double>(added_by_query[task.index]));
        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "total_ms"),
                                              elapsed_ms_since(task_t0));
    }

    batch_context.diagnostics().set_value("query_bridge.batch_total_ms",
                                          elapsed_ms_since(batch_t0));
    return finish_batch_bridge();
}

} // namespace rbf
