#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>
#include <SBF/connector.h>

#include "env_config.h"
#include "planning_forest_audit.h"
#include "planning_forest_query_bridge_corridor_utils.h"
#include "planning_forest_diagnostics.h"
#include "planning_forest_qroot_helpers.h"
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

using detail::env_double_list_or_empty;
using detail::env_double_or_default;
using detail::env_index_list_contains;
using detail::env_indexed_double_or_default;
using detail::env_indexed_int_or_default;
using detail::env_int_list_or_empty;
using detail::env_int_or_default;

int RBFPlanningForest::bridge_query_with_waypoint_path(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    const std::vector<Eigen::VectorXd>& waypoint_path,
    bool short_local_bridge,
    const RRTConnectConfig& bridge_rrt,
    int query_index,
    bool allow_residual_segments) {
    if (waypoint_path.empty() || boxes_.empty() || !oracle_) {
        return 0;
    }
    auto finish_bridge = [&](int added_total) {
        sync_adaptive_partition_segment_edges(&last_build_, "query_bridge");
        if (added_total > 0) {
            refresh_adaptive_partition_diagnostics(&last_build_);
        }
        return added_total;
    };
    const int start_box_id = locate_box_partition_first(start, config_.query.nearest_if_outside);
    if (start_box_id < 0) {
        return 0;
    }
    const int goal_box_id = locate_box_partition_first(goal, config_.query.nearest_if_outside);
    if (goal_box_id < 0 || goal_box_id == start_box_id) {
        return 0;
    }
    StageContext context = StageContext::from_runtime(config_.runtime);
    const QueryBridgeEdgeRuntimeOptions edge_options =
        query_bridge_edge_runtime_options();
    const int bridge_edge_query_index =
        edge_options.scene_reusable_edges ? -1 : query_index;
    context.diagnostics().set_value("query_bridge.scene_reusable_edges",
                                    edge_options.scene_reusable_edges ? 1.0 : 0.0);
    context.diagnostics().set_value("query_bridge.direct_segment_after_rrt",
                                    edge_options.direct_segment_after_rrt ? 1.0 : 0.0);
    context.diagnostics().set_value("query_bridge.direct_segment_after_rrt_min_length",
                                    edge_options.direct_segment_after_rrt_min_length);
    struct QueryBridgePaveDiagnosticsFlush {
        BuildProfile& profile;
        StageContext& context;
        ~QueryBridgePaveDiagnosticsFlush() {
            for (const auto& [key, value] : context.diagnostics().snapshot()) {
                profile.diagnostics[key] = value;
            }
        }
    } pave_diagnostics_flush{last_build_, context};
    CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
    const double bridge_waypoint_length = query_bridge_waypoint_length(waypoint_path);
    const bool direct_segment_after_rrt_candidate =
        edge_options.direct_segment_after_rrt &&
        bridge_waypoint_length >= edge_options.direct_segment_after_rrt_min_length &&
        config_.connector.segment_edges_enabled &&
        config_.connector.rrt_segment_edges;
    context.diagnostics().set_value("query_bridge.direct_segment_after_rrt_candidate",
                                    direct_segment_after_rrt_candidate ? 1.0 : 0.0);
    int direct_segment_edges_added = 0;
    int box_corridor_edges_added = 0;
    const bool defer_query_segment_edge = true;
    const double query_bridge_depth_failures_before =
        boundary_max_depth_failure_count_local(context);
    int next_id = next_box_id();
    auto append_partition_after_pave = [&](std::size_t boxes_before, const char* prefix) {
        if (boxes_.size() > boxes_before) {
            append_adaptive_partition_boxes(boxes_before, &last_build_, prefix);
        }
    };
    auto capped_ffb_depth = [&](int requested_depth) {
        const int max_tree_depth = std::max(1, config_.database.max_tree_depth);
        return std::min(max_tree_depth, std::max(1, requested_depth));
    };
    const int query_bridge_ffb_depth = capped_ffb_depth(
        config_.query_bridge_pave_depth > 0
            ? config_.query_bridge_pave_depth
            : config_.connector.pave.find_free_box.max_depth);
    context.diagnostics().set_value("query_bridge.pave_ffb_depth",
                                    static_cast<double>(query_bridge_ffb_depth));
    auto set_query_bridge_task_value = [&](const std::string& suffix, double value) {
        if (query_index < 0) {
            return;
        }
        context.diagnostics().set_value(
            "query_bridge.batch_task." + std::to_string(query_index) + "." + suffix,
            value);
    };
    std::vector<Eigen::VectorXd> corridor_path = waypoint_path;
    const QueryBridgeWaypointShortcutOptions waypoint_shortcut_options =
        query_bridge_waypoint_shortcut_options(direct_segment_after_rrt_candidate);
    context.diagnostics().set_value("query_bridge.waypoint_shortcut_enabled",
                                    waypoint_shortcut_options.enabled ? 1.0 : 0.0);
    if (waypoint_shortcut_options.enabled && corridor_path.size() > 2) {
        using Clock = std::chrono::steady_clock;
        const auto shortcut_t0 = Clock::now();
        const double before_length = query_bridge_waypoint_length(corridor_path);
        std::vector<Eigen::VectorXd> shortened =
            collision_shortcut_path(corridor_path,
                                    checker,
                                    collision_shortcut_resolution(config_.query));
        const double after_length = query_bridge_waypoint_length(shortened);
        context.diagnostics().add_counter("query_bridge.waypoint_shortcut_attempts");
        set_query_bridge_task_value("waypoint_shortcut_before_length", before_length);
        set_query_bridge_task_value("waypoint_shortcut_after_length", after_length);
        if (!shortened.empty() &&
            after_length + waypoint_shortcut_options.min_gain < before_length) {
            const PathAuditCheck shortcut_audit =
                audit_waypoint_path(shortened,
                                    checker,
                                    config_.query.audit_resolution,
                                    config_.query.audit_segment_step);
            if (shortcut_audit.passed) {
                context.diagnostics().add_counter("query_bridge.waypoint_shortcut_accepts");
                context.diagnostics().add_counter("query_bridge.waypoint_shortcut_delta",
                                                  before_length - after_length);
                set_query_bridge_task_value("waypoint_shortcut_accepted", 1.0);
                set_query_bridge_task_value("waypoint_shortcut_delta",
                                            before_length - after_length);
                corridor_path = std::move(shortened);
            } else {
                context.diagnostics().add_counter("query_bridge.waypoint_shortcut_audit_rejects");
                set_query_bridge_task_value("waypoint_shortcut_audit_reject", 1.0);
            }
        }
        context.diagnostics().record_timing(
            "query_bridge.waypoint_shortcut_ms_total",
            std::chrono::duration<double, std::milli>(Clock::now() -
                                                      shortcut_t0).count());
    }
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
    auto locate_query_boxes = [&]() {
        using Clock = std::chrono::steady_clock;
        const auto locate_t0 = Clock::now();
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
        return std::pair<int, int>{source_box_id, target_box_id};
    };
    auto query_boxes_connected = [&](int source_box_id, int target_box_id) {
        if (source_box_id < 0 || target_box_id < 0) {
            return false;
        }
        return box_only_path_connected_partition_first(source_box_id, target_box_id);
    };
    auto try_reverse_boundary_pave =
        [&](const ChainPaveConfig& forward_config,
            int forward_added,
            int& accumulated_added) -> std::pair<int, int> {
        auto [source_box_id, target_box_id] = locate_query_boxes();
        if (query_boxes_connected(source_box_id, target_box_id)) {
            return {source_box_id, target_box_id};
        }
        if (partition_native_mode()) {
            context.diagnostics().add_counter(
                "query_bridge.partition_legacy_reverse_chain_pave_skipped");
            return {source_box_id, target_box_id};
        }
        const int remaining_chain = forward_config.max_chain - std::max(0, forward_added);
        if (target_box_id < 0 || remaining_chain <= 0) {
            return {source_box_id, target_box_id};
        }
        ChainPaveConfig reverse_config = forward_config;
        reverse_config.max_chain = remaining_chain;
        std::vector<Eigen::VectorXd> reverse_path(corridor_path.rbegin(),
                                                  corridor_path.rend());
        context.diagnostics().add_counter("query_bridge.reverse_boundary_pave_attempts");
        const std::size_t boxes_before_reverse = boxes_.size();
        const int reverse_added = chain_pave_along_path(reverse_path,
                                                        target_box_id,
                                                        boxes_,
                                                        *oracle_,
                                                        adjacency_,
                                                        next_id,
                                                        context,
                                                        reverse_config);
        if (reverse_added > 0) {
            append_partition_after_pave(boxes_before_reverse,
                                        "query_bridge.reverse_boundary_pave");
            accumulated_added += reverse_added;
            context.diagnostics().add_counter("query_bridge.reverse_boundary_pave_added",
                                              static_cast<double>(reverse_added));
            context.diagnostics().add_counter(
                "query_bridge.full_adjacency_rebuilds_avoided");
            invalidate_query_cache();
        }
        return locate_query_boxes();
    };
    const bool bridge_internal_simplify =
        query_bridge_internal_simplify_enabled(direct_segment_after_rrt_candidate);
    context.diagnostics().set_value("query_bridge.internal_simplify_enabled",
                                    bridge_internal_simplify ? 1.0 : 0.0);
    if (bridge_internal_simplify &&
        config_.query.final_rrt_simplify &&
        config_.query.final_rrt_simplify_timeout_ms > 0.0 &&
        corridor_path.size() >= 2) {
        using Clock = std::chrono::steady_clock;
        const auto simplify_t0 = Clock::now();
        auto elapsed_ms = [&]() {
            return std::chrono::duration<double, std::milli>(Clock::now() -
                                                             simplify_t0)
                .count();
        };
        RRTConnectConfig simplify_config = config_.connector.rrt;
        simplify_config.max_iters =
            std::max(1, config_.query.final_rrt_simplify_max_iters);
        simplify_config.segment_resolution =
            std::max(simplify_config.segment_resolution,
                     config_.query.audit_resolution);
        simplify_config.segment_step = config_.query.audit_segment_step;
        simplify_config.shortcut_path = true;
        const int attempts = std::max(1, config_.query.final_rrt_simplify_attempts);
        double best_length = query_bridge_waypoint_length(corridor_path);
        for (int attempt = 0; attempt < attempts; ++attempt) {
            const double remaining_ms =
                config_.query.final_rrt_simplify_timeout_ms - elapsed_ms();
            if (remaining_ms <= 0.0) {
                break;
            }
            const int attempts_left = attempts - attempt;
            simplify_config.timeout_ms =
                std::max(1.0, remaining_ms / static_cast<double>(attempts_left));
            const int simplify_seed =
                derived_planner_seed(config_.grower.rng_seed,
                                     kSeedBridgeSimplifyOffset,
                                     attempt);
            auto candidate = rrt_connect(start,
                                         goal,
                                         checker,
                                         audit_robot_,
                                         simplify_config,
                                         simplify_seed);
            if (candidate.empty()) {
                continue;
            }
            const double candidate_length = query_bridge_waypoint_length(candidate);
            if (candidate_length + 1e-12 >= best_length) {
                continue;
            }
            if (audit_waypoint_path(candidate,
                                    checker,
                                    config_.query.audit_resolution,
                                    config_.query.audit_segment_step)
                    .passed) {
                best_length = candidate_length;
                corridor_path = std::move(candidate);
            }
        }
    }
    int dense_repair_added = 0;
    bool dense_repair_attempted = false;
    const double audited_bridge_length = query_bridge_waypoint_length(corridor_path);
    context.diagnostics().set_value("query_bridge.direct_segment_after_rrt_final_length",
                                    audited_bridge_length);
    if (direct_segment_after_rrt_candidate) {
        context.diagnostics().add_counter(
            "query_bridge.direct_segment_after_rrt_final_attempts");
        context.diagnostics().add_counter(
            "query_bridge.direct_segment_after_rrt_shortening_delta",
            std::max(0.0, bridge_waypoint_length - audited_bridge_length));
        const PathAuditCheck segment_audit =
            audit_waypoint_path(corridor_path,
                                checker,
                                config_.query.audit_resolution,
                                config_.query.audit_segment_step);
        if (!segment_audit.passed) {
            context.diagnostics().add_counter(
                "query_bridge.direct_segment_after_rrt_audit_rejects");
        } else {
            const int edge_id = add_segment_edge_partition_first(
                start_box_id,
                goal_box_id,
                corridor_path,
                SegmentEdgeType::QueryBridge,
                bridge_rrt.segment_resolution,
                SegmentEdgeValidation::CollisionChecked,
                true,
                bridge_edge_query_index);
            if (edge_id >= 0) {
                direct_segment_edges_added += 1;
                context.diagnostics().add_counter(
                    "query_bridge.direct_segment_after_rrt_edges");
                invalidate_query_cache();
                sync_adaptive_partition_segment_edges(
                    &last_build_,
                    "query_bridge.direct_segment_after_rrt");
                refresh_adaptive_partition_diagnostics(&last_build_);
                return finish_bridge(direct_segment_edges_added);
            }
            context.diagnostics().add_counter(
                "query_bridge.direct_segment_after_rrt_add_fail");
        }
    }
    const double dense_box_corridor_max_length =
        std::max(0.0, env_double_or_default("RBF_QUERY_BRIDGE_DIRECT_MAX_LENGTH", 6.5));
    const bool dense_box_corridor_candidate =
        defer_query_segment_edge &&
        audited_bridge_length > 0.0 &&
        audited_bridge_length <= dense_box_corridor_max_length;
    auto try_direct_ffb_corridor = [&]() -> int {
        using Clock = std::chrono::steady_clock;
        const auto t0 = Clock::now();
        const bool graphless_direct_corridor = partition_native_mode();
        const std::size_t boxes_before_direct_corridor = boxes_.size();
        auto finish_direct_corridor = [&](int value) {
            if (boxes_.size() > boxes_before_direct_corridor) {
                append_adaptive_partition_boxes(boxes_before_direct_corridor,
                                                &last_build_,
                                                "query_bridge.direct_corridor");
            }
            sync_adaptive_partition_segment_edges(&last_build_, "query_bridge.direct_corridor");
            return value;
        };
        auto refresh_direct_corridor_partition = [&]() {
            if (boxes_.size() > boxes_before_direct_corridor) {
                append_adaptive_partition_boxes(boxes_before_direct_corridor,
                                                &last_build_,
                                                "query_bridge.direct_corridor");
            }
            sync_adaptive_partition_segment_edges(&last_build_,
                                                  "query_bridge.direct_corridor");
        };
        const double audit_step = config_.query.audit_segment_step > 0.0
            ? config_.query.audit_segment_step
            : 0.01;
        const double base_sample_step =
            env_double_or_default("RBF_QUERY_BRIDGE_DIRECT_SAMPLE_STEP",
                                  audit_step);
        const double sample_step =
            std::max(1e-4,
                     env_indexed_double_or_default("RBF_QUERY_BRIDGE_DIRECT_SAMPLE_STEP",
                                                   query_index,
                                                   base_sample_step));
        context.diagnostics().set_value("query_bridge.direct_corridor_sample_step",
                                        sample_step);
        const std::vector<Eigen::VectorXd> samples =
            densify_waypoint_path_local(corridor_path, sample_step);
        if (samples.size() < 2) {
            return 0;
        }

        const bool use_partition_cover_index =
            graphless_direct_corridor &&
            adaptive_partition_query_enabled_ &&
            adaptive_partition_;
        const bool use_partition_neighbor_candidates =
            use_partition_cover_index &&
            env_int_or_default("RBF_QUERY_BRIDGE_PARTITION_NEIGHBOR_CANDIDATES", 0) != 0;
        const bool immediate_partition_append =
            use_partition_cover_index &&
            env_int_or_default("RBF_QUERY_BRIDGE_DIRECT_APPEND_PARTITION_IMMEDIATE", 0) != 0;
        const int partition_append_batch_size = immediate_partition_append
            ? std::max(1,
                       env_int_or_default(
                           "RBF_QUERY_BRIDGE_DIRECT_PARTITION_APPEND_BATCH_SIZE",
                           32))
            : 0;
        const bool detailed_direct_timing =
            env_int_or_default("RBF_QUERY_BRIDGE_DETAILED_TIMING", 0) != 0;
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_partition_neighbor_candidates_enabled",
            use_partition_neighbor_candidates ? 1.0 : 0.0);
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_immediate_partition_append_enabled",
            immediate_partition_append ? 1.0 : 0.0);
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_partition_append_batch_size",
            static_cast<double>(partition_append_batch_size));
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_detailed_timing_enabled",
            detailed_direct_timing ? 1.0 : 0.0);
        BoxSpatialIndex direct_box_index;
        if (!use_partition_cover_index) {
            direct_box_index.rebuild(boxes_, config_.query.adjacency_tolerance);
        }
        std::unordered_map<int, int> box_id_to_index;
        if (use_partition_neighbor_candidates) {
            box_id_to_index.reserve(boxes_.size() * 2);
            for (std::size_t box_index = 0; box_index < boxes_.size(); ++box_index) {
                box_id_to_index.emplace(boxes_[box_index].id, static_cast<int>(box_index));
            }
        }
        std::vector<int> corridor_new_box_indices;
        std::size_t direct_partition_append_base = boxes_.size();
        std::vector<std::vector<int>> sample_layers(samples.size());
        std::vector<bool> covered(samples.size(), false);
        std::vector<QueryBridgeResidualMilestone> repair_milestones;
        repair_milestones.reserve(samples.size());
        auto mark_from_index = [&](std::size_t from_index) {
            const auto mark_t0 = Clock::now();
            int changed = 0;
            for (std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {
                if (from_index == 0) {
                    std::vector<int> candidates;
                    if (use_partition_cover_index) {
                        candidates = adaptive_partition_->covering_box_indices(
                            samples[sample_index],
                            config_.query.adjacency_tolerance);
                    } else {
                        candidates = direct_box_index.point_candidates(samples[sample_index]);
                    }
                    for (int box_index : candidates) {
                        if (box_index < 0 || box_index >= static_cast<int>(boxes_.size())) {
                            continue;
                        }
                        if (!intervals_contain_point_local(
                                boxes_[static_cast<std::size_t>(box_index)].joint_intervals,
                                samples[sample_index],
                                config_.query.adjacency_tolerance)) {
                            continue;
                        }
                        auto& layer = sample_layers[sample_index];
                        if (std::find(layer.begin(), layer.end(), box_index) == layer.end()) {
                            layer.push_back(box_index);
                        }
                        if (!covered[sample_index]) {
                            covered[sample_index] = true;
                            changed += 1;
                        }
                    }
                    continue;
                }
                for (std::size_t box_index = from_index; box_index < boxes_.size(); ++box_index) {
                    if (!intervals_contain_point_local(boxes_[box_index].joint_intervals,
                                                       samples[sample_index],
                                                       config_.query.adjacency_tolerance)) {
                        continue;
                    }
                    auto& layer = sample_layers[sample_index];
                    const int index_value = static_cast<int>(box_index);
                    if (std::find(layer.begin(), layer.end(), index_value) == layer.end()) {
                        layer.push_back(index_value);
                    }
                    if (!covered[sample_index]) {
                        covered[sample_index] = true;
                        changed += 1;
                    }
                    if (from_index == 0) {
                        break;
                    }
                }
            }
            context.diagnostics().record_timing(
                from_index == 0
                    ? "query_bridge.direct_corridor_mark_initial_ms"
                    : "query_bridge.direct_corridor_mark_incremental_ms",
                std::chrono::duration<double, std::milli>(Clock::now() - mark_t0).count());
            return changed;
        };
        mark_from_index(0);

        QueryBridgeLocalDsu dsu(boxes_.size());
        double transition_connected_ms = 0.0;
        double bad_transitions_ms = 0.0;
        double current_cover_ms = 0.0;
        double current_cover_partition_ms = 0.0;
        double current_cover_corridor_scan_ms = 0.0;
        double current_cover_direct_index_ms = 0.0;
        double duplicate_lookup_ms = 0.0;
        double commit_total_ms = 0.0;
        double commit_dynamic_policy_ms = 0.0;
        double commit_partition_append_ms = 0.0;
        double assimilate_sample_scan_ms = 0.0;
        double assimilate_candidate_build_ms = 0.0;
        double assimilate_adjacency_ms = 0.0;
        double segment_insert_ms = 0.0;
        double direct_task_build_ms = 0.0;
        double direct_loop_ms = 0.0;
        double repair_loop_ms = 0.0;
        double adaptive_loop_ms = 0.0;
        double lateral_loop_ms = 0.0;
        double residual_segment_loop_ms = 0.0;
        int transition_connected_calls = 0;
        int bad_transitions_calls = 0;
        int current_cover_calls = 0;
        int duplicate_lookup_calls = 0;
        int commit_calls = 0;
        int assimilate_calls = 0;
        int assimilate_coverage_boxes = 0;
        int assimilate_coverage_span_max = 0;
        double assimilate_coverage_span_sum = 0.0;
        int segment_insert_calls = 0;
        int direct_partition_append_calls = 0;
        int direct_partition_append_boxes = 0;
        const bool local_assimilate_sample_scan =
            env_int_or_default("RBF_QUERY_BRIDGE_LOCAL_SAMPLE_ASSIMILATION", 1) != 0;
        int assimilate_local_hits = 0;
        int assimilate_full_scan_fallbacks = 0;
        int assimilate_local_sample_tests = 0;
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_local_sample_assimilation_enabled",
            local_assimilate_sample_scan ? 1.0 : 0.0);
        auto append_direct_partition_batch = [&](bool force) {
            if (!immediate_partition_append ||
                !adaptive_partition_ ||
                direct_partition_append_base >= boxes_.size()) {
                return 0;
            }
            const std::size_t pending = boxes_.size() - direct_partition_append_base;
            if (!force && pending < static_cast<std::size_t>(partition_append_batch_size)) {
                return 0;
            }
            const auto partition_append_t0 =
                detailed_direct_timing ? Clock::now() : Clock::time_point{};
            const int appended = adaptive_partition_->append_boxes(
                boxes_,
                direct_partition_append_base,
                config_.query.adjacency_tolerance);
            if (detailed_direct_timing) {
                commit_partition_append_ms +=
                    std::chrono::duration<double, std::milli>(Clock::now() -
                                                              partition_append_t0).count();
            }
            direct_partition_append_calls += 1;
            direct_partition_append_boxes += std::max(0, appended);
            context.diagnostics().add_counter(
                appended > 0
                    ? "query_bridge.direct_corridor_batched_partition_appends"
                    : "query_bridge.direct_corridor_batched_partition_append_rejects");
            direct_partition_append_base = boxes_.size();
            return appended;
        };
        auto transition_connected = [&](int transition) {
            const auto timing_t0 = detailed_direct_timing ? Clock::now() : Clock::time_point{};
            auto finish = [&](bool value) {
                if (detailed_direct_timing) {
                    transition_connected_ms +=
                        std::chrono::duration<double, std::milli>(Clock::now() - timing_t0).count();
                    transition_connected_calls += 1;
                }
                return value;
            };
            if (transition < 0 || transition + 1 >= static_cast<int>(sample_layers.size())) {
                return finish(false);
            }
            const auto& lhs_layer = sample_layers[static_cast<std::size_t>(transition)];
            const auto& rhs_layer = sample_layers[static_cast<std::size_t>(transition + 1)];
            if (lhs_layer.empty() || rhs_layer.empty()) {
                return finish(false);
            }
            for (int lhs : lhs_layer) {
                const int root = dsu.find(lhs);
                for (int rhs : rhs_layer) {
                    if (root == dsu.find(rhs)) {
                        return finish(true);
                    }
                }
            }
            return finish(false);
        };
        auto bad_transitions = [&]() {
            const auto timing_t0 = detailed_direct_timing ? Clock::now() : Clock::time_point{};
            std::vector<int> bad;
            for (std::size_t sample_index = 0; sample_index + 1 < sample_layers.size(); ++sample_index) {
                if (!transition_connected(static_cast<int>(sample_index))) {
                    bad.push_back(static_cast<int>(sample_index));
                }
            }
            if (detailed_direct_timing) {
                bad_transitions_ms +=
                    std::chrono::duration<double, std::milli>(Clock::now() - timing_t0).count();
                bad_transitions_calls += 1;
            }
            return bad;
        };
        auto endpoint_layers_connected = [&]() {
            if (sample_layers.empty() ||
                sample_layers.front().empty() ||
                sample_layers.back().empty()) {
                return false;
            }
            const int root = dsu.find(sample_layers.front().front());
            for (int index : sample_layers.back()) {
                if (root == dsu.find(index)) {
                    return true;
                }
            }
            return false;
        };
        auto direct_boxes_adjacent = [&](int lhs, int rhs) {
            if (lhs < 0 || rhs < 0 ||
                lhs >= static_cast<int>(boxes_.size()) ||
                rhs >= static_cast<int>(boxes_.size())) {
                return false;
            }
            const int lhs_box_id = boxes_[static_cast<std::size_t>(lhs)].id;
            const int rhs_box_id = boxes_[static_cast<std::size_t>(rhs)].id;
            if (graphless_direct_corridor &&
                use_partition_cover_index &&
                adaptive_partition_ &&
                adaptive_partition_->contains_box_id(lhs_box_id) &&
                adaptive_partition_->contains_box_id(rhs_box_id)) {
                context.diagnostics().add_counter(
                    "query_bridge.direct_corridor_partition_neighbor_tests");
                const bool adjacent = adaptive_partition_->boxes_are_neighbors(lhs_box_id,
                                                                               rhs_box_id);
                if (adjacent) {
                    context.diagnostics().add_counter(
                        "query_bridge.direct_corridor_partition_neighbor_hits");
                }
                return adjacent;
            }
            return boxes_connected(boxes_[static_cast<std::size_t>(lhs)],
                                   boxes_[static_cast<std::size_t>(rhs)],
                                   config_.query.adjacency_tolerance);
        };
        auto initialize_dsu = [&]() {
            const auto dsu_t0 = Clock::now();
            for (const auto& layer : sample_layers) {
                if (layer.empty()) {
                    continue;
                }
                const int root = layer.front();
                for (int index : layer) {
                    dsu.unite(root, index);
                }
            }
            for (std::size_t sample_index = 0; sample_index + 1 < sample_layers.size(); ++sample_index) {
                for (int lhs : sample_layers[sample_index]) {
                    for (int rhs : sample_layers[sample_index + 1]) {
                        if (direct_boxes_adjacent(lhs, rhs)) {
                            dsu.unite(lhs, rhs);
                            if (!graphless_direct_corridor) {
                                append_local_edge(adjacency_,
                                                  boxes_[static_cast<std::size_t>(lhs)].id,
                                                  boxes_[static_cast<std::size_t>(rhs)].id);
                            }
                        }
                    }
                }
            }
            context.diagnostics().record_timing(
                "query_bridge.direct_corridor_initialize_dsu_ms",
                std::chrono::duration<double, std::milli>(Clock::now() - dsu_t0).count());
        };
        initialize_dsu();

        std::unordered_map<OracleNodeId, int> node_to_box_index;
        node_to_box_index.reserve(boxes_.size() + samples.size());
        for (std::size_t box_index = 0; box_index < boxes_.size(); ++box_index) {
            const auto node = static_cast<OracleNodeId>(boxes_[box_index].tree_id);
            if (node != kInvalidOracleNodeId &&
                node_to_box_index.find(node) == node_to_box_index.end()) {
                node_to_box_index[node] = static_cast<int>(box_index);
            }
        }
        auto find_duplicate_box_index = [&](OracleNodeId node,
                                            const std::vector<Interval>& intervals) {
            const auto timing_t0 = detailed_direct_timing ? Clock::now() : Clock::time_point{};
            auto finish = [&](int value) {
                if (detailed_direct_timing) {
                    duplicate_lookup_ms +=
                        std::chrono::duration<double, std::milli>(Clock::now() - timing_t0).count();
                    duplicate_lookup_calls += 1;
                }
                return value;
            };
            if (node != kInvalidOracleNodeId) {
                const auto node_it = node_to_box_index.find(node);
                if (node_it != node_to_box_index.end()) {
                    return finish(node_it->second);
                }
                return finish(-1);
            }
            for (std::size_t box_index = 0; box_index < boxes_.size(); ++box_index) {
                const auto& box = boxes_[box_index];
                if (box.joint_intervals.size() != intervals.size()) {
                    continue;
                }
                bool same = true;
                for (std::size_t dim = 0; dim < intervals.size(); ++dim) {
                    if (std::abs(box.joint_intervals[dim].lo - intervals[dim].lo) > 1e-12 ||
                        std::abs(box.joint_intervals[dim].hi - intervals[dim].hi) > 1e-12) {
                        same = false;
                        break;
                    }
                }
                if (same) {
                    return finish(static_cast<int>(box_index));
                }
            }
            return finish(-1);
        };
        std::vector<int> repair_indices;
        auto assimilate_box = [&](int box_index, int transition_hint) {
            if (detailed_direct_timing) {
                assimilate_calls += 1;
            }
            const auto assimilate_t0 = Clock::now();
            const int box_id = boxes_[static_cast<std::size_t>(box_index)].id;
            if (!graphless_direct_corridor) {
                adjacency_[box_id];
            }
            int first_covered_sample = static_cast<int>(samples.size());
            int last_covered_sample = -1;
            int covered_sample_count = 0;
            const auto sample_scan_t0 = detailed_direct_timing ? Clock::now() : Clock::time_point{};
            const auto& box_intervals = boxes_[static_cast<std::size_t>(box_index)].joint_intervals;
            auto record_sample_coverage = [&](std::size_t sample_index) {
                const int sample_index_int = static_cast<int>(sample_index);
                first_covered_sample = std::min(first_covered_sample, sample_index_int);
                last_covered_sample = std::max(last_covered_sample, sample_index_int);
                covered_sample_count += 1;
                auto& layer = sample_layers[sample_index];
                if (!layer.empty()) {
                    dsu.unite(box_index, layer.front());
                }
                if (std::find(layer.begin(), layer.end(), box_index) == layer.end()) {
                    layer.push_back(box_index);
                }
                covered[sample_index] = true;
            };
            auto sample_in_box = [&](int sample_index) {
                if (sample_index < 0 || sample_index >= static_cast<int>(samples.size())) {
                    return false;
                }
                assimilate_local_sample_tests += 1;
                return intervals_contain_point_local(
                    box_intervals,
                    samples[static_cast<std::size_t>(sample_index)],
                    config_.query.adjacency_tolerance);
            };
            bool used_full_sample_scan = true;
            if (local_assimilate_sample_scan && !samples.empty()) {
                used_full_sample_scan = false;
                int anchor = -1;
                const std::array<int, 5> anchors = {
                    transition_hint,
                    transition_hint + 1,
                    transition_hint - 1,
                    transition_hint + 2,
                    transition_hint - 2,
                };
                for (int candidate_anchor : anchors) {
                    if (sample_in_box(candidate_anchor)) {
                        anchor = candidate_anchor;
                        break;
                    }
                }
                if (anchor >= 0) {
                    int left = anchor;
                    int right = anchor;
                    while (left > 0 && sample_in_box(left - 1)) {
                        --left;
                    }
                    while (right + 1 < static_cast<int>(samples.size()) &&
                           sample_in_box(right + 1)) {
                        ++right;
                    }
                    for (int sample_index = left; sample_index <= right; ++sample_index) {
                        record_sample_coverage(static_cast<std::size_t>(sample_index));
                    }
                    assimilate_local_hits += 1;
                } else {
                    used_full_sample_scan = true;
                    assimilate_full_scan_fallbacks += 1;
                }
            }
            if (used_full_sample_scan) {
                for (std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {
                    if (!intervals_contain_point_local(box_intervals,
                                                       samples[sample_index],
                                                       config_.query.adjacency_tolerance)) {
                        continue;
                    }
                    record_sample_coverage(sample_index);
                }
            }
            if (covered_sample_count > 0) {
                const int span = last_covered_sample - first_covered_sample + 1;
                assimilate_coverage_boxes += 1;
                assimilate_coverage_span_sum += static_cast<double>(span);
                assimilate_coverage_span_max = std::max(assimilate_coverage_span_max, span);
                context.diagnostics().add_counter(
                    "query_bridge.direct_corridor_assimilate_covered_samples",
                    static_cast<double>(covered_sample_count));
            }
            if (detailed_direct_timing) {
                assimilate_sample_scan_ms +=
                    std::chrono::duration<double, std::milli>(Clock::now() - sample_scan_t0).count();
            }
            const auto candidate_build_t0 = detailed_direct_timing ? Clock::now() : Clock::time_point{};
            std::vector<int> candidates;
            auto add_layer = [&](int layer_index) {
                if (layer_index < 0 || layer_index >= static_cast<int>(sample_layers.size())) {
                    return;
                }
                const auto& layer = sample_layers[static_cast<std::size_t>(layer_index)];
                candidates.insert(candidates.end(), layer.begin(), layer.end());
            };
            add_layer(transition_hint - 1);
            add_layer(transition_hint);
            add_layer(transition_hint + 1);
            add_layer(transition_hint + 2);
            if (covered_sample_count > 0) {
                add_layer(first_covered_sample - 1);
                add_layer(first_covered_sample);
                add_layer(first_covered_sample + 1);
                add_layer(last_covered_sample - 1);
                add_layer(last_covered_sample);
                add_layer(last_covered_sample + 1);
            }
            candidates.insert(candidates.end(), repair_indices.begin(), repair_indices.end());
            if (use_partition_neighbor_candidates && adaptive_partition_) {
                const auto partition_neighbor_ids =
                    adaptive_partition_->adjacent_box_ids(
                        boxes_[static_cast<std::size_t>(box_index)],
                        config_.query.adjacency_tolerance);
                context.diagnostics().add_counter(
                    "query_bridge.direct_corridor_partition_neighbor_candidates",
                    static_cast<double>(partition_neighbor_ids.size()));
                for (int neighbor_box_id : partition_neighbor_ids) {
                    const auto index_it = box_id_to_index.find(neighbor_box_id);
                    if (index_it != box_id_to_index.end()) {
                        candidates.push_back(index_it->second);
                    }
                }
            }
            std::sort(candidates.begin(), candidates.end());
            candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
            if (detailed_direct_timing) {
                assimilate_candidate_build_ms +=
                    std::chrono::duration<double, std::milli>(Clock::now() - candidate_build_t0).count();
            }
            const auto adjacency_t0 = detailed_direct_timing ? Clock::now() : Clock::time_point{};
            int local_edges = 0;
            for (int candidate : candidates) {
                if (candidate == box_index ||
                    candidate < 0 ||
                    candidate >= static_cast<int>(boxes_.size())) {
                    continue;
                }
                if (direct_boxes_adjacent(box_index, candidate)) {
                    dsu.unite(box_index, candidate);
                    bool edge_counted = true;
                    if (!graphless_direct_corridor) {
                        const std::size_t before = adjacency_[box_id].size();
                        append_local_edge(adjacency_,
                                          box_id,
                                          boxes_[static_cast<std::size_t>(candidate)].id);
                        edge_counted = adjacency_[box_id].size() > before;
                    }
                    if (edge_counted) {
                        local_edges += 1;
                    }
                }
            }
            if (detailed_direct_timing) {
                assimilate_adjacency_ms +=
                    std::chrono::duration<double, std::milli>(Clock::now() - adjacency_t0).count();
            }
            context.diagnostics().add_counter(
                "query_bridge.direct_corridor_incremental_adjacency_checks",
                static_cast<double>(candidates.size()));
            context.diagnostics().add_counter(
                "query_bridge.direct_corridor_incremental_adjacency_edges",
                static_cast<double>(local_edges));
            context.diagnostics().add_counter(
                "query_bridge.direct_corridor_full_adjacency_scans_avoided");
            context.diagnostics().record_timing(
                "query_bridge.direct_corridor_assimilate_ms",
                std::chrono::duration<double, std::milli>(Clock::now() - assimilate_t0).count());
            return covered_sample_count;
        };
        bool adopt_certified_subchain_attempted = false;
        auto try_adopt_certified_subchain = [&](int source_box_id,
                                                int target_box_id,
                                                const char* reason) -> int {
            const auto adopt_t0 = Clock::now();
            auto& diagnostics = context.diagnostics();
            auto finish_adopt = [&](int value) {
                const double elapsed =
                    std::chrono::duration<double, std::milli>(Clock::now() - adopt_t0).count();
                diagnostics.record_timing("query_bridge.hipac_promote_transition.ms_total",
                                          elapsed);
                diagnostics.add_counter("query_bridge.hipac_promote_transition.ms_total",
                                        elapsed);
                set_query_bridge_task_value("hipac_promote_transition_ms", elapsed);
                if (value > 0) {
                    set_query_bridge_task_value("hipac_promote_transition_added",
                                                static_cast<double>(value));
                }
                if (reason != nullptr) {
                    diagnostics.set_value(std::string("query_bridge.hipac_promote_transition.reason.") +
                                              reason,
                                          1.0);
                }
                return value;
            };
            if (adopt_certified_subchain_attempted) {
                diagnostics.add_counter("query_bridge.hipac_promote_transition.skipped_repeat");
                return finish_adopt(0);
            }
            adopt_certified_subchain_attempted = true;
            if (!last_adaptive_partition_config_.hipac_online_connectivity ||
                !last_adaptive_partition_config_.hipac_promote_transition_slices ||
                last_adaptive_partition_config_.hipac_promote_transition_max_attempts_per_query <= 0 ||
                !partition_native_mode() ||
                source_box_id < 0 ||
                target_box_id < 0 ||
                source_box_id == target_box_id) {
                diagnostics.add_counter("query_bridge.hipac_promote_transition.disabled");
                return finish_adopt(0);
            }
            const bool target_index =
                csv_index_list_contains(
                    last_adaptive_partition_config_.hipac_promote_transition_target_query_indices,
                    query_index);
            if (!target_index) {
                diagnostics.add_counter("query_bridge.hipac_promote_transition.target_rejects");
                return finish_adopt(0);
            }
            diagnostics.add_counter("query_bridge.hipac_promote_transition.attempts");

            const int min_boxes =
                std::max(1, last_adaptive_partition_config_.hipac_promote_transition_min_boxes);
            const int max_boxes =
                std::max(min_boxes, last_adaptive_partition_config_.hipac_promote_transition_max_boxes);
            const BoxNode* source_box_ptr = find_box_by_id(boxes_, source_box_id);
            const BoxNode* target_box_ptr = find_box_by_id(boxes_, target_box_id);
            if (source_box_ptr == nullptr || target_box_ptr == nullptr) {
                diagnostics.add_counter("query_bridge.hipac_promote_transition.missing_endpoint_box");
                return finish_adopt(0);
            }
            const auto source_it =
                std::find_if(boxes_.begin(), boxes_.end(), [&](const BoxNode& box) {
                    return box.id == source_box_id;
                });
            const auto target_it =
                std::find_if(boxes_.begin(), boxes_.end(), [&](const BoxNode& box) {
                    return box.id == target_box_id;
                });
            if (source_it == boxes_.end() || target_it == boxes_.end()) {
                diagnostics.add_counter("query_bridge.hipac_promote_transition.missing_endpoint_index");
                return finish_adopt(0);
            }
            const int source_index = static_cast<int>(std::distance(boxes_.begin(), source_it));
            const int target_index_box = static_cast<int>(std::distance(boxes_.begin(), target_it));

            std::unordered_map<int, int> first_sample_by_box;
            first_sample_by_box.reserve(boxes_.size() - boxes_before_direct_corridor + 8);
            for (std::size_t sample_index = 0; sample_index < sample_layers.size(); ++sample_index) {
                for (int box_index : sample_layers[sample_index]) {
                    if (box_index < static_cast<int>(boxes_before_direct_corridor) ||
                        box_index < 0 ||
                        box_index >= static_cast<int>(boxes_.size())) {
                        continue;
                    }
                    first_sample_by_box.emplace(box_index, static_cast<int>(sample_index));
                }
            }
            std::vector<int> candidate_indices;
            candidate_indices.reserve(first_sample_by_box.size());
            for (const auto& [box_index, sample_index] : first_sample_by_box) {
                (void)sample_index;
                if (box_index == source_index || box_index == target_index_box) {
                    continue;
                }
                const auto& box = boxes_[static_cast<std::size_t>(box_index)];
                if (box.safety_status != BoxSafetyStatus::CertifiedFree ||
                    box.strict_audit_required) {
                    diagnostics.add_counter(
                        "query_bridge.hipac_promote_transition.reject_non_certified");
                    continue;
                }
                candidate_indices.push_back(box_index);
            }
            std::sort(candidate_indices.begin(),
                      candidate_indices.end(),
                      [&](int lhs, int rhs) {
                const int lhs_sample = first_sample_by_box.count(lhs) > 0
                    ? first_sample_by_box[lhs]
                    : std::numeric_limits<int>::max();
                const int rhs_sample = first_sample_by_box.count(rhs) > 0
                    ? first_sample_by_box[rhs]
                    : std::numeric_limits<int>::max();
                if (lhs_sample != rhs_sample) {
                    return lhs_sample < rhs_sample;
                }
                return lhs < rhs;
            });
            if (static_cast<int>(candidate_indices.size()) < min_boxes) {
                diagnostics.add_counter("query_bridge.hipac_promote_transition.too_few_boxes");
                diagnostics.add_counter("query_bridge.hipac_promote_transition.candidate_boxes",
                                        static_cast<double>(candidate_indices.size()));
                return finish_adopt(0);
            }
            if (static_cast<int>(candidate_indices.size()) > max_boxes) {
                candidate_indices.resize(static_cast<std::size_t>(max_boxes));
            }
            diagnostics.add_counter("query_bridge.hipac_promote_transition.candidate_boxes",
                                    static_cast<double>(candidate_indices.size()));

            std::vector<int> local_indices;
            local_indices.reserve(candidate_indices.size() + 2);
            local_indices.push_back(source_index);
            local_indices.insert(local_indices.end(),
                                 candidate_indices.begin(),
                                 candidate_indices.end());
            local_indices.push_back(target_index_box);
            const int local_source = 0;
            const int local_target = static_cast<int>(local_indices.size()) - 1;
            std::vector<std::vector<int>> local_adj(local_indices.size());
            int exact_tests = 0;
            int exact_edges = 0;
            for (std::size_t lhs = 0; lhs < local_indices.size(); ++lhs) {
                for (std::size_t rhs = lhs + 1; rhs < local_indices.size(); ++rhs) {
                    ++exact_tests;
                    if (boxes_connected(boxes_[static_cast<std::size_t>(local_indices[lhs])],
                                        boxes_[static_cast<std::size_t>(local_indices[rhs])],
                                        config_.query.adjacency_tolerance)) {
                        local_adj[lhs].push_back(static_cast<int>(rhs));
                        local_adj[rhs].push_back(static_cast<int>(lhs));
                        ++exact_edges;
                    }
                }
            }
            diagnostics.add_counter("query_bridge.hipac_promote_transition.local_adj_tests",
                                    static_cast<double>(exact_tests));
            diagnostics.add_counter("query_bridge.hipac_promote_transition.local_adj_edges",
                                    static_cast<double>(exact_edges));
            auto shortest_local_path = [&](int source_node, int target_node) {
                std::vector<int> parent(local_indices.size(), -1);
                std::queue<int> queue;
                parent[static_cast<std::size_t>(source_node)] = source_node;
                queue.push(source_node);
                while (!queue.empty()) {
                    const int current = queue.front();
                    queue.pop();
                    if (current == target_node) {
                        break;
                    }
                    for (int neighbor : local_adj[static_cast<std::size_t>(current)]) {
                        if (parent[static_cast<std::size_t>(neighbor)] >= 0) {
                            continue;
                        }
                        parent[static_cast<std::size_t>(neighbor)] = current;
                        queue.push(neighbor);
                    }
                }
                std::vector<int> path;
                if (parent[static_cast<std::size_t>(target_node)] < 0) {
                    return path;
                }
                for (int current = target_node;
                     current != source_node;
                     current = parent[static_cast<std::size_t>(current)]) {
                    path.push_back(current);
                }
                path.push_back(source_node);
                std::reverse(path.begin(), path.end());
                return path;
            };
            auto promote_local_path = [&](const std::vector<int>& local_path,
                                          const char* mode) {
                if (local_path.size() < 3) {
                    diagnostics.add_counter("query_bridge.hipac_promote_transition.short_chain");
                    return 0;
                }
                const int source_local = local_path.front();
                const int target_local = local_path.back();
                const BoxNode& portal_source =
                    boxes_[static_cast<std::size_t>(local_indices[static_cast<std::size_t>(source_local)])];
                const BoxNode& portal_target =
                    boxes_[static_cast<std::size_t>(local_indices[static_cast<std::size_t>(target_local)])];
                std::vector<BoxNode> internal_boxes;
                internal_boxes.reserve(local_path.size());
                for (int local_node : local_path) {
                    if (local_node == source_local || local_node == target_local) {
                        continue;
                    }
                    internal_boxes.push_back(
                        boxes_[static_cast<std::size_t>(local_indices[static_cast<std::size_t>(local_node)])]);
                }
                if (static_cast<int>(internal_boxes.size()) < min_boxes) {
                    diagnostics.add_counter("query_bridge.hipac_promote_transition.short_chain");
                    return 0;
                }
                const int edge_id = append_portal_corridor_edge(segment_edges_,
                                                                portal_source,
                                                                portal_target,
                                                                std::move(internal_boxes),
                                                                -1,
                                                                config_.query.adjacency_tolerance,
                                                                bridge_edge_query_index);
                if (edge_id < 0) {
                    diagnostics.add_counter("query_bridge.hipac_promote_transition.edge_fail");
                    return 0;
                }
                sync_adaptive_partition_segment_edges(&last_build_,
                                                      "query_bridge.hipac_promote_transition");
                diagnostics.add_counter("query_bridge.hipac_promote_transition.added");
                diagnostics.add_counter(std::string("query_bridge.hipac_promote_transition.added_") +
                                            mode);
                diagnostics.add_counter("query_bridge.hipac_promote_transition.internal_boxes",
                                        static_cast<double>(local_path.size() - 2));
                set_query_bridge_task_value("hipac_promote_transition_internal_boxes",
                                            static_cast<double>(local_path.size() - 2));
                invalidate_query_cache();
                return 1;
            };
            std::vector<int> full_local_path = shortest_local_path(local_source, local_target);
            if (!full_local_path.empty()) {
                const int promoted = promote_local_path(full_local_path, "full");
                if (promoted > 0) {
                    return finish_adopt(promoted);
                }
            } else {
                diagnostics.add_counter("query_bridge.hipac_promote_transition.chain_fail");
            }

            std::vector<int> component_id(local_indices.size(), -1);
            int component_count = 0;
            for (int node = 1; node + 1 < static_cast<int>(local_indices.size()); ++node) {
                if (component_id[static_cast<std::size_t>(node)] >= 0) {
                    continue;
                }
                std::queue<int> component_queue;
                component_id[static_cast<std::size_t>(node)] = component_count;
                component_queue.push(node);
                while (!component_queue.empty()) {
                    const int current = component_queue.front();
                    component_queue.pop();
                    for (int neighbor : local_adj[static_cast<std::size_t>(current)]) {
                        if (neighbor <= local_source || neighbor >= local_target ||
                            component_id[static_cast<std::size_t>(neighbor)] >= 0) {
                            continue;
                        }
                        component_id[static_cast<std::size_t>(neighbor)] = component_count;
                        component_queue.push(neighbor);
                    }
                }
                ++component_count;
            }
            diagnostics.add_counter("query_bridge.hipac_promote_transition.slice_components",
                                    static_cast<double>(component_count));
            std::vector<std::vector<int>> nodes_by_component(static_cast<std::size_t>(component_count));
            for (int node = 1; node + 1 < static_cast<int>(local_indices.size()); ++node) {
                const int component = component_id[static_cast<std::size_t>(node)];
                if (component >= 0) {
                    nodes_by_component[static_cast<std::size_t>(component)].push_back(node);
                }
            }
            struct SliceCandidate {
                int first = -1;
                int last = -1;
                int count = 0;
                int span = 0;
            };
            std::vector<SliceCandidate> slices;
            slices.reserve(nodes_by_component.size());
            auto sample_rank = [&](int local_node) {
                const int box_index =
                    local_indices[static_cast<std::size_t>(local_node)];
                const auto it = first_sample_by_box.find(box_index);
                return it == first_sample_by_box.end()
                    ? std::numeric_limits<int>::max()
                    : it->second;
            };
            for (auto& nodes : nodes_by_component) {
                if (static_cast<int>(nodes.size()) < min_boxes + 2) {
                    continue;
                }
                std::sort(nodes.begin(), nodes.end(), [&](int lhs, int rhs) {
                    const int lhs_rank = sample_rank(lhs);
                    const int rhs_rank = sample_rank(rhs);
                    if (lhs_rank != rhs_rank) {
                        return lhs_rank < rhs_rank;
                    }
                    return lhs < rhs;
                });
                SliceCandidate slice;
                slice.first = nodes.front();
                slice.last = nodes.back();
                slice.count = static_cast<int>(nodes.size());
                slice.span = std::max(0, sample_rank(slice.last) - sample_rank(slice.first));
                slices.push_back(slice);
            }
            std::sort(slices.begin(), slices.end(), [](const SliceCandidate& lhs,
                                                       const SliceCandidate& rhs) {
                if (lhs.count != rhs.count) {
                    return lhs.count > rhs.count;
                }
                if (lhs.span != rhs.span) {
                    return lhs.span > rhs.span;
                }
                return lhs.first < rhs.first;
            });
            for (const auto& slice : slices) {
                std::vector<int> slice_path = shortest_local_path(slice.first, slice.last);
                if (slice_path.empty()) {
                    diagnostics.add_counter("query_bridge.hipac_promote_transition.slice_chain_fail");
                    continue;
                }
                const int promoted = promote_local_path(slice_path, "slice");
                if (promoted > 0) {
                    return finish_adopt(promoted);
                }
            }
            diagnostics.add_counter("query_bridge.hipac_promote_transition.failures");
            return finish_adopt(0);
        };
        auto commit_result = [&](FindFreeBoxResult result,
                                 const Eigen::VectorXd& seed,
                                 int transition_hint) -> int {
            const auto commit_t0 = detailed_direct_timing ? Clock::now() : Clock::time_point{};
            auto finish = [&](int value) {
                if (detailed_direct_timing) {
                    commit_total_ms +=
                        std::chrono::duration<double, std::milli>(Clock::now() - commit_t0).count();
                    commit_calls += 1;
                }
                return value;
            };
            if (!result.found ||
                !intervals_contain_point_local(result.intervals,
                                               seed,
                                               config_.query.adjacency_tolerance)) {
                return finish(-1);
            }
            const int duplicate_index = find_duplicate_box_index(result.node,
                                                                 result.intervals);
            if (duplicate_index >= 0) {
                const int covered_count = assimilate_box(duplicate_index, transition_hint);
                if (covered_count == 0) {
                    repair_milestones.push_back(
                        {query_bridge_seed_path_param(samples, seed, transition_hint), seed, duplicate_index});
                }
                return finish(duplicate_index);
            }
            const auto dynamic_policy_t0 = detailed_direct_timing ? Clock::now() : Clock::time_point{};
            if (!allow_dynamic_commit(*oracle_, result, config_.connector.pave.commit_policy)) {
                if (detailed_direct_timing) {
                    commit_dynamic_policy_ms +=
                        std::chrono::duration<double, std::milli>(Clock::now() - dynamic_policy_t0).count();
                }
                return finish(-1);
            }
            if (detailed_direct_timing) {
                commit_dynamic_policy_ms +=
                    std::chrono::duration<double, std::milli>(Clock::now() - dynamic_policy_t0).count();
            }
            BoxNode box;
            box.id = next_id++;
            box.joint_intervals = result.intervals;
            box.seed_config = seed;
            box.tree_id = result.node;
            box.parent_box_id = -1;
            box.root_id = box.id;
            box.safety_status = result.validation_detail.safety_status;
            box.strict_audit_required = result.validation_detail.strict_audit_required;
            box.compute_volume();
            if (box.tree_id != kInvalidOracleNodeId) {
                oracle_->reserve_node(box.tree_id, box.id);
            }
            const int box_index = static_cast<int>(boxes_.size());
            boxes_.push_back(box);
            raw_boxes_.push_back(box);
            if (result.node != kInvalidOracleNodeId) {
                node_to_box_index.emplace(result.node, box_index);
            }
            if (use_partition_cover_index) {
                corridor_new_box_indices.push_back(box_index);
                append_direct_partition_batch(false);
            } else {
                direct_box_index.add_box(boxes_.back(),
                                         box_index,
                                         config_.query.adjacency_tolerance);
            }
            if (use_partition_neighbor_candidates) {
                box_id_to_index[box.id] = box_index;
            }
            dsu.add();
            const int covered_count = assimilate_box(box_index, transition_hint);
            if (covered_count == 0) {
                repair_milestones.push_back(
                    {query_bridge_seed_path_param(samples, seed, transition_hint), seed, box_index});
            }
            return finish(box_index);
        };
        auto current_boxes_cover_point = [&](const Eigen::VectorXd& point) {
            const auto timing_t0 = detailed_direct_timing ? Clock::now() : Clock::time_point{};
            auto finish = [&](bool value) {
                if (detailed_direct_timing) {
                    current_cover_ms +=
                        std::chrono::duration<double, std::milli>(Clock::now() - timing_t0).count();
                    current_cover_calls += 1;
                }
                return value;
            };
            if (use_partition_cover_index) {
                const auto partition_cover_t0 =
                    detailed_direct_timing ? Clock::now() : Clock::time_point{};
                const bool partition_covered =
                    !adaptive_partition_->covering_box_ids(point,
                                                           config_.query.adjacency_tolerance).empty();
                if (detailed_direct_timing) {
                    current_cover_partition_ms +=
                        std::chrono::duration<double, std::milli>(Clock::now() -
                                                                  partition_cover_t0).count();
                }
                if (partition_covered) {
                    return finish(true);
                }
            }
            if (use_partition_cover_index) {
                const auto corridor_scan_t0 =
                    detailed_direct_timing ? Clock::now() : Clock::time_point{};
                for (int box_index : corridor_new_box_indices) {
                    if (box_index >= 0 &&
                        box_index < static_cast<int>(boxes_.size()) &&
                        intervals_contain_point_local(boxes_[static_cast<std::size_t>(box_index)].joint_intervals,
                                                       point,
                                                       config_.query.adjacency_tolerance)) {
                        if (detailed_direct_timing) {
                            current_cover_corridor_scan_ms +=
                                std::chrono::duration<double, std::milli>(Clock::now() -
                                                                          corridor_scan_t0).count();
                        }
                        return finish(true);
                    }
                }
                if (detailed_direct_timing) {
                    current_cover_corridor_scan_ms +=
                        std::chrono::duration<double, std::milli>(Clock::now() -
                                                                  corridor_scan_t0).count();
                }
                return finish(false);
            }
            const auto direct_index_t0 = detailed_direct_timing ? Clock::now() : Clock::time_point{};
            const bool covered_by_direct_index =
                direct_box_index.covering_box(boxes_,
                                              point,
                                              config_.query.adjacency_tolerance) >= 0;
            if (detailed_direct_timing) {
                current_cover_direct_index_ms +=
                    std::chrono::duration<double, std::milli>(Clock::now() - direct_index_t0).count();
            }
            return finish(covered_by_direct_index);
        };
        FindFreeBoxOptions direct_options = config_.connector.pave.find_free_box;
        direct_options.max_depth = query_bridge_ffb_depth;
        if (config_.query_bridge_ffb_start_depth >= 0) {
            direct_options.start_depth = config_.query_bridge_ffb_start_depth;
            direct_options.skip_to_depth = config_.query_bridge_ffb_start_depth;
        }
        direct_options.reject_seed_collision = false;
        direct_options.skip_existing_cover_check = true;
        direct_options.materialize_result_node = false;
        direct_options.record_diagnostics =
            env_int_or_default("RBF_QUERY_BRIDGE_FFB_DIAGNOSTICS", 0) != 0;
        const std::vector<Interval> direct_planning_domain =
            oracle_ ? oracle_->planning_intervals() : std::vector<Interval>{};
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_ffb_diagnostics_enabled",
            direct_options.record_diagnostics ? 1.0 : 0.0);
        int direct_calls = 0;
        int direct_added = 0;
        double direct_ffb_ms = 0.0;
        double repair_ffb_ms = 0.0;
        double adaptive_repair_ffb_ms = 0.0;
        double lateral_repair_ffb_ms = 0.0;
        double residual_segment_audit_ms = 0.0;
        const QueryBridgeDirectFfbTaskRuntimeOptions direct_task_options =
            query_bridge_direct_ffb_task_runtime_options(samples.size());
        const auto direct_task_build_t0 =
            detailed_direct_timing ? Clock::now() : Clock::time_point{};
        const QueryBridgeDirectFfbTaskBuildResult direct_task_build =
            query_bridge_build_direct_ffb_tasks(
                samples,
                covered,
                direct_task_options.build);
        const std::vector<QueryBridgeDirectFfbTask>& direct_tasks = direct_task_build.tasks;
        const int uncovered_gap_groups = direct_task_build.uncovered_gap_groups;
        if (detailed_direct_timing) {
            direct_task_build_ms =
                std::chrono::duration<double, std::milli>(Clock::now() -
                                                          direct_task_build_t0).count();
        }
        context.diagnostics().set_value("query_bridge.direct_corridor_direct_grouped_seeds",
                                        direct_task_options.build.grouped_direct_seeds ? 1.0 : 0.0);
        context.diagnostics().set_value("query_bridge.direct_corridor_coverage_order_direct_tasks",
                                        direct_task_options.coverage_order_direct_tasks ? 1.0 : 0.0);
        context.diagnostics().set_value("query_bridge.direct_corridor_center_out_direct_tasks",
                                        direct_task_options.build.center_out_direct_tasks ? 1.0 : 0.0);
        context.diagnostics().set_value("query_bridge.direct_corridor_ffb_start_depth",
                                        static_cast<double>(std::max(direct_options.start_depth,
                                                                    direct_options.skip_to_depth)));
        context.diagnostics().set_value("query_bridge.direct_corridor_uncovered_gap_groups",
                                        static_cast<double>(uncovered_gap_groups));
        context.diagnostics().set_value("query_bridge.direct_corridor_direct_max_seeds_per_gap",
                                        static_cast<double>(direct_task_options.build.max_group_seeds));
        context.diagnostics().set_value("query_bridge.direct_corridor_direct_tasks",
                                        static_cast<double>(direct_tasks.size()));
        const auto direct_loop_t0 =
            detailed_direct_timing ? Clock::now() : Clock::time_point{};
        for (const auto& task : direct_tasks) {
            if (task.sample_index < covered.size() && covered[task.sample_index]) {
                context.diagnostics().add_counter(
                    "query_bridge.direct_corridor_direct_skip_covered");
                continue;
            }
            const auto direct_ffb_t0 = Clock::now();
            const FindFreeBoxResult result = find_free_box_in_domain(
                task.seed,
                direct_planning_domain,
                context,
                direct_options);
            direct_ffb_ms +=
                std::chrono::duration<double, std::milli>(Clock::now() - direct_ffb_t0).count();
            direct_calls += 1;
            const std::size_t before_boxes = boxes_.size();
            const int box_index = commit_result(std::move(result),
                                                task.seed,
                                                task.transition_hint);
            if (box_index >= 0 && boxes_.size() > before_boxes) {
                direct_added += 1;
            }
        }
        if (detailed_direct_timing) {
            direct_loop_ms =
                std::chrono::duration<double, std::milli>(Clock::now() -
                                                          direct_loop_t0).count();
        }
        const QueryBridgeRepairSubdivisionOptions repair_subdivision_options =
            query_bridge_repair_subdivision_options(query_index);
        const int subdivisions = repair_subdivision_options.subdivisions;
        const std::vector<double>& fractions = repair_subdivision_options.fractions;
        int repair_calls = 0;
        int repair_added = 0;
        const auto initial_bad = bad_transitions();
        const auto repair_loop_t0 =
            detailed_direct_timing ? Clock::now() : Clock::time_point{};
        if (subdivisions > 1) {
            for (int transition : initial_bad) {
                if (transition_connected(transition)) {
                    continue;
                }
                const Eigen::VectorXd& a = samples[static_cast<std::size_t>(transition)];
                const Eigen::VectorXd& b = samples[static_cast<std::size_t>(transition + 1)];
                for (double u : fractions) {
                    if (transition_connected(transition)) {
                        break;
                    }
                    const Eigen::VectorXd seed = (1.0 - u) * a + u * b;
                    if (current_boxes_cover_point(seed)) {
                        context.diagnostics().add_counter(
                            "query_bridge.direct_corridor_repair_skip_covered");
                        continue;
                    }
                    const auto repair_ffb_t0 = Clock::now();
                    const FindFreeBoxResult result = find_free_box_in_domain(
                        seed,
                        direct_planning_domain,
                        context,
                        direct_options);
                    repair_ffb_ms +=
                        std::chrono::duration<double, std::milli>(Clock::now() -
                                                                  repair_ffb_t0).count();
                    repair_calls += 1;
                    const std::size_t before_boxes = boxes_.size();
                    const int box_index = commit_result(std::move(result), seed, transition);
                    if (box_index >= 0) {
                        if (std::find(repair_indices.begin(), repair_indices.end(), box_index) ==
                            repair_indices.end()) {
                            repair_indices.push_back(box_index);
                        }
                        if (boxes_.size() > before_boxes) {
                            repair_added += 1;
                        }
                        if (transition_connected(transition)) {
                            break;
                        }
                    }
                }
            }
        }
        if (detailed_direct_timing) {
            repair_loop_ms =
                std::chrono::duration<double, std::milli>(Clock::now() -
                                                          repair_loop_t0).count();
        }
        std::vector<int> final_bad = bad_transitions();
        auto bad_transition_fraction = [&](const std::vector<int>& transitions) {
            return query_bridge_transition_fraction(samples,
                                                    transitions,
                                                    audited_bridge_length,
                                                    query_bridge_waypoint_length(samples));
        };
        const QueryBridgeAdaptiveRepairOptions adaptive_repair_options =
            query_bridge_adaptive_repair_options(query_index,
                                                 subdivisions,
                                                 audit_step,
                                                 sample_step);
        const int adaptive_repair_priority_mode = adaptive_repair_options.priority_mode;
        auto order_adaptive_repair_transitions =
            [&](const std::vector<int>& transitions) {
                return query_bridge_order_transitions_by_gap_length(samples,
                                                                    transitions,
                                                                    adaptive_repair_priority_mode);
            };
        int adaptive_repair_calls = 0;
        int adaptive_repair_added = 0;
        int adaptive_repair_max_subdivisions_used = subdivisions;
        const bool adaptive_step_repair = adaptive_repair_options.enabled;
        const double adaptive_target_segment_fraction =
            adaptive_repair_options.target_segment_fraction;
        const double adaptive_initial_bad_fraction =
            bad_transition_fraction(final_bad);
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_adaptive_repair_priority",
            static_cast<double>(adaptive_repair_priority_mode));
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_adaptive_repair_target_segment_fraction",
            adaptive_target_segment_fraction);
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_adaptive_initial_bad_fraction",
            adaptive_initial_bad_fraction);
        if (adaptive_step_repair && !final_bad.empty()) {
            const auto adaptive_loop_t0 =
                detailed_direct_timing ? Clock::now() : Clock::time_point{};
            const int adaptive_max_subdivisions = adaptive_repair_options.max_subdivisions;
            const double adaptive_fine_step = adaptive_repair_options.fine_step;
            const int adaptive_max_calls = adaptive_repair_options.max_calls;
            std::vector<int> ordered_final_bad =
                order_adaptive_repair_transitions(final_bad);
            for (int transition : ordered_final_bad) {
                if (adaptive_repair_calls >= adaptive_max_calls) {
                    break;
                }
                if (adaptive_target_segment_fraction > 0.0 &&
                    bad_transition_fraction(final_bad) <= adaptive_target_segment_fraction) {
                    context.diagnostics().add_counter(
                        "query_bridge.direct_corridor_adaptive_repair_target_stops");
                    break;
                }
                if (transition_connected(transition) ||
                    transition < 0 ||
                    transition + 1 >= static_cast<int>(samples.size())) {
                    continue;
                }
                const Eigen::VectorXd& a = samples[static_cast<std::size_t>(transition)];
                const Eigen::VectorXd& b = samples[static_cast<std::size_t>(transition + 1)];
                const double gap_length = (b - a).norm();
                const int target_subdivisions = std::min(
                    adaptive_max_subdivisions,
                    std::max(subdivisions + 1,
                             static_cast<int>(std::ceil(gap_length / adaptive_fine_step))));
                adaptive_repair_max_subdivisions_used =
                    std::max(adaptive_repair_max_subdivisions_used, target_subdivisions);
                const std::vector<double> adaptive_fractions =
                    query_bridge_center_ordered_fractions(target_subdivisions);
                for (double u : adaptive_fractions) {
                    if (adaptive_repair_calls >= adaptive_max_calls) {
                        break;
                    }
                    if (transition_connected(transition)) {
                        break;
                    }
                    const Eigen::VectorXd seed = (1.0 - u) * a + u * b;
                    if (current_boxes_cover_point(seed)) {
                        context.diagnostics().add_counter(
                            "query_bridge.direct_corridor_adaptive_repair_skip_covered");
                        continue;
                    }
                    const auto adaptive_ffb_t0 = Clock::now();
                    const FindFreeBoxResult result = find_free_box_in_domain(
                        seed,
                        direct_planning_domain,
                        context,
                        direct_options);
                    adaptive_repair_ffb_ms +=
                        std::chrono::duration<double, std::milli>(Clock::now() -
                                                                  adaptive_ffb_t0).count();
                    adaptive_repair_calls += 1;
                    const std::size_t before_boxes = boxes_.size();
                    const int box_index = commit_result(std::move(result), seed, transition);
                    if (box_index >= 0) {
                        if (std::find(repair_indices.begin(), repair_indices.end(), box_index) ==
                            repair_indices.end()) {
                            repair_indices.push_back(box_index);
                        }
                        if (boxes_.size() > before_boxes) {
                            adaptive_repair_added += 1;
                        }
                        if (adaptive_target_segment_fraction > 0.0) {
                            final_bad = bad_transitions();
                        }
                        if (transition_connected(transition)) {
                            break;
                        }
                    }
                }
            }
            final_bad = bad_transitions();
            if (detailed_direct_timing) {
                adaptive_loop_ms =
                    std::chrono::duration<double, std::milli>(Clock::now() -
                                                              adaptive_loop_t0).count();
            }
        }
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_adaptive_final_bad_fraction",
            bad_transition_fraction(final_bad));
        int lateral_repair_calls = 0;
        int lateral_repair_added = 0;
        const QueryBridgeLateralRepairOptions lateral_repair_options =
            query_bridge_lateral_repair_options(sample_step);
        const bool lateral_repair = lateral_repair_options.enabled;
        if (lateral_repair && !final_bad.empty()) {
            const auto lateral_loop_t0 =
                detailed_direct_timing ? Clock::now() : Clock::time_point{};
            const auto domain = oracle_->planning_intervals();
            for (int transition : final_bad) {
                if (lateral_repair_calls >= lateral_repair_options.max_calls) {
                    break;
                }
                if (transition_connected(transition) ||
                    transition < 0 ||
                    transition + 1 >= static_cast<int>(samples.size())) {
                    continue;
                }
                const Eigen::VectorXd& a = samples[static_cast<std::size_t>(transition)];
                const Eigen::VectorXd& b = samples[static_cast<std::size_t>(transition + 1)];
                const Eigen::VectorXd seed = 0.5 * (a + b);
                const Eigen::VectorXd direction = b - a;
                for (const Eigen::VectorXd& lateral_seed :
                     query_bridge_lateral_candidates(seed,
                                                     direction,
                                                     domain,
                                                     lateral_repair_options.dims,
                                                     lateral_repair_options.rounds,
                                                     lateral_repair_options.offset)) {
                    if (lateral_repair_calls >= lateral_repair_options.max_calls) {
                        break;
                    }
                    if (transition_connected(transition)) {
                        break;
                    }
                    if (current_boxes_cover_point(lateral_seed)) {
                        context.diagnostics().add_counter(
                            "query_bridge.direct_corridor_lateral_repair_skip_covered");
                        continue;
                    }
                    const auto lateral_ffb_t0 = Clock::now();
                    const FindFreeBoxResult result = find_free_box_in_domain(
                        lateral_seed,
                        direct_planning_domain,
                        context,
                        direct_options);
                    lateral_repair_ffb_ms +=
                        std::chrono::duration<double, std::milli>(Clock::now() -
                                                                  lateral_ffb_t0).count();
                    lateral_repair_calls += 1;
                    const std::size_t before_boxes = boxes_.size();
                    const int box_index = commit_result(std::move(result),
                                                        lateral_seed,
                                                        transition);
                    if (box_index >= 0) {
                        if (std::find(repair_indices.begin(), repair_indices.end(), box_index) ==
                            repair_indices.end()) {
                            repair_indices.push_back(box_index);
                        }
                        if (boxes_.size() > before_boxes) {
                            lateral_repair_added += 1;
                        }
                        if (transition_connected(transition)) {
                            break;
                        }
                    }
                }
            }
            final_bad = bad_transitions();
            if (detailed_direct_timing) {
                lateral_loop_ms =
                    std::chrono::duration<double, std::milli>(Clock::now() -
                                                              lateral_loop_t0).count();
            }
        }
        int local_segment_edges_added = 0;
        int local_segment_gap_samples_max = 0;
        if (!final_bad.empty() &&
            allow_residual_segments &&
            config_.connector.segment_edges_enabled &&
            config_.connector.rrt_segment_edges) {
            const auto residual_segment_loop_t0 =
                detailed_direct_timing ? Clock::now() : Clock::time_point{};
            const bool group_residual_gaps =
                env_int_or_default("RBF_QUERY_BRIDGE_GROUP_RESIDUAL_GAPS", 0) != 0;
            const std::vector<std::pair<int, int>> gap_groups =
                query_bridge_group_residual_gap_transitions(final_bad,
                                                            sample_layers.size(),
                                                            group_residual_gaps);
            context.diagnostics().set_value(
                "query_bridge.direct_corridor_segment_gap_groups",
                static_cast<double>(gap_groups.size()));
            std::vector<std::pair<int, int>> pending_gap_groups;
            pending_gap_groups.reserve(gap_groups.size());
            for (auto it = gap_groups.rbegin(); it != gap_groups.rend(); ++it) {
                pending_gap_groups.push_back(*it);
            }
            const bool residual_milestone_segments =
                env_int_or_default("RBF_QUERY_BRIDGE_RESIDUAL_MILESTONE_SEGMENTS", 0) != 0;
            context.diagnostics().set_value(
                "query_bridge.direct_corridor_residual_milestone_segments",
                residual_milestone_segments ? 1.0 : 0.0);
            context.diagnostics().set_value(
                "query_bridge.direct_corridor_repair_milestones",
                static_cast<double>(repair_milestones.size()));
            auto insert_residual_segment = [&](int lhs_index,
                                               int rhs_index,
                                               Eigen::VectorXd lhs_point,
                                               Eigen::VectorXd rhs_point,
                                               int sample_gap) {
                if (lhs_index < 0 || rhs_index < 0 ||
                    lhs_index >= static_cast<int>(boxes_.size()) ||
                    rhs_index >= static_cast<int>(boxes_.size())) {
                    return false;
                }
                if (dsu.find(lhs_index) == dsu.find(rhs_index)) {
                    return false;
                }
                std::vector<Eigen::VectorXd> gap_path{std::move(lhs_point), std::move(rhs_point)};
                const auto segment_audit_t0 = Clock::now();
                const PathAuditCheck gap_audit =
                    audit_waypoint_path(gap_path,
                                        checker,
                                        config_.query.audit_resolution,
                                        config_.query.audit_segment_step);
                residual_segment_audit_ms +=
                    std::chrono::duration<double, std::milli>(Clock::now() -
                                                              segment_audit_t0).count();
                if (!gap_audit.passed) {
                    context.diagnostics().add_counter(
                        "query_bridge.direct_corridor_segment_audit_rejects");
                    return false;
                }
                const auto segment_insert_t0 =
                    detailed_direct_timing ? Clock::now() : Clock::time_point{};
                const int edge_id = add_segment_edge_partition_first(
                    boxes_[static_cast<std::size_t>(lhs_index)].id,
                    boxes_[static_cast<std::size_t>(rhs_index)].id,
                    std::move(gap_path),
                    SegmentEdgeType::QueryBridge,
                    bridge_rrt.segment_resolution,
                    SegmentEdgeValidation::CollisionChecked,
                    true,
                    bridge_edge_query_index);
                if (detailed_direct_timing) {
                    segment_insert_ms +=
                        std::chrono::duration<double, std::milli>(Clock::now() - segment_insert_t0).count();
                    segment_insert_calls += 1;
                }
                if (edge_id >= 0) {
                    local_segment_edges_added += 1;
                    local_segment_gap_samples_max =
                        std::max(local_segment_gap_samples_max, sample_gap);
                    dsu.unite(lhs_index, rhs_index);
                    return true;
                }
                return false;
            };
            if (residual_milestone_segments) {
                const std::vector<QueryBridgeResidualMilestone> compact =
                    query_bridge_compact_residual_milestones(samples,
                                                             sample_layers,
                                                             repair_milestones,
                                                             static_cast<int>(boxes_.size()),
                                                             dsu);
                context.diagnostics().set_value(
                    "query_bridge.direct_corridor_residual_milestones",
                    static_cast<double>(compact.size()));
                for (std::size_t index = 0; index + 1 < compact.size(); ++index) {
                    const auto& lhs = compact[index];
                    const auto& rhs = compact[index + 1];
                    if (rhs.param <= lhs.param + 1e-9) {
                        continue;
                    }
                    const int sample_gap = static_cast<int>(
                        std::ceil(std::max(0.0, rhs.param - lhs.param)));
                    insert_residual_segment(lhs.box_index,
                                            rhs.box_index,
                                            lhs.point,
                                            rhs.point,
                                            sample_gap);
                }
            } else {
                while (!pending_gap_groups.empty()) {
                    const auto gap_group = pending_gap_groups.back();
                    pending_gap_groups.pop_back();
                    const int lhs_sample =
                        query_bridge_nearest_nonempty_layer(sample_layers, gap_group.first, -1);
                    const int rhs_sample =
                        query_bridge_nearest_nonempty_layer(sample_layers, gap_group.second + 1, 1);
                    if (lhs_sample < 0 || rhs_sample < 0 || lhs_sample >= rhs_sample) {
                        continue;
                    }
                    const auto& lhs_layer = sample_layers[static_cast<std::size_t>(lhs_sample)];
                    const auto& rhs_layer = sample_layers[static_cast<std::size_t>(rhs_sample)];
                    if (lhs_layer.empty() || rhs_layer.empty()) {
                        continue;
                    }
                    const int lhs_index = lhs_layer.front();
                    const int rhs_index = rhs_layer.front();
                    const auto lhs_point = samples[static_cast<std::size_t>(lhs_sample)];
                    const auto rhs_point = samples[static_cast<std::size_t>(rhs_sample)];
                    const bool inserted = insert_residual_segment(lhs_index,
                                                                  rhs_index,
                                                                  lhs_point,
                                                                  rhs_point,
                                                                  rhs_sample - lhs_sample);
                    if (!inserted) {
                        if (group_residual_gaps && gap_group.first < gap_group.second) {
                            const int mid = (gap_group.first + gap_group.second) / 2;
                            pending_gap_groups.emplace_back(mid + 1, gap_group.second);
                            pending_gap_groups.emplace_back(gap_group.first, mid);
                            context.diagnostics().add_counter(
                                "query_bridge.direct_corridor_segment_group_splits");
                        }
                        continue;
                    }
                }
            }
            if (detailed_direct_timing) {
                residual_segment_loop_ms =
                    std::chrono::duration<double, std::milli>(Clock::now() -
                                                              residual_segment_loop_t0).count();
            }
        }
        const double direct_corridor_elapsed_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        context.diagnostics().set_value("query_bridge.direct_corridor_ms",
                                        direct_corridor_elapsed_ms);
        context.diagnostics().add_counter("query_bridge.direct_corridor_ms_total",
                                          direct_corridor_elapsed_ms);
        context.diagnostics().set_value("query_bridge.direct_corridor_samples",
                                        static_cast<double>(samples.size()));
        context.diagnostics().add_counter("query_bridge.direct_corridor_samples_total",
                                          static_cast<double>(samples.size()));
        context.diagnostics().set_value("query_bridge.direct_corridor_ffb_calls",
                                        static_cast<double>(direct_calls));
        context.diagnostics().add_counter("query_bridge.direct_corridor_ffb_calls_total",
                                          static_cast<double>(direct_calls));
        context.diagnostics().set_value("query_bridge.direct_corridor_direct_ffb_ms",
                                        direct_ffb_ms);
        context.diagnostics().set_value("query_bridge.direct_corridor_repair_ffb_ms",
                                        repair_ffb_ms);
        context.diagnostics().set_value("query_bridge.direct_corridor_adaptive_repair_ffb_ms",
                                        adaptive_repair_ffb_ms);
        context.diagnostics().set_value("query_bridge.direct_corridor_lateral_repair_ffb_ms",
                                        lateral_repair_ffb_ms);
        context.diagnostics().set_value("query_bridge.direct_corridor_segment_audit_ms",
                                        residual_segment_audit_ms);
        const int all_ffb_calls =
            direct_calls + repair_calls + adaptive_repair_calls + lateral_repair_calls;
        context.diagnostics().set_value("query_bridge.direct_corridor_all_ffb_calls",
                                        static_cast<double>(all_ffb_calls));
        context.diagnostics().add_counter("query_bridge.direct_corridor_all_ffb_calls_total",
                                          static_cast<double>(all_ffb_calls));
        context.diagnostics().set_value("query_bridge.direct_corridor_added",
                                        static_cast<double>(direct_added));
        context.diagnostics().add_counter("query_bridge.direct_corridor_added_total",
                                          static_cast<double>(direct_added));
        context.diagnostics().set_value("query_bridge.direct_corridor_repair_calls",
                                        static_cast<double>(repair_calls));
        context.diagnostics().add_counter("query_bridge.direct_corridor_repair_calls_total",
                                          static_cast<double>(repair_calls));
        context.diagnostics().set_value("query_bridge.direct_corridor_repair_added",
                                        static_cast<double>(repair_added));
        context.diagnostics().add_counter("query_bridge.direct_corridor_repair_added_total",
                                          static_cast<double>(repair_added));
        context.diagnostics().set_value("query_bridge.direct_corridor_adaptive_repair_calls",
                                        static_cast<double>(adaptive_repair_calls));
        context.diagnostics().add_counter(
            "query_bridge.direct_corridor_adaptive_repair_calls_total",
            static_cast<double>(adaptive_repair_calls));
        context.diagnostics().set_value("query_bridge.direct_corridor_adaptive_repair_added",
                                        static_cast<double>(adaptive_repair_added));
        context.diagnostics().add_counter(
            "query_bridge.direct_corridor_adaptive_repair_added_total",
            static_cast<double>(adaptive_repair_added));
        context.diagnostics().set_value("query_bridge.direct_corridor_lateral_repair_enabled",
                                        lateral_repair ? 1.0 : 0.0);
        context.diagnostics().set_value("query_bridge.direct_corridor_lateral_repair_calls",
                                        static_cast<double>(lateral_repair_calls));
        context.diagnostics().add_counter(
            "query_bridge.direct_corridor_lateral_repair_calls_total",
            static_cast<double>(lateral_repair_calls));
        context.diagnostics().set_value("query_bridge.direct_corridor_lateral_repair_added",
                                        static_cast<double>(lateral_repair_added));
        context.diagnostics().add_counter(
            "query_bridge.direct_corridor_lateral_repair_added_total",
            static_cast<double>(lateral_repair_added));
        context.diagnostics().set_value("query_bridge.direct_corridor_adaptive_repair_max_subdivisions",
                                        static_cast<double>(adaptive_repair_max_subdivisions_used));
        context.diagnostics().set_value("query_bridge.direct_corridor_repair_subdivisions",
                                        static_cast<double>(subdivisions));
        context.diagnostics().set_value("query_bridge.direct_corridor_bad_initial",
                                        static_cast<double>(initial_bad.size()));
        context.diagnostics().add_counter("query_bridge.direct_corridor_bad_initial_total",
                                          static_cast<double>(initial_bad.size()));
        context.diagnostics().set_value("query_bridge.direct_corridor_bad_final",
                                        static_cast<double>(final_bad.size()));
        context.diagnostics().add_counter("query_bridge.direct_corridor_bad_final_total",
                                          static_cast<double>(final_bad.size()));
        context.diagnostics().set_value("query_bridge.direct_corridor_segment_edges",
                                        static_cast<double>(local_segment_edges_added));
        context.diagnostics().add_counter("query_bridge.direct_corridor_segment_edges_total",
                                          static_cast<double>(local_segment_edges_added));
        context.diagnostics().set_value("query_bridge.direct_corridor_segment_gap_samples_max",
                                        static_cast<double>(local_segment_gap_samples_max));
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_assimilate_coverage_span_max",
            static_cast<double>(assimilate_coverage_span_max));
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_assimilate_coverage_span_mean",
            assimilate_coverage_boxes > 0
                ? assimilate_coverage_span_sum / static_cast<double>(assimilate_coverage_boxes)
                : 0.0);
        if (detailed_direct_timing) {
            context.diagnostics().add_counter("query_bridge.direct_corridor_transition_connected_ms",
                                              transition_connected_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_transition_connected_calls",
                                              static_cast<double>(transition_connected_calls));
            context.diagnostics().add_counter("query_bridge.direct_corridor_bad_transitions_ms",
                                              bad_transitions_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_bad_transitions_calls",
                                              static_cast<double>(bad_transitions_calls));
            context.diagnostics().add_counter("query_bridge.direct_corridor_current_cover_ms",
                                              current_cover_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_current_cover_calls",
                                              static_cast<double>(current_cover_calls));
            context.diagnostics().add_counter("query_bridge.direct_corridor_current_cover_partition_ms",
                                              current_cover_partition_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_current_cover_corridor_scan_ms",
                                              current_cover_corridor_scan_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_current_cover_direct_index_ms",
                                              current_cover_direct_index_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_duplicate_lookup_ms",
                                              duplicate_lookup_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_duplicate_lookup_calls",
                                              static_cast<double>(duplicate_lookup_calls));
            context.diagnostics().add_counter("query_bridge.direct_corridor_commit_total_ms",
                                              commit_total_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_commit_calls",
                                              static_cast<double>(commit_calls));
            context.diagnostics().add_counter("query_bridge.direct_corridor_commit_dynamic_policy_ms",
                                              commit_dynamic_policy_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_commit_partition_append_ms",
                                              commit_partition_append_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_partition_append_calls",
                                              static_cast<double>(direct_partition_append_calls));
            context.diagnostics().add_counter("query_bridge.direct_corridor_partition_append_boxes",
                                              static_cast<double>(direct_partition_append_boxes));
            context.diagnostics().add_counter("query_bridge.direct_corridor_assimilate_calls",
                                              static_cast<double>(assimilate_calls));
            context.diagnostics().add_counter("query_bridge.direct_corridor_assimilate_sample_scan_ms",
                                              assimilate_sample_scan_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_assimilate_local_hits",
                                              static_cast<double>(assimilate_local_hits));
            context.diagnostics().add_counter("query_bridge.direct_corridor_assimilate_full_scan_fallbacks",
                                              static_cast<double>(assimilate_full_scan_fallbacks));
            context.diagnostics().add_counter("query_bridge.direct_corridor_assimilate_local_sample_tests",
                                              static_cast<double>(assimilate_local_sample_tests));
            context.diagnostics().add_counter("query_bridge.direct_corridor_assimilate_candidate_build_ms",
                                              assimilate_candidate_build_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_assimilate_adjacency_ms",
                                              assimilate_adjacency_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_segment_insert_ms",
                                              segment_insert_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_segment_insert_calls",
                                              static_cast<double>(segment_insert_calls));
            context.diagnostics().add_counter("query_bridge.direct_corridor_direct_task_build_ms",
                                              direct_task_build_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_direct_loop_ms",
                                              direct_loop_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_repair_loop_ms",
                                              repair_loop_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_adaptive_loop_ms",
                                              adaptive_loop_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_lateral_loop_ms",
                                              lateral_loop_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_residual_segment_loop_ms",
                                              residual_segment_loop_ms);
            set_query_bridge_task_value("direct_corridor_transition_connected_ms",
                                        transition_connected_ms);
            set_query_bridge_task_value("direct_corridor_transition_connected_calls",
                                        static_cast<double>(transition_connected_calls));
            set_query_bridge_task_value("direct_corridor_bad_transitions_ms",
                                        bad_transitions_ms);
            set_query_bridge_task_value("direct_corridor_bad_transitions_calls",
                                        static_cast<double>(bad_transitions_calls));
            set_query_bridge_task_value("direct_corridor_current_cover_ms",
                                        current_cover_ms);
            set_query_bridge_task_value("direct_corridor_current_cover_calls",
                                        static_cast<double>(current_cover_calls));
            set_query_bridge_task_value("direct_corridor_current_cover_partition_ms",
                                        current_cover_partition_ms);
            set_query_bridge_task_value("direct_corridor_current_cover_corridor_scan_ms",
                                        current_cover_corridor_scan_ms);
            set_query_bridge_task_value("direct_corridor_current_cover_direct_index_ms",
                                        current_cover_direct_index_ms);
            set_query_bridge_task_value("direct_corridor_duplicate_lookup_ms",
                                        duplicate_lookup_ms);
            set_query_bridge_task_value("direct_corridor_duplicate_lookup_calls",
                                        static_cast<double>(duplicate_lookup_calls));
            set_query_bridge_task_value("direct_corridor_commit_total_ms",
                                        commit_total_ms);
            set_query_bridge_task_value("direct_corridor_commit_calls",
                                        static_cast<double>(commit_calls));
            set_query_bridge_task_value("direct_corridor_commit_dynamic_policy_ms",
                                        commit_dynamic_policy_ms);
            set_query_bridge_task_value("direct_corridor_commit_partition_append_ms",
                                        commit_partition_append_ms);
            set_query_bridge_task_value("direct_corridor_partition_append_calls",
                                        static_cast<double>(direct_partition_append_calls));
            set_query_bridge_task_value("direct_corridor_partition_append_boxes",
                                        static_cast<double>(direct_partition_append_boxes));
            set_query_bridge_task_value("direct_corridor_assimilate_calls",
                                        static_cast<double>(assimilate_calls));
            set_query_bridge_task_value("direct_corridor_assimilate_sample_scan_ms",
                                        assimilate_sample_scan_ms);
            set_query_bridge_task_value("direct_corridor_assimilate_local_hits",
                                        static_cast<double>(assimilate_local_hits));
            set_query_bridge_task_value("direct_corridor_assimilate_full_scan_fallbacks",
                                        static_cast<double>(assimilate_full_scan_fallbacks));
            set_query_bridge_task_value("direct_corridor_assimilate_local_sample_tests",
                                        static_cast<double>(assimilate_local_sample_tests));
            set_query_bridge_task_value("direct_corridor_assimilate_candidate_build_ms",
                                        assimilate_candidate_build_ms);
            set_query_bridge_task_value("direct_corridor_assimilate_adjacency_ms",
                                        assimilate_adjacency_ms);
            set_query_bridge_task_value("direct_corridor_segment_insert_ms",
                                        segment_insert_ms);
            set_query_bridge_task_value("direct_corridor_segment_insert_calls",
                                        static_cast<double>(segment_insert_calls));
            set_query_bridge_task_value("direct_corridor_direct_task_build_ms",
                                        direct_task_build_ms);
            set_query_bridge_task_value("direct_corridor_assimilate_coverage_span_max",
                                        static_cast<double>(assimilate_coverage_span_max));
            set_query_bridge_task_value(
                "direct_corridor_assimilate_coverage_span_mean",
                assimilate_coverage_boxes > 0
                    ? assimilate_coverage_span_sum / static_cast<double>(assimilate_coverage_boxes)
                    : 0.0);
            set_query_bridge_task_value("direct_corridor_direct_loop_ms",
                                        direct_loop_ms);
            set_query_bridge_task_value("direct_corridor_repair_loop_ms",
                                        repair_loop_ms);
            set_query_bridge_task_value("direct_corridor_adaptive_loop_ms",
                                        adaptive_loop_ms);
            set_query_bridge_task_value("direct_corridor_lateral_loop_ms",
                                        lateral_loop_ms);
            set_query_bridge_task_value("direct_corridor_residual_segment_loop_ms",
                                        residual_segment_loop_ms);
        }
        auto [source_box_id, target_box_id] = locate_query_boxes();
        const bool local_corridor_connected =
            final_bad.empty() && endpoint_layers_connected();
        context.diagnostics().set_value("query_bridge.direct_corridor_local_connected",
                                        local_corridor_connected ? 1.0 : 0.0);
        set_query_bridge_task_value("direct_corridor_ms",
                                    direct_corridor_elapsed_ms);
        set_query_bridge_task_value("direct_corridor_samples",
                                    static_cast<double>(samples.size()));
        set_query_bridge_task_value("direct_corridor_ffb_calls",
                                    static_cast<double>(direct_calls));
        set_query_bridge_task_value("direct_corridor_direct_ffb_ms",
                                    direct_ffb_ms);
        set_query_bridge_task_value("direct_corridor_repair_ffb_ms",
                                    repair_ffb_ms);
        set_query_bridge_task_value("direct_corridor_adaptive_repair_ffb_ms",
                                    adaptive_repair_ffb_ms);
        set_query_bridge_task_value("direct_corridor_lateral_repair_ffb_ms",
                                    lateral_repair_ffb_ms);
        set_query_bridge_task_value("direct_corridor_segment_audit_ms",
                                    residual_segment_audit_ms);
        set_query_bridge_task_value("direct_corridor_all_ffb_calls",
                                    static_cast<double>(all_ffb_calls));
        set_query_bridge_task_value("direct_corridor_added",
                                    static_cast<double>(direct_added));
        set_query_bridge_task_value("direct_corridor_repair_calls",
                                    static_cast<double>(repair_calls));
        set_query_bridge_task_value("direct_corridor_repair_added",
                                    static_cast<double>(repair_added));
        set_query_bridge_task_value("direct_corridor_adaptive_repair_calls",
                                    static_cast<double>(adaptive_repair_calls));
        set_query_bridge_task_value("direct_corridor_adaptive_repair_added",
                                    static_cast<double>(adaptive_repair_added));
        set_query_bridge_task_value("direct_corridor_lateral_repair_calls",
                                    static_cast<double>(lateral_repair_calls));
        set_query_bridge_task_value("direct_corridor_lateral_repair_added",
                                    static_cast<double>(lateral_repair_added));
        set_query_bridge_task_value("direct_corridor_bad_initial",
                                    static_cast<double>(initial_bad.size()));
        set_query_bridge_task_value("direct_corridor_bad_final",
                                    static_cast<double>(final_bad.size()));
        set_query_bridge_task_value("direct_corridor_segment_edges",
                                    static_cast<double>(local_segment_edges_added));
        set_query_bridge_task_value("direct_corridor_local_connected",
                                    local_corridor_connected ? 1.0 : 0.0);
        auto set_direct_corridor_ffb_diag = [&](const std::string& ffb_key,
                                                const std::string& suffix) {
            set_query_bridge_task_value(
                "direct_corridor_" + suffix,
                context.diagnostics().value("ffb." + ffb_key, 0.0));
        };
        set_direct_corridor_ffb_diag("find_calls", "ffb_find_calls");
        set_direct_corridor_ffb_diag("binary_requested", "ffb_binary_requested");
        set_direct_corridor_ffb_diag("virtual_sparse_binary_attempts",
                                     "ffb_virtual_sparse_binary_attempts");
        set_direct_corridor_ffb_diag("virtual_sparse_binary_successes",
                                     "ffb_virtual_sparse_binary_successes");
        set_direct_corridor_ffb_diag("virtual_sparse_binary_probes",
                                     "ffb_virtual_sparse_binary_probes");
        set_direct_corridor_ffb_diag("binary_materialized_fallback_calls",
                                     "ffb_binary_materialized_fallback_calls");
        set_direct_corridor_ffb_diag("binary_blocked_adaptive_depths",
                                     "ffb_binary_blocked_adaptive_depths");
        set_direct_corridor_ffb_diag("binary_virtual_unsupported",
                                     "ffb_binary_virtual_unsupported");
        set_direct_corridor_ffb_diag("linear_descent_calls",
                                     "ffb_linear_descent_calls");
        if (final_bad.empty() &&
            source_box_id >= 0 &&
            target_box_id >= 0 &&
            (local_corridor_connected ||
             box_only_path_connected_partition_first(source_box_id, target_box_id))) {
            try_adopt_certified_subchain(source_box_id,
                                         target_box_id,
                                         "box_connected");
            const int edge_id = add_segment_edge_partition_first(                                                 source_box_id,
                                                 target_box_id,
                                                 corridor_path,
                                                 SegmentEdgeType::BoxCorridor,
                                                 bridge_rrt.segment_resolution,
                                                 SegmentEdgeValidation::CollisionChecked,
                                                 false,
                                                 bridge_edge_query_index);
            if (edge_id >= 0) {
                invalidate_query_cache();
                return finish_direct_corridor(direct_added + repair_added + local_segment_edges_added + 1);
            }
            return finish_direct_corridor(direct_added + repair_added + local_segment_edges_added);
        }
        if (!final_bad.empty() &&
            allow_residual_segments &&
            local_segment_edges_added > 0 &&
            source_box_id >= 0 &&
            target_box_id >= 0) {
            refresh_direct_corridor_partition();
            const bool locally_overlay_connected =
                overlay_path_connected_partition_first(source_box_id, target_box_id);
            context.diagnostics().set_value(
                "query_bridge.direct_corridor_local_residual_overlay_connected",
                locally_overlay_connected ? 1.0 : 0.0);
            set_query_bridge_task_value("direct_corridor_local_residual_overlay_connected",
                                        locally_overlay_connected ? 1.0 : 0.0);
            if (locally_overlay_connected) {
                const bool add_full_residual_overlay_when_connected =
                    env_int_or_default(
                        "RBF_QUERY_BRIDGE_FULL_RESIDUAL_OVERLAY_WHEN_CONNECTED",
                        0) != 0;
                int full_edge_id = -1;
                if (add_full_residual_overlay_when_connected) {
                    const PathAuditCheck full_residual_audit =
                        audit_waypoint_path(corridor_path,
                                            checker,
                                            config_.query.audit_resolution,
                                            config_.query.audit_segment_step);
                    if (full_residual_audit.passed) {
                        full_edge_id = add_segment_edge_partition_first(
                            source_box_id,
                            target_box_id,
                            corridor_path,
                            SegmentEdgeType::QueryBridge,
                            bridge_rrt.segment_resolution,
                            SegmentEdgeValidation::CollisionChecked,
                            true,
                            bridge_edge_query_index);
                        if (full_edge_id >= 0) {
                            context.diagnostics().add_counter(
                                "query_bridge.direct_corridor_full_residual_edges");
                            context.diagnostics().add_counter(
                                "query_bridge.direct_corridor_full_residual_edges_with_local_overlay");
                            set_query_bridge_task_value("direct_corridor_full_residual_edge",
                                                        1.0);
                        }
                    } else {
                        context.diagnostics().add_counter(
                            "query_bridge.direct_corridor_full_residual_audit_rejects");
                    }
                }
                try_adopt_certified_subchain(source_box_id,
                                             target_box_id,
                                             "local_residual_overlay");
                invalidate_query_cache();
                return finish_direct_corridor(direct_added + repair_added + local_segment_edges_added +
                                              (full_edge_id >= 0 ? 1 : 0));
            }
            context.diagnostics().add_counter(
                "query_bridge.direct_corridor_full_residual_without_local_overlay");
            const PathAuditCheck full_residual_audit =
                audit_waypoint_path(corridor_path,
                                    checker,
                                    config_.query.audit_resolution,
                                    config_.query.audit_segment_step);
            if (!full_residual_audit.passed) {
                context.diagnostics().add_counter(
                    "query_bridge.direct_corridor_full_residual_audit_rejects");
                invalidate_query_cache();
                return finish_direct_corridor(direct_added + repair_added + local_segment_edges_added);
            }
            const int edge_id = add_segment_edge_partition_first(                                                 source_box_id,
                                                 target_box_id,
                                                 corridor_path,
                                                 SegmentEdgeType::QueryBridge,
                                                 bridge_rrt.segment_resolution,
                                                 SegmentEdgeValidation::CollisionChecked,
                                                 true,
                                                 bridge_edge_query_index);
            if (edge_id >= 0) {
                context.diagnostics().add_counter(
                    "query_bridge.direct_corridor_full_residual_edges");
                if (!locally_overlay_connected) {
                    context.diagnostics().add_counter(
                        "query_bridge.direct_corridor_full_residual_edges_without_local_overlay");
                }
                set_query_bridge_task_value("direct_corridor_full_residual_edge",
                                            1.0);
            }
            try_adopt_certified_subchain(source_box_id,
                                         target_box_id,
                                         "full_residual");
            invalidate_query_cache();
            return finish_direct_corridor(direct_added + repair_added + local_segment_edges_added +
                                          (edge_id >= 0 ? 1 : 0));
        }
        return finish_direct_corridor(0);
    };
    if (dense_box_corridor_candidate) {
        const int direct_corridor_added = try_direct_ffb_corridor();
        if (direct_corridor_added > 0) {
            return direct_corridor_added;
        }
        if (partition_native_mode()) {
            context.diagnostics().add_counter(
                "query_bridge.partition_legacy_dense_chain_pave_skipped");
            dense_repair_attempted = true;
        }
    }
    if (dense_box_corridor_candidate && !partition_native_mode()) {
        dense_repair_attempted = true;
        ChainPaveConfig dense_config = config_.connector.pave;
        dense_config.max_chain = std::max(dense_config.max_chain, 256);
        dense_config.refine_covered_waypoints = true;
        dense_config.fill_gaps = true;
        dense_config.find_free_box.max_depth = query_bridge_ffb_depth;
	        dense_config.gap_fill_sample_step = 0.0025;
        dense_config.gap_fill_time_budget_ms = 0.0;
        dense_config.gap_fill_max_ffb_calls = -1;
        dense_config.gap_fill_min_arc_gain = 0.0;
        dense_config.require_connected_chain = true;
        const std::size_t boxes_before_dense = boxes_.size();
        dense_repair_added = chain_pave_along_path(
            corridor_path,
            start_box_id,
            boxes_,
            *oracle_,
            adjacency_,
            next_id,
            context,
            dense_config);
        if (dense_repair_added > 0) {
            append_partition_after_pave(boxes_before_dense,
                                        "query_bridge.dense_boundary_pave");
            context.diagnostics().add_counter(
                "query_bridge.full_adjacency_rebuilds_avoided");
            invalidate_query_cache();
        }
        auto [source_box_id, target_box_id] =
            try_reverse_boundary_pave(dense_config,
                                      dense_repair_added,
                                      dense_repair_added);
        if (source_box_id >= 0 &&
            target_box_id >= 0 &&
            box_only_path_connected_partition_first(source_box_id, target_box_id)) {
            const int edge_id = add_segment_edge_partition_first(	                                 source_box_id,
	                                 target_box_id,
	                                 corridor_path,
                                                 SegmentEdgeType::BoxCorridor,
                                                 bridge_rrt.segment_resolution,
                                                 SegmentEdgeValidation::CollisionChecked,
                                                 false,
                                                 bridge_edge_query_index);
            box_corridor_edges_added = edge_id >= 0 ? 1 : 0;
            if (box_corridor_edges_added > 0) {
                invalidate_query_cache();
            }
            return finish_bridge(dense_repair_added + box_corridor_edges_added);
        }
    }
    ChainPaveConfig pave_config = config_.connector.pave;
    if (defer_query_segment_edge) {
        pave_config.max_chain = std::max(pave_config.max_chain, 256);
        pave_config.refine_covered_waypoints = true;
        pave_config.fill_gaps = true;
        pave_config.find_free_box.max_depth = query_bridge_ffb_depth;
	        pave_config.gap_fill_sample_step = std::min(pave_config.gap_fill_sample_step, 0.02);
        pave_config.gap_fill_time_budget_ms =
            std::max(pave_config.gap_fill_time_budget_ms, short_local_bridge ? 350.0 : 200.0);
        pave_config.gap_fill_max_ffb_calls =
            std::max(pave_config.gap_fill_max_ffb_calls, short_local_bridge ? 768 : 512);
        pave_config.gap_fill_min_arc_gain = 0.0;
        pave_config.require_connected_chain = true;
    }
    int added = 0;
    if (!partition_native_mode()) {
        const std::size_t boxes_before_forward = boxes_.size();
        added = chain_pave_along_path(
            corridor_path,
            start_box_id,
            boxes_,
            *oracle_,
            adjacency_,
            next_id,
            context,
            pave_config);
        if (added > 0) {
            append_partition_after_pave(boxes_before_forward,
                                        "query_bridge.forward_boundary_pave");
            context.diagnostics().add_counter(
                "query_bridge.full_adjacency_rebuilds_avoided");
            invalidate_query_cache();
        }
    } else {
        context.diagnostics().add_counter(
            "query_bridge.partition_legacy_forward_chain_pave_skipped");
    }
    auto [source_box_id, target_box_id] =
        try_reverse_boundary_pave(pave_config, added, added);
    if (added > 0 &&
        source_box_id >= 0 &&
        target_box_id >= 0 &&
        box_only_path_connected_partition_first(source_box_id, target_box_id)) {
        const int edge_id = add_segment_edge_partition_first(                                             source_box_id,
                                             target_box_id,
                                             corridor_path,
                                             SegmentEdgeType::BoxCorridor,
                                             bridge_rrt.segment_resolution,
                                             SegmentEdgeValidation::CollisionChecked,
                                             false,
                                             bridge_edge_query_index);
        box_corridor_edges_added = edge_id >= 0 ? 1 : 0;
        if (box_corridor_edges_added > 0) {
            invalidate_query_cache();
        }
        return finish_bridge(added + box_corridor_edges_added);
    }
    if (dense_box_corridor_candidate && !dense_repair_attempted && !partition_native_mode()) {
        ChainPaveConfig dense_config = config_.connector.pave;
        dense_config.max_chain = std::max(dense_config.max_chain, 256);
        dense_config.refine_covered_waypoints = true;
        dense_config.fill_gaps = true;
        dense_config.find_free_box.max_depth = query_bridge_ffb_depth;
	        dense_config.gap_fill_sample_step = 0.0025;
        dense_config.gap_fill_time_budget_ms = 0.0;
        dense_config.gap_fill_max_ffb_calls = -1;
        dense_config.gap_fill_min_arc_gain = 0.0;
        dense_config.require_connected_chain = true;
        const std::size_t boxes_before_dense_retry = boxes_.size();
        dense_repair_added = chain_pave_along_path(
            corridor_path,
            start_box_id,
            boxes_,
            *oracle_,
            adjacency_,
            next_id,
            context,
            dense_config);
        if (dense_repair_added > 0) {
            append_partition_after_pave(boxes_before_dense_retry,
                                        "query_bridge.dense_boundary_retry");
            context.diagnostics().add_counter(
                "query_bridge.full_adjacency_rebuilds_avoided");
            invalidate_query_cache();
        }
        std::tie(source_box_id, target_box_id) =
            try_reverse_boundary_pave(dense_config,
                                      dense_repair_added,
                                      dense_repair_added);
        if (source_box_id >= 0 &&
            target_box_id >= 0 &&
            box_only_path_connected_partition_first(source_box_id, target_box_id)) {
            const int edge_id = add_segment_edge_partition_first(                                                 source_box_id,
                                                 target_box_id,
                                                 corridor_path,
                                                 SegmentEdgeType::BoxCorridor,
                                                 bridge_rrt.segment_resolution,
                                                 SegmentEdgeValidation::CollisionChecked,
                                                 false,
                                                 bridge_edge_query_index);
            box_corridor_edges_added = edge_id >= 0 ? 1 : 0;
            if (box_corridor_edges_added > 0) {
                invalidate_query_cache();
            }
            return finish_bridge(added + dense_repair_added + box_corridor_edges_added);
        }
    }
    if (!partition_native_mode()) {
        IslandConnectorConfig gap_config = config_.connector;
        gap_config.max_total_bridge_boxes = 0;
        IslandConnector gap_connector(*oracle_, robot_, checker, gap_config);
        const auto gap_result = gap_connector.connect_all(boxes_, adjacency_, segment_edges_, next_id, context);
        (void)gap_result;
        invalidate_query_cache();
    } else {
        context.diagnostics().add_counter(
            "query_bridge.partition_legacy_gap_connector_skipped");
    }
    source_box_id = locate_box_partition_first(start, config_.query.nearest_if_outside);
    target_box_id = locate_box_partition_first(goal, config_.query.nearest_if_outside);
    if (added > 0 &&
        source_box_id >= 0 &&
        target_box_id >= 0 &&
        box_only_path_connected_partition_first(source_box_id, target_box_id)) {
        const int edge_id = add_segment_edge_partition_first(	                                             source_box_id,
	                                             target_box_id,
	                                             corridor_path,
                                             SegmentEdgeType::BoxCorridor,
                                             bridge_rrt.segment_resolution,
                                             SegmentEdgeValidation::CollisionChecked,
                                             false,
                                             bridge_edge_query_index);
        box_corridor_edges_added = edge_id >= 0 ? 1 : 0;
        if (box_corridor_edges_added > 0) {
            invalidate_query_cache();
        }
        return finish_bridge(added + box_corridor_edges_added);
    }
    if (defer_query_segment_edge) {
        const bool max_depth_ffb_failed =
            boundary_max_depth_failure_count_local(context) >
            query_bridge_depth_failures_before + 0.5;
        if (config_.connector.segment_edges_enabled && config_.connector.rrt_segment_edges &&
            max_depth_ffb_failed) {
            if (source_box_id >= 0 && target_box_id >= 0) {
                const PathAuditCheck segment_audit =
                    audit_waypoint_path(corridor_path,
                                        checker,
                                        config_.query.audit_resolution,
                                        config_.query.audit_segment_step);
                if (!segment_audit.passed) {
                    context.diagnostics().add_counter(
                        "query_bridge.segment_edge_audit_rejects");
                    return finish_bridge(added + dense_repair_added + box_corridor_edges_added);
                }
                const int edge_id = add_segment_edge_partition_first(                                                     source_box_id,
                                                     target_box_id,
                                                     corridor_path,
                                                     SegmentEdgeType::QueryBridge,
                                                     bridge_rrt.segment_resolution,
                                                     SegmentEdgeValidation::CollisionChecked,
                                                     true,
                                                     bridge_edge_query_index);
                direct_segment_edges_added = edge_id >= 0 ? 1 : 0;
                if (direct_segment_edges_added > 0) {
                    invalidate_query_cache();
                }
            }
        } else if (config_.connector.segment_edges_enabled &&
                   config_.connector.rrt_segment_edges &&
                   !max_depth_ffb_failed) {
            context.diagnostics().add_counter(
                "query_bridge.segment_edge_blocked_no_max_depth_ffb_failure");
        }
    }
    return finish_bridge(added + dense_repair_added + box_corridor_edges_added + direct_segment_edges_added);
}

} // namespace rbf
