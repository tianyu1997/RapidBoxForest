#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>
#include <SBF/connector.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <vector>

#include "planning_forest_audit.h"

namespace rbf {

namespace {

constexpr int kSeedAttemptStride = 7919;
constexpr int kSeedDebugBridgeOffset = 701;
constexpr int kSeedCorridorRefineOffset = 809;

int derived_planner_seed(int base_seed, int offset, int attempt = 0) {
    constexpr long long modulus = 2147483647LL;
    long long value = static_cast<long long>(base_seed);
    value += static_cast<long long>(offset);
    value += static_cast<long long>(attempt) * kSeedAttemptStride;
    value %= modulus;
    if (value < 0) {
        value += modulus;
    }
    return static_cast<int>(value);
}

RRTConnectConfig with_query_root_hull_domain(const RRTConnectConfig& config,
                                             const BoxOracle& oracle,
                                             const Eigen::Ref<const Eigen::VectorXd>& start,
                                             const Eigen::Ref<const Eigen::VectorXd>& goal) {
    RRTConnectConfig out = config;
    auto lhs = oracle.planning_intervals();
    auto rhs = oracle.planning_intervals();
    (void)start;
    (void)goal;
    if (lhs.size() == rhs.size()) {
        for (std::size_t index = 0; index < lhs.size(); ++index) {
            lhs[index] = lhs[index].hull(rhs[index]);
        }
    }
    out.domain_intervals = std::move(lhs);
    return out;
}

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
    double audit_segment_step) {
    using Clock = std::chrono::steady_clock;
    std::vector<Eigen::VectorXd> best;
    double best_length = std::numeric_limits<double>::infinity();
    const int safe_attempts = std::max(1, attempts);
    const double safe_total_ms = total_timeout_ms > 0.0 ? total_timeout_ms : base_config.timeout_ms;
    const auto t0 = Clock::now();
    auto elapsed_ms = [&]() {
        return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    };
    for (int attempt = 0; attempt < safe_attempts; ++attempt) {
        if (context.should_stop()) {
            break;
        }
        RRTConnectConfig config = base_config;
        if (safe_total_ms > 0.0) {
            const double remaining_ms = safe_total_ms - elapsed_ms();
            if (remaining_ms <= 0.0) {
                break;
            }
            const int attempts_left = safe_attempts - attempt;
            config.timeout_ms = std::max(1.0, remaining_ms / static_cast<double>(attempts_left));
        }
        std::vector<Eigen::VectorXd> path =
            rrt_connect(start, goal, checker, robot, context, config,
                        seed_base + attempt * kSeedAttemptStride);
        if (path.empty()) {
            continue;
        }
        const PathAuditCheck audit =
            audit_waypoint_path(path, checker, audit_resolution, audit_segment_step);
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

} // namespace

DebugChainPaveResult RBFPlanningForest::debug_chain_pave(const Eigen::Ref<const Eigen::VectorXd>& start,
                                                         const Eigen::Ref<const Eigen::VectorXd>& goal,
                                                         const ChainPaveConfig& pave) {
    DebugChainPaveResult out;
    if (boxes_.empty() || !oracle_) {
        return out;
    }
    const int start_box_id = locate_box_partition_first(start, config_.query.nearest_if_outside);
    if (start_box_id < 0) {
        return out;
    }
    const int goal_box_id = locate_box_partition_first(goal, config_.query.nearest_if_outside);
    if (goal_box_id < 0 || goal_box_id == start_box_id) {
        return out;
    }
    out.start_box_id = start_box_id;
    out.goal_box_id = goal_box_id;
    for (const auto& box : boxes_) {
        if (box.id == start_box_id) {
            out.start_box = box.joint_intervals;
        }
        if (box.id == goal_box_id) {
            out.goal_box = box.joint_intervals;
        }
    }
    StageContext context = StageContext::from_runtime(config_.runtime);
    CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
    RRTConnectConfig bridge_rrt = with_query_root_hull_domain(config_.connector.rrt, *oracle_, start, goal);
    bridge_rrt.segment_resolution = std::max(bridge_rrt.segment_resolution, config_.query.audit_resolution);
    auto waypoint_path = rrt_connect(
        start,
        goal,
        checker,
        audit_robot_,
        context,
        bridge_rrt,
        derived_planner_seed(config_.grower.rng_seed, kSeedDebugBridgeOffset));
    if (waypoint_path.empty()) {
        return out;
    }
    out.bridge_found = true;
    out.waypoints = waypoint_path;
    out.audit_passed = audit_waypoint_path(waypoint_path,
                                           checker,
                                           config_.query.audit_resolution,
                                           config_.query.audit_segment_step)
                           .passed;
    const std::size_t boxes_before = boxes_.size();
    if (partition_native_mode()) {
        out.added = add_partition_box_corridor_overlay(start,
                                                       goal,
                                                       waypoint_path,
                                                       "debug_chain_pave",
                                                       false,
                                                       false,
                                                       -1,
                                                       &last_build_);
        out.boundary_ffb_calls = static_cast<int>(
            last_build_.diagnostics["debug_chain_pave.partition_box_corridor_overlay_attempts"]);
        out.boundary_commits = out.added;
        for (std::size_t i = boxes_before; i < boxes_.size(); ++i) {
            out.committed_boxes.push_back(boxes_[i].joint_intervals);
        }
        out.all_boxes.reserve(boxes_.size());
        for (const auto& box : boxes_) {
            out.all_boxes.push_back(box.joint_intervals);
        }
        if (out.added > 0) {
            invalidate_query_cache();
        }
        invalidate_query_cache();
        return out;
    }
    int next_id = next_box_id();
    ChainPaveConfig debug_pave = pave;
    debug_pave.debug_boundary_failures = &out.boundary_failures;
    out.added = chain_pave_along_path(
        waypoint_path,
        start_box_id,
        boxes_,
        *oracle_,
        adjacency_,
        next_id,
        context,
        debug_pave);
    if (boxes_.size() > boxes_before) {
        append_adaptive_partition_boxes(boxes_before, &last_build_, "debug_chain_pave");
    }
    out.fast_gap_fill_ffb_calls = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_fast_ffb_calls", 0.0));
    out.fast_gap_fill_ms =
        context.diagnostics().value("connector.chain_pave_fast_ms", 0.0);
    out.boundary_ffb_calls = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_ffb_calls", 0.0));
    out.boundary_commits = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_commits", 0.0));
    out.boundary_reject_not_free = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_reject_not_free", 0.0));
    out.boundary_reject_non_adjacent = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_reject_non_adjacent", 0.0));
    out.boundary_fail_seed_collision = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_seed_collision", 0.0));
    out.boundary_fail_depth_cap = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_depth_cap", 0.0));
    out.boundary_fail_unknown_depth_cap = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_unknown_depth_cap", 0.0));
    out.boundary_fail_reserved_depth_cap = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_reserved_depth_cap", 0.0));
    out.boundary_fail_occupied = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_occupied", 0.0));
    out.boundary_fail_deadline = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_deadline", 0.0));
    out.boundary_fail_out_of_domain = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_out_of_domain", 0.0));
    out.boundary_fail_split = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_split", 0.0));
    out.boundary_failed_seed_memoized = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_failed_seed_memoized", 0.0));
    out.boundary_skip_failed_seed = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_skip_failed_seed", 0.0));
    out.boundary_stall = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_stall", 0.0));
    out.boundary_target_hits = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_target_hits", 0.0));
    for (std::size_t i = boxes_before; i < boxes_.size(); ++i) {
        out.committed_boxes.push_back(boxes_[i].joint_intervals);
    }
    // Export EVERY forest box so callers can measure the bridge's true coverage:
    // chain_pave may COVER a path point by reusing a pre-existing forest box
    // (committed during build), which would otherwise be invisible to a caller
    // inspecting only `committed_boxes` and thus look like an uncovered gap.
    out.all_boxes.reserve(boxes_.size());
    for (const auto& box : boxes_) {
        out.all_boxes.push_back(box.joint_intervals);
    }
    if (out.added > 0) {
        invalidate_query_cache();
    }
    invalidate_query_cache();
    return out;
}

DebugChainPaveResult RBFPlanningForest::debug_chain_pave_waypoints(
    const std::vector<Eigen::VectorXd>& waypoint_path,
    const ChainPaveConfig& pave) {
    DebugChainPaveResult out;
    if (waypoint_path.empty() || boxes_.empty() || !oracle_) {
        return out;
    }
    const Eigen::VectorXd& start = waypoint_path.front();
    const Eigen::VectorXd& goal = waypoint_path.back();
    const int start_box_id = locate_box_partition_first(start, config_.query.nearest_if_outside);
    if (start_box_id < 0) {
        return out;
    }
    const int goal_box_id = locate_box_partition_first(goal, config_.query.nearest_if_outside);
    if (goal_box_id < 0 || goal_box_id == start_box_id) {
        return out;
    }
    out.start_box_id = start_box_id;
    out.goal_box_id = goal_box_id;
    out.bridge_found = true;
    out.waypoints = waypoint_path;
    for (const auto& box : boxes_) {
        if (box.id == start_box_id) {
            out.start_box = box.joint_intervals;
        }
        if (box.id == goal_box_id) {
            out.goal_box = box.joint_intervals;
        }
    }
    CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
    out.audit_passed = audit_waypoint_path(waypoint_path,
                                           checker,
                                           config_.query.audit_resolution,
                                           config_.query.audit_segment_step)
                           .passed;
    StageContext context = StageContext::from_runtime(config_.runtime);
    const std::size_t boxes_before = boxes_.size();
    if (partition_native_mode()) {
        out.added = add_partition_box_corridor_overlay(start,
                                                       goal,
                                                       waypoint_path,
                                                       "debug_chain_pave_waypoints",
                                                       false,
                                                       false,
                                                       -1,
                                                       &last_build_);
        out.boundary_ffb_calls = static_cast<int>(
            last_build_.diagnostics[
                "debug_chain_pave_waypoints.partition_box_corridor_overlay_attempts"]);
        out.boundary_commits = out.added;
        for (std::size_t i = boxes_before; i < boxes_.size(); ++i) {
            out.committed_boxes.push_back(boxes_[i].joint_intervals);
        }
        out.all_boxes.reserve(boxes_.size());
        for (const auto& box : boxes_) {
            out.all_boxes.push_back(box.joint_intervals);
        }
        if (out.added > 0) {
            invalidate_query_cache();
        }
        invalidate_query_cache();
        return out;
    }
    int next_id = next_box_id();
    ChainPaveConfig debug_pave = pave;
    debug_pave.debug_boundary_failures = &out.boundary_failures;
    out.added = chain_pave_along_path(
        waypoint_path,
        start_box_id,
        boxes_,
        *oracle_,
        adjacency_,
        next_id,
        context,
        debug_pave);
    if (boxes_.size() > boxes_before) {
        append_adaptive_partition_boxes(boxes_before, &last_build_, "debug_chain_pave_waypoints");
    }
    out.fast_gap_fill_ffb_calls = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_fast_ffb_calls", 0.0));
    out.fast_gap_fill_ms =
        context.diagnostics().value("connector.chain_pave_fast_ms", 0.0);
    out.boundary_ffb_calls = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_ffb_calls", 0.0));
    out.boundary_commits = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_commits", 0.0));
    out.boundary_reject_not_free = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_reject_not_free", 0.0));
    out.boundary_reject_non_adjacent = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_reject_non_adjacent", 0.0));
    out.boundary_fail_seed_collision = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_seed_collision", 0.0));
    out.boundary_fail_depth_cap = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_depth_cap", 0.0));
    out.boundary_fail_unknown_depth_cap = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_unknown_depth_cap", 0.0));
    out.boundary_fail_reserved_depth_cap = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_reserved_depth_cap", 0.0));
    out.boundary_fail_occupied = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_occupied", 0.0));
    out.boundary_fail_deadline = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_deadline", 0.0));
    out.boundary_fail_out_of_domain = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_out_of_domain", 0.0));
    out.boundary_fail_split = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_split", 0.0));
    out.boundary_failed_seed_memoized = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_failed_seed_memoized", 0.0));
    out.boundary_skip_failed_seed = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_skip_failed_seed", 0.0));
    out.boundary_stall = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_stall", 0.0));
    out.boundary_target_hits = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_target_hits", 0.0));
    for (std::size_t i = boxes_before; i < boxes_.size(); ++i) {
        out.committed_boxes.push_back(boxes_[i].joint_intervals);
    }
    out.all_boxes.reserve(boxes_.size());
    for (const auto& box : boxes_) {
        out.all_boxes.push_back(box.joint_intervals);
    }
    if (out.added > 0) {
        invalidate_query_cache();
    }
    invalidate_query_cache();
    return out;
}

int RBFPlanningForest::refine_query_corridor(const Eigen::Ref<const Eigen::VectorXd>& start,
                                         const Eigen::Ref<const Eigen::VectorXd>& goal,
                                         int max_boxes_to_add) {
    return refine_query_corridor(start,
                                 goal,
                                 max_boxes_to_add,
                                 CorridorRefineMode::LegacyBridge,
                                 std::numeric_limits<double>::infinity(),
                                 std::numeric_limits<double>::infinity());
}

int RBFPlanningForest::refine_query_corridor(const Eigen::Ref<const Eigen::VectorXd>& start,
                                         const Eigen::Ref<const Eigen::VectorXd>& goal,
                                         int max_boxes_to_add,
                                         CorridorRefineMode mode,
                                         double long_path_ratio,
                                         double long_path_min_delta) {
    if (boxes_.empty() || !oracle_ || max_boxes_to_add <= 0) {
        return 0;
    }
    CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
    QueryResult probe = run_query_internal(start, goal, false);

    std::vector<Eigen::VectorXd> waypoint_path;
    bool add_query_segment_edge = false;
    const bool graph_only_success =
        probe.success && probe.audit_passed && probe.repair_count == 0 && probe.segment_edges_used == 0 && !probe.path.empty();
    if (graph_only_success) {
        if (mode != CorridorRefineMode::BoxOnlyLongPath) {
            return 0;
        }
        const double direct = (goal - start).norm();
        const double delta = probe.path_length - direct;
        const bool ratio_trigger =
            direct > 1e-9 && std::isfinite(long_path_ratio) && probe.path_length / direct >= long_path_ratio;
        const bool delta_trigger = std::isfinite(long_path_min_delta) && delta >= long_path_min_delta;
        if (!ratio_trigger && !delta_trigger) {
            return 0;
        }
        StageContext rrt_context = StageContext::serial();
        RRTConnectConfig refine_rrt = with_query_root_hull_domain(config_.connector.rrt, *oracle_, start, goal);
        refine_rrt.segment_resolution = std::max(refine_rrt.segment_resolution, config_.query.audit_resolution);
        const int refine_attempts = std::max(1, config_.connector.max_pairs_per_gap);
        waypoint_path = best_audited_rrt_bridge_path(start,
                                                     goal,
                                                     checker,
                                                     audit_robot_,
                                                     rrt_context,
                                                     refine_rrt,
                                                     refine_attempts,
                                                     config_.connector.per_pair_timeout_ms * refine_attempts,
                                                     derived_planner_seed(config_.grower.rng_seed,
                                                                          kSeedCorridorRefineOffset,
                                                                          0),
                                                     config_.query.audit_resolution,
                                                     config_.query.audit_segment_step);
        if (waypoint_path.empty()) {
            waypoint_path = probe.path;
        }
    } else if (probe.success && probe.audit_passed && !probe.path.empty()) {
        if (mode == CorridorRefineMode::BoxOnlyLongPath) {
            const double direct = (goal - start).norm();
            const double delta = probe.path_length - direct;
            const bool ratio_trigger =
                direct > 1e-9 && std::isfinite(long_path_ratio) && probe.path_length / direct >= long_path_ratio;
            const bool delta_trigger = std::isfinite(long_path_min_delta) && delta >= long_path_min_delta;
            if (!ratio_trigger && !delta_trigger) {
                return 0;
            }
            StageContext rrt_context = StageContext::serial();
            RRTConnectConfig refine_rrt = with_query_root_hull_domain(config_.connector.rrt, *oracle_, start, goal);
            refine_rrt.segment_resolution = std::max(refine_rrt.segment_resolution, config_.query.audit_resolution);
            const int refine_attempts = std::max(1, config_.connector.max_pairs_per_gap);
            waypoint_path = best_audited_rrt_bridge_path(start,
                                                         goal,
                                                         checker,
                                                         audit_robot_,
                                                         rrt_context,
                                                         refine_rrt,
                                                         refine_attempts,
                                                         config_.connector.per_pair_timeout_ms * refine_attempts,
                                                         derived_planner_seed(config_.grower.rng_seed,
                                                                              kSeedCorridorRefineOffset,
                                                                              1),
                                                         config_.query.audit_resolution,
                                                         config_.query.audit_segment_step);
            if (waypoint_path.empty()) {
                waypoint_path = probe.path;
            }
        } else {
            waypoint_path = probe.path;
            add_query_segment_edge = true;
        }
    } else {
        StageContext rrt_context = StageContext::serial();
        RRTConnectConfig refine_rrt = with_query_root_hull_domain(config_.connector.rrt, *oracle_, start, goal);
        refine_rrt.segment_resolution = std::max(refine_rrt.segment_resolution, config_.query.audit_resolution);
        const int refine_attempts = std::max(1, config_.connector.max_pairs_per_gap);
        waypoint_path = best_audited_rrt_bridge_path(start,
                                                     goal,
                                                     checker,
                                                     audit_robot_,
                                                     rrt_context,
                                                     refine_rrt,
                                                     refine_attempts,
                                                     config_.connector.per_pair_timeout_ms * refine_attempts,
                                                     derived_planner_seed(config_.grower.rng_seed,
                                                                          kSeedCorridorRefineOffset,
                                                                          2),
                                                     config_.query.audit_resolution,
                                                     config_.query.audit_segment_step);
        add_query_segment_edge = mode != CorridorRefineMode::BoxOnlyLongPath;
    }
    if (waypoint_path.empty()) {
        return 0;
    }
    const int anchor_box_id = locate_box_partition_first(waypoint_path.front(), config_.query.nearest_if_outside);
    if (anchor_box_id < 0) {
        return 0;
    }

    if (partition_native_mode()) {
        last_build_.diagnostics["refine_query_corridor.partition_native"] = 1.0;
        const double edge_count_before =
            last_build_.diagnostics["refine_query_corridor.partition_box_corridor_overlay_added"];
        const int added = add_partition_box_corridor_overlay(start,
                                                             goal,
                                                             waypoint_path,
                                                             "refine_query_corridor",
                                                             true,
                                                             true,
                                                             -1,
                                                             &last_build_);
        const double edge_count_after =
            last_build_.diagnostics["refine_query_corridor.partition_box_corridor_overlay_added"];
        if (edge_count_after > edge_count_before) {
            last_build_.diagnostics["refine_query_corridor.partition_native_direct_overlay"] += 1.0;
            last_build_.diagnostics["refine_query_corridor.partition_native_query_bridge_skipped"] += 1.0;
            last_build_.diagnostics["refine_query_corridor.partition_native_box_corridor_edges"] += 1.0;
        }
        return added;
    }

    ChainPaveConfig pave_config = config_.connector.pave;
    pave_config.max_chain = std::min(max_boxes_to_add, std::max(1, max_boxes_to_add));
    pave_config.max_steps_per_waypoint = std::max(1, pave_config.max_steps_per_waypoint);
    pave_config.refine_covered_waypoints = true;
    if (mode == CorridorRefineMode::BoxOnlyLongPath) {
        pave_config.fill_gaps = true;
        pave_config.find_free_box.max_depth = std::max(pave_config.find_free_box.max_depth, 64);
        pave_config.gap_fill_sample_step = std::min(pave_config.gap_fill_sample_step, 0.02);
        pave_config.gap_fill_time_budget_ms = std::max(pave_config.gap_fill_time_budget_ms, 200.0);
        pave_config.gap_fill_max_ffb_calls = std::max(pave_config.gap_fill_max_ffb_calls, 512);
        pave_config.gap_fill_min_arc_gain = 0.0;
        pave_config.require_connected_chain = true;
    }
    StageContext context = StageContext::serial();
    int next_id = next_box_id();
    const std::size_t boxes_before_refine = boxes_.size();
    const int added = chain_pave_along_path(
        waypoint_path,
        anchor_box_id,
        boxes_,
        *oracle_,
        adjacency_,
        next_id,
        context,
        pave_config);
    if (added > 0) {
        append_adaptive_partition_boxes(boxes_before_refine,
                                        &last_build_,
                                        "refine_query_corridor");
        invalidate_query_cache();
        if (add_query_segment_edge &&
            config_.connector.segment_edges_enabled &&
            config_.connector.rrt_segment_edges &&
            audit_waypoint_path(waypoint_path,
                                checker,
                                config_.query.audit_resolution,
                                config_.query.audit_segment_step)
                .passed) {
            const int source_box_id = locate_box_partition_first(start, config_.query.nearest_if_outside);
            const int target_box_id = locate_box_partition_first(goal, config_.query.nearest_if_outside);
            if (source_box_id >= 0 && target_box_id >= 0) {
                add_segment_edge_partition_first(                                 source_box_id,
                                 target_box_id,
                                 waypoint_path,
                                 SegmentEdgeType::QueryBridge,
                                 std::max(1, config_.query.audit_resolution),
                                 SegmentEdgeValidation::CollisionChecked,
                                 false);
            }
        }
        invalidate_query_cache();
    }
    return added;
}

} // namespace rbf
