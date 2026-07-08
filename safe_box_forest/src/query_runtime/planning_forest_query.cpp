#include <SBF/safe_box_forest.h>

#include <SBF/scene.h>

#include <SBF/runtime.h>

#include <SBF/adaptive_grid_partition.h>
#include <SBF/box_graph.h>
#include <SBF/connector.h>
#include <SBF/oracle.h>
#include <SBF/query.h>

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <vector>

#include "../graph_partition/adaptive_grid_partition_options.h"
#include "../planning_core/planning_forest_audit.h"
#include "planning_forest_query_repair.h"
#include "planning_forest_query_utils.h"

namespace rbf {

QueryResult RBFPlanningForest::query(const Eigen::Ref<const Eigen::VectorXd>& start,
                                 const Eigen::Ref<const Eigen::VectorXd>& goal) const {
    return run_query_internal(start, goal, true, {});
}

QueryResult RBFPlanningForest::query(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    const RBFQueryRuntimeOptions& runtime_options) const {
    return run_query_internal(start, goal, true, runtime_options);
}

QueryResult RBFPlanningForest::run_query_internal(const Eigen::Ref<const Eigen::VectorXd>& start,
                                              const Eigen::Ref<const Eigen::VectorXd>& goal,
                                              bool allow_collision_shortcut,
                                              const RBFQueryRuntimeOptions& runtime_options) const {
    using Clock = std::chrono::steady_clock;
    QueryConfig query_config = config_.query;
    const QueryGraphCostOptions graph_cost =
        query_graph_cost_options_from_runtime(config_, runtime_options);
    if (!allow_collision_shortcut) {
        query_config.collision_shortcut = false;
    }
    const bool do_collision_shortcut = query_config.collision_shortcut;
    QueryResult result;
    QueryResult partition_attempt;
    if (adaptive_partition_query_enabled_ &&
        adaptive_partition_ &&
        !adaptive_partition_->empty()) {
        AdaptiveGridPartitionQueryOptions partition_options;
        partition_options.nearest_if_outside = query_config.nearest_if_outside;
        partition_options.shortcut_boxes = query_config.shortcut_boxes;
        partition_options.shortcut_cost.cost_aware = query_config.shortcut_cost_aware;
        partition_options.shortcut_cost.cost_factor =
            std::max(1.0, query_config.shortcut_cost_factor);
        partition_options.max_expansions = last_build_.diagnostics.count("adaptive.grid_planning_max_expansions") > 0
            ? static_cast<int>(last_build_.diagnostics.at("adaptive.grid_planning_max_expansions"))
            : 0;
        partition_options.adjacency_tolerance = query_config.adjacency_tolerance;
        partition_options.graph_cost = graph_cost;
        const auto partition_result = adaptive_partition_->query(start, goal, partition_options);
        result.start_box_id = partition_result.start_box_id;
        result.goal_box_id = partition_result.goal_box_id;
        result.partition_search_ms = partition_result.search_ms;
        result.query_time_ms = partition_result.search_ms;
        result.non_grid_cells_used = partition_result.non_grid_cells_used;
        if (partition_result.found) {
            result.success = true;
            result.box_sequence = partition_result.box_sequence;
            result.segment_edge_sequence = partition_result.segment_edge_sequence;
            if (result.segment_edge_sequence.size() + 1 != result.box_sequence.size()) {
                result.segment_edge_sequence.assign(
                    result.box_sequence.size() > 0 ? result.box_sequence.size() - 1 : 0,
                    -1);
            }
            result.partition_cells_used = static_cast<int>(result.box_sequence.size());
            result.path = partition_result.path;
            if (result.path.empty()) {
                if (partition_native_mode()) {
                    throw std::runtime_error(
                        "partition_native query returned a box sequence without partition waypoints");
                }
                result.path = extract_partition_waypoints_local(result.box_sequence,
                                                                result.segment_edge_sequence,
                                                                boxes_,
                                                                segment_edges_,
                                                                start,
                                                                goal,
                                                                query_config.adjacency_tolerance);
            }
            result.path_length = path_length(result.path);
            result.raw_path_length = result.path_length;
        }
        partition_attempt = result;
    }
    if (!result.success && !partition_native_mode()) {
        CorridorQuery query_engine(query_config);
        QueryResult graph_result = query_engine.run(query_cache(), start, goal, graph_cost);
        graph_result.partition_search_ms = partition_attempt.partition_search_ms;
        graph_result.partition_repair_ms = partition_attempt.partition_repair_ms;
        graph_result.partition_cells_used = partition_attempt.partition_cells_used;
        graph_result.non_grid_cells_used = partition_attempt.non_grid_cells_used;
        result = std::move(graph_result);
    }
    if (result.success && do_collision_shortcut && !query_config.strict_path_audit && result.path.size() > 2) {
        CollisionChecker checker = make_audit_checker(audit_robot_, scene_, query_config);
        result.path = collision_shortcut_path(result.path,
                                             checker,
                                             collision_shortcut_resolution(query_config));
        result.path_length = path_length(result.path);
    }
    summarize_query_path(result, boxes_, segment_edges_);
    if (!result.success && query_config.strict_path_audit && query_config.repair_on_audit_failure) {
        CollisionChecker checker = make_audit_checker(audit_robot_, scene_, query_config);
        const auto repair_t0 = Clock::now();
        RRTConnectConfig repair_config = config_.connector.rrt;
        repair_config.max_iters = std::max(repair_config.max_iters, query_config.repair_rrt_max_iters);
        if (query_config.repair_timeout_ms > 0.0) {
            repair_config.timeout_ms = query_config.repair_timeout_ms;
        }
        repair_config.segment_resolution = std::max(repair_config.segment_resolution, query_config.audit_resolution);
        std::vector<Eigen::VectorXd> repair_path = rrt_connect(
            start,
            goal,
            checker,
            audit_robot_,
            repair_config,
            derived_planner_seed(config_.grower.rng_seed, kSeedRepairGlobalOffset));
        if (!repair_path.empty()) {
            PathAuditCheck repair_audit = audit_waypoint_path(repair_path,
                                                             checker,
                                                             query_config.audit_resolution,
                                                             query_config.audit_segment_step);
            if (repair_audit.passed && do_collision_shortcut && repair_path.size() > 2) {
                std::vector<Eigen::VectorXd> shortened = collision_shortcut_path(
                    repair_path,
                    checker,
                    collision_shortcut_resolution(query_config));
                PathAuditCheck shortened_audit = audit_waypoint_path(shortened,
                                                                     checker,
                                                                     query_config.audit_resolution,
                                                                     query_config.audit_segment_step);
                if (shortened_audit.passed && path_length(shortened) <= path_length(repair_path) + 1e-12) {
                    repair_path = std::move(shortened);
                    repair_audit = shortened_audit;
                }
            }
            if (repair_audit.passed) {
                result.success = true;
                result.path = std::move(repair_path);
                result.path_length = path_length(result.path);
                result.raw_path_length = result.path_length;
                result.repair_count += 1;
                result.audit_status = PathAuditStatus::Repaired;
                result.audit_passed = true;
                result.failed_segment_index = repair_audit.failed_segment_index;
                result.remaining_unsafe_assumptions = 0;
            }
        }
        result.repair_time_ms += std::chrono::duration<double, std::milli>(Clock::now() - repair_t0).count();
    }
    if (result.success && query_config.strict_path_audit) {
        CollisionChecker checker = make_audit_checker(audit_robot_, scene_, query_config);
        auto try_final_simplify = [&]() {
            if (!query_config.final_rrt_simplify ||
                !(query_config.final_rrt_simplify_timeout_ms > 0.0) ||
                result.path_length <= 0.0) {
                return;
            }
            const auto simplify_t0 = Clock::now();
            auto simplify_elapsed_ms = [&]() {
                return std::chrono::duration<double, std::milli>(Clock::now() - simplify_t0).count();
            };
            RRTConnectConfig simplify_config = config_.connector.rrt;
            // Final OMPL-style simplification is audited in native C-space and is
            // intentionally not restricted to one active LECT root sector. The
            // raw box/segment statistics are computed before this replacement.
            simplify_config.max_iters = std::max(1, query_config.final_rrt_simplify_max_iters);
            simplify_config.segment_resolution = std::max(simplify_config.segment_resolution,
                                                          query_config.audit_resolution);
            simplify_config.segment_step = query_config.audit_segment_step;
            simplify_config.shortcut_path = true;
            const int attempts = std::max(1, query_config.final_rrt_simplify_attempts);
            for (int attempt = 0; attempt < attempts; ++attempt) {
                const double remaining_ms = query_config.final_rrt_simplify_timeout_ms - simplify_elapsed_ms();
                if (remaining_ms <= 0.0) {
                    break;
                }
                const int attempts_left = attempts - attempt;
                simplify_config.timeout_ms = std::max(1.0, remaining_ms / static_cast<double>(attempts_left));
                std::vector<Eigen::VectorXd> simplified = rrt_connect(start,
                                                                      goal,
                                                                      checker,
                                                                      audit_robot_,
                                                                      simplify_config,
                                                                      derived_planner_seed(config_.grower.rng_seed,
                                                                                           kSeedFinalSimplifyOffset,
                                                                                           attempt));
                if (!simplified.empty()) {
                    PathAuditCheck simplified_audit = audit_waypoint_path(simplified,
                                                                          checker,
                                                                          query_config.audit_resolution,
                                                                          query_config.audit_segment_step);
                    const double simplified_length = path_length(simplified);
                    if (simplified_audit.passed &&
                        simplified_length + 1e-12 < result.path_length) {
                        result.path = std::move(simplified);
                        result.path_length = simplified_length;
                        result.failed_segment_index = simplified_audit.failed_segment_index;
                        result.audit_passed = true;
                        result.audit_status = result.repair_count > 0 ? PathAuditStatus::Repaired : PathAuditStatus::Passed;
                    }
                }
            }
            const double simplify_ms =
                std::chrono::duration<double, std::milli>(Clock::now() - simplify_t0).count();
            result.final_simplify_time_ms += simplify_ms;
        };
        const auto audit_t0 = Clock::now();
        PathAuditCheck audit = audit_waypoint_path(result.path,
                                                   checker,
                                                   query_config.audit_resolution,
                                                   query_config.audit_segment_step);
        result.failed_segment_index = audit.failed_segment_index;
        if (audit.passed) {
            if (do_collision_shortcut && result.path.size() > 2) {
                std::vector<Eigen::VectorXd> shortened = collision_shortcut_path(
                    result.path,
                    checker,
                    collision_shortcut_resolution(query_config));
                PathAuditCheck shortened_audit = audit_waypoint_path(shortened,
                                                                     checker,
                                                                     query_config.audit_resolution,
                                                                     query_config.audit_segment_step);
                if (shortened_audit.passed && path_length(shortened) <= result.path_length + 1e-12) {
                    result.path = std::move(shortened);
                    result.path_length = path_length(result.path);
                    audit = shortened_audit;
                    result.failed_segment_index = audit.failed_segment_index;
                }
            }
            result.audit_status = result.repair_count > 0 ? PathAuditStatus::Repaired : PathAuditStatus::Passed;
            result.audit_passed = true;
            result.remaining_unsafe_assumptions = 0;
            try_final_simplify();
        } else if (query_config.repair_on_audit_failure) {
            const auto repair_t0 = Clock::now();
            const RRTConnectConfig repair_domain_config = oracle_
                ? with_query_root_hull_domain(config_.connector.rrt, *oracle_, start, goal)
                : config_.connector.rrt;
            const bool repaired = try_local_birrt_repair(result,
                                                         audit,
                                                         checker,
                                                         audit_robot_,
                                                         query_config,
                                                         repair_domain_config,
                                                         config_.grower.rng_seed);
            result.repair_time_ms = std::chrono::duration<double, std::milli>(Clock::now() - repair_t0).count();
            if (repaired) {
                PathAuditCheck repaired_audit = audit_waypoint_path(result.path,
                                                                    checker,
                                                                    query_config.audit_resolution,
                                                                    query_config.audit_segment_step);
                if (repaired_audit.passed && do_collision_shortcut && result.path.size() > 2) {
                    std::vector<Eigen::VectorXd> shortened = collision_shortcut_path(
                        result.path,
                        checker,
                        collision_shortcut_resolution(query_config));
                    PathAuditCheck shortened_audit = audit_waypoint_path(shortened,
                                                                         checker,
                                                                         query_config.audit_resolution,
                                                                         query_config.audit_segment_step);
                    if (shortened_audit.passed && path_length(shortened) <= result.path_length + 1e-12) {
                        result.path = std::move(shortened);
                        result.path_length = path_length(result.path);
                        repaired_audit = shortened_audit;
                    }
                }
                result.failed_segment_index = repaired_audit.failed_segment_index;
                result.audit_status = repaired_audit.passed ? PathAuditStatus::Repaired : PathAuditStatus::Failed;
                result.audit_passed = repaired_audit.passed;
                result.success = repaired_audit.passed;
                if (repaired_audit.passed) {
                    result.remaining_unsafe_assumptions = 0;
                    try_final_simplify();
                }
            } else {
                result.audit_status = PathAuditStatus::Failed;
                result.audit_passed = false;
                result.success = false;
            }
        } else {
            result.audit_status = PathAuditStatus::Failed;
            result.audit_passed = false;
            result.success = false;
        }
        result.audit_time_ms = std::chrono::duration<double, std::milli>(Clock::now() - audit_t0).count();
        summarize_query_path(result, boxes_, segment_edges_);
        if (result.audit_passed) {
            result.remaining_unsafe_assumptions = 0;
        }
    }
    return result;
}

}  // namespace rbf
