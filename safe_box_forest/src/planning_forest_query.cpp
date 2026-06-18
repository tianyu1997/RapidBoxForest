#include <SBF/safe_box_forest.h>

#include <SBF/query.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <vector>

#include "adaptive_grid_partition_options.h"
#include "env_config.h"
#include "planning_forest_audit.h"
#include "planning_forest_query_utils.h"

namespace rbf {

namespace {

using detail::env_int_or_default;

bool collision_bracket(const Eigen::VectorXd& lhs,
                       const Eigen::VectorXd& rhs,
                       const CollisionChecker& checker,
                       int resolution,
                       Eigen::VectorXd& repair_start,
                       Eigen::VectorXd& repair_goal) {
    if (!checker.check_config(lhs) && !checker.check_config(rhs)) {
        repair_start = lhs;
        repair_goal = rhs;
        return true;
    }
    const int samples = std::max(4, resolution);
    const Eigen::VectorXd diff = rhs - lhs;
    int first_collision = -1;
    int last_collision = -1;
    for (int sample = 0; sample <= samples; ++sample) {
        const double t = static_cast<double>(sample) / static_cast<double>(samples);
        const Eigen::VectorXd q = lhs + t * diff;
        if (checker.check_config(q)) {
            if (first_collision < 0) {
                first_collision = sample;
            }
            last_collision = sample;
        }
    }
    if (first_collision <= 0 || last_collision < 0 || last_collision >= samples) {
        return false;
    }
    const double t0 = static_cast<double>(first_collision - 1) / static_cast<double>(samples);
    const double t1 = static_cast<double>(last_collision + 1) / static_cast<double>(samples);
    repair_start = lhs + t0 * diff;
    repair_goal = lhs + t1 * diff;
    return !checker.check_config(repair_start) && !checker.check_config(repair_goal);
}

void summarize_query_path(QueryResult& result,
                          const std::vector<BoxNode>& boxes,
                          const SegmentEdgeList& segment_edges) {
    if (result.raw_path_length <= 0.0 && result.path_length > 0.0) {
        result.raw_path_length = result.path_length;
    }
    result.segment_edge_length = 0.0;
    result.segment_edges_used = 0;
    result.obb_edges_used = 0;
    result.obb_regions_used = 0;
    result.obb_edge_length = 0.0;
    for (int edge_id : result.segment_edge_sequence) {
        if (edge_id < 0) {
            continue;
        }
        if (const SegmentEdge* edge = find_segment_edge_by_id(segment_edges, edge_id)) {
            if (counts_as_segment_edge(edge->type)) {
                result.segment_edges_used += 1;
                result.segment_edge_length +=
                    uncovered_segment_edge_length(*edge, boxes);
            }
            if (!edge->obb_centers.empty()) {
                result.obb_edges_used += 1;
                result.obb_regions_used += static_cast<int>(edge->obb_centers.size());
                result.obb_edge_length += edge->obb_covered_length > 0.0
                    ? edge->obb_covered_length
                    : edge->length;
            }
        }
    }
    int provisional_or_unknown_boxes = 0;
    for (int box_id : result.box_sequence) {
        const BoxNode* box = find_box_by_id(boxes, box_id);
        if (box == nullptr || box->safety_status != BoxSafetyStatus::CertifiedFree || box->strict_audit_required) {
            provisional_or_unknown_boxes += 1;
        }
    }
    const double box_path_length = std::max(0.0, result.path_length - result.segment_edge_length);
    const double residual_denominator =
        result.raw_path_length > 1e-12 ? result.raw_path_length : result.path_length;
    result.residual_segment_fraction =
        residual_denominator > 1e-12
            ? result.segment_edge_length / residual_denominator
            : 0.0;
    if (provisional_or_unknown_boxes == 0) {
        result.certified_box_length = box_path_length;
        result.provisional_audited_length = 0.0;
    } else {
        result.certified_box_length = 0.0;
        result.provisional_audited_length = box_path_length;
    }
    result.remaining_unsafe_assumptions = provisional_or_unknown_boxes;
}

bool try_local_birrt_repair(QueryResult& result,
                            const PathAuditCheck& audit,
                            const CollisionChecker& checker,
                            const Robot& robot,
                            const QueryConfig& query_config,
                            const RRTConnectConfig& base_repair_config,
                            int planner_seed_base) {
    if (audit.failed_segment_index < 0 || audit.failed_segment_index + 1 >= static_cast<int>(result.path.size())) {
        return false;
    }
    Eigen::VectorXd repair_start;
    Eigen::VectorXd repair_goal;
    if (!collision_bracket(result.path[static_cast<std::size_t>(audit.failed_segment_index)],
                           result.path[static_cast<std::size_t>(audit.failed_segment_index + 1)],
                           checker,
                           query_config.audit_resolution,
                           repair_start,
                           repair_goal)) {
        return false;
    }

    RRTConnectConfig repair_config = base_repair_config;
    repair_config.max_iters = std::max(repair_config.max_iters, query_config.repair_rrt_max_iters);
    if (query_config.repair_timeout_ms > 0.0) {
        repair_config.timeout_ms = query_config.repair_timeout_ms;
    }
    repair_config.segment_resolution = std::max(repair_config.segment_resolution, query_config.audit_resolution);

    const int attempts = std::max(1, query_config.repair_max_attempts);
    std::vector<Eigen::VectorXd> best_repaired;
    double best_length = std::numeric_limits<double>::infinity();
    for (int attempt = 0; attempt < attempts; ++attempt) {
        RRTConnectConfig attempt_config = repair_config;
        if (query_config.repair_local_sampling_radius > 0.0 && attempt + 1 < attempts) {
            const double growth = std::max(1.0, query_config.repair_local_sampling_growth);
            attempt_config.local_sampling_radius = query_config.repair_local_sampling_radius * std::pow(growth, attempt);
        }
        auto repair_path = rrt_connect(repair_start,
                                       repair_goal,
                                       checker,
                                       robot,
                                       attempt_config,
                                       derived_planner_seed(planner_seed_base,
                                                            kSeedRepairLocalOffset,
                                                            attempt,
                                                            0,
                                                            audit.failed_segment_index));
        if (repair_path.empty()) {
            continue;
        }
        std::vector<Eigen::VectorXd> repaired;
        repaired.reserve(result.path.size() + repair_path.size() + 2);
        for (int index = 0; index <= audit.failed_segment_index; ++index) {
            append_waypoint_unique(repaired, result.path[static_cast<std::size_t>(index)]);
        }
        append_waypoint_unique(repaired, repair_start);
        for (const auto& waypoint : repair_path) {
            append_waypoint_unique(repaired, waypoint);
        }
        append_waypoint_unique(repaired, repair_goal);
        for (std::size_t index = static_cast<std::size_t>(audit.failed_segment_index + 1); index < result.path.size(); ++index) {
            append_waypoint_unique(repaired, result.path[index]);
        }
        if (audit_waypoint_path(repaired,
                                checker,
                                query_config.audit_resolution,
                                query_config.audit_segment_step)
                .passed) {
            if (query_config.collision_shortcut && repaired.size() > 2) {
                std::vector<Eigen::VectorXd> shortened = collision_shortcut_path(
                    repaired,
                    checker,
                    collision_shortcut_resolution(query_config));
                if (audit_waypoint_path(shortened,
                                        checker,
                                        query_config.audit_resolution,
                                        query_config.audit_segment_step)
                        .passed &&
                    path_length(shortened) <= path_length(repaired) + 1e-12) {
                    repaired = std::move(shortened);
                }
            }
            const double repaired_length = path_length(repaired);
            if (repaired_length < best_length) {
                best_length = repaired_length;
                best_repaired = std::move(repaired);
            }
        }
    }
    if (!best_repaired.empty()) {
        result.path = std::move(best_repaired);
        result.path_length = best_length;
        result.repair_count += 1;
        return true;
    }
    return false;
}

}  // namespace

QueryResult RBFPlanningForest::query(const Eigen::Ref<const Eigen::VectorXd>& start,
                                 const Eigen::Ref<const Eigen::VectorXd>& goal) const {
    return run_query_internal(start, goal, true);
}

QueryResult RBFPlanningForest::run_query_internal(const Eigen::Ref<const Eigen::VectorXd>& start,
                                              const Eigen::Ref<const Eigen::VectorXd>& goal,
                                              bool allow_collision_shortcut) const {
    using Clock = std::chrono::steady_clock;
    QueryConfig query_config = config_.query;
    if (!allow_collision_shortcut) {
        query_config.collision_shortcut = false;
    }
    const bool do_collision_shortcut = query_config.collision_shortcut;
    const int active_query_index = env_int_or_default("RBF_ACTIVE_QUERY_INDEX", -1);
    const bool partition_last_query_cache_enabled =
        partition_native_mode() &&
        adaptive_partition_query_enabled_ &&
        adaptive_partition_ &&
        partition_last_query_cache_enabled_from_env();
    auto same_vector = [](const Eigen::VectorXd& lhs,
                          const Eigen::Ref<const Eigen::VectorXd>& rhs) {
        return lhs.size() == rhs.size() &&
               (lhs.size() == 0 || (lhs - rhs).cwiseAbs().maxCoeff() <= 0.0);
    };
    if (partition_last_query_cache_enabled &&
        partition_last_query_cache_.valid &&
        partition_last_query_cache_.allow_collision_shortcut == allow_collision_shortcut &&
        partition_last_query_cache_.active_query_index == active_query_index &&
        same_vector(partition_last_query_cache_.start, start) &&
        same_vector(partition_last_query_cache_.goal, goal)) {
        QueryResult cached = partition_last_query_cache_.result;
        cached.query_time_ms = 0.0;
        cached.partition_search_ms = 0.0;
        cached.audit_time_ms = 0.0;
        cached.final_simplify_time_ms = 0.0;
        return cached;
    }
    QueryResult result;
    QueryResult partition_attempt;
    if (adaptive_partition_query_enabled_ &&
        adaptive_partition_ &&
        !adaptive_partition_->empty()) {
        AdaptiveGridPartitionQueryOptions partition_options;
        partition_options.nearest_if_outside = query_config.nearest_if_outside;
        partition_options.shortcut_boxes = query_config.shortcut_boxes;
        partition_options.max_expansions = last_build_.diagnostics.count("adaptive.grid_planning_max_expansions") > 0
            ? static_cast<int>(last_build_.diagnostics.at("adaptive.grid_planning_max_expansions"))
            : 0;
        partition_options.adjacency_tolerance = query_config.adjacency_tolerance;
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
        QueryResult graph_result = query_engine.run(query_cache(), start, goal);
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
    if (partition_last_query_cache_enabled) {
        partition_last_query_cache_.valid = true;
        partition_last_query_cache_.allow_collision_shortcut = allow_collision_shortcut;
        partition_last_query_cache_.active_query_index = active_query_index;
        partition_last_query_cache_.start = start;
        partition_last_query_cache_.goal = goal;
        partition_last_query_cache_.result = result;
    }
    return result;
}

}  // namespace rbf
