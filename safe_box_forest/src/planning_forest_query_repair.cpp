#include "planning_forest_query_repair.h"

#include "planning_forest_query_utils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace rbf {

namespace {

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

}  // namespace

QueryGraphCostOptions query_graph_cost_options_from_runtime(
    const RBFPlanningConfig& config,
    const RBFQueryRuntimeOptions& runtime_options) {
    QueryGraphCostOptions options;
    options.box_transition_penalty =
        std::max(0.0, config.query_box_transition_edge_cost_penalty);
    options.box_nonprogress_penalty =
        std::max(0.0, config.query_box_transition_nonprogress_penalty);
    options.box_line_deviation_penalty =
        std::max(0.0, config.query_box_transition_line_deviation_penalty);
    options.query_bridge_penalty =
        std::max(0.0, config.query_bridge_edge_cost_penalty);
    options.active_query_index = runtime_options.active_query_index;
    options.foreign_query_edge_penalty = options.active_query_index >= 0
        ? std::max(0.0, config.query_foreign_edge_cost_penalty)
        : 0.0;
    return options;
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

}  // namespace rbf
