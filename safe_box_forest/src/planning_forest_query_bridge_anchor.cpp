#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>
#include <SBF/connector.h>

#include "planning_forest_audit.h"
#include "planning_forest_diagnostics.h"
#include "planning_forest_qroot_helpers.h"
#include "planning_forest_query_utils.h"
#include "virtual_sparse_ffb.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace rbf {

int RBFPlanningForest::anchor_query_endpoint(const Eigen::Ref<const Eigen::VectorXd>& point) {
    StageContext context = StageContext::from_runtime(config_.runtime);
    const int box_id = anchor_query_endpoint_box(point, context);
    merge_diagnostic_snapshot(last_build_.diagnostics, context.diagnostics().snapshot());
    return box_id;
}

int RBFPlanningForest::anchor_query_endpoint_box(const Eigen::Ref<const Eigen::VectorXd>& point,
                                                 StageContext& context) {
    if (!oracle_) {
        return -1;
    }
    record_portal_membership_policy(context.diagnostics(), config_.portal_membership_policy);
    context.diagnostics().add_counter("portal_membership.global_forest_lookup");
    int existing = -1;
    for (const auto& box : boxes_) {
        if (box.contains(point, config_.query.adjacency_tolerance)) {
            existing = box.id;
            break;
        }
    }
    if (existing < 0 && config_.query.nearest_if_outside) {
        existing = locate_box_partition_first(point, config_.query.nearest_if_outside);
    }
    if (existing >= 0) {
        context.diagnostics().add_counter("query_bridge.endpoint_anchor_already_covered");
        return existing;
    }

    FindFreeBoxOptions options = config_.connector.pave.find_free_box;
    const int requested_anchor_depth =
        config_.query_endpoint_anchor_ffb_depth > 0
            ? config_.query_endpoint_anchor_ffb_depth
            : config_.query_bridge_pave_depth;
    if (requested_anchor_depth > 0) {
        options.max_depth = std::min(std::max(1, config_.database.max_tree_depth),
                                     std::max(1, requested_anchor_depth));
    }
    options.reject_seed_collision = false;
    std::vector<int> anchor_depth_schedule = config_.query_endpoint_anchor_ffb_depths;
    const int max_tree_depth = std::max(1, config_.database.max_tree_depth);
    auto normalize_depth = [&](int depth) {
        return std::min(max_tree_depth, std::max(1, depth));
    };
    std::vector<int> normalized_depths;
    normalized_depths.reserve(anchor_depth_schedule.size() + 1U);
    for (int depth : anchor_depth_schedule) {
        if (depth <= 0) {
            continue;
        }
        const int normalized = normalize_depth(depth);
        if (std::find(normalized_depths.begin(), normalized_depths.end(), normalized) ==
            normalized_depths.end()) {
            normalized_depths.push_back(normalized);
        }
    }
    if (normalized_depths.empty()) {
        normalized_depths.push_back(normalize_depth(options.max_depth));
    } else {
        const int final_depth = normalize_depth(options.max_depth);
        if (std::find(normalized_depths.begin(), normalized_depths.end(), final_depth) ==
            normalized_depths.end()) {
            normalized_depths.push_back(final_depth);
        }
    }
    context.diagnostics().set_value("query_bridge.endpoint_anchor_ffb_depth",
                                    static_cast<double>(normalized_depths.back()));
    context.diagnostics().set_value("query_bridge.endpoint_anchor_ffb_depth_schedule_size",
                                    static_cast<double>(normalized_depths.size()));

    BoxNode root_domain;
    root_domain.id = -1;
    root_domain.joint_intervals = oracle_->planning_intervals();
    root_domain.compute_volume();

    if (partition_native_mode()) {
        const bool endpoint_point_anchor = config_.query_endpoint_point_anchor;
        context.diagnostics().set_value("query_bridge.endpoint_point_anchor_enabled",
                                        endpoint_point_anchor ? 1.0 : 0.0);
        if (endpoint_point_anchor) {
            bool in_domain = static_cast<int>(root_domain.joint_intervals.size()) == point.size();
            if (in_domain) {
                for (int dim = 0; dim < point.size(); ++dim) {
                    if (!root_domain.joint_intervals[static_cast<std::size_t>(dim)].contains(point[dim], 0.0)) {
                        in_domain = false;
                        break;
                    }
                }
            }
            if (!in_domain) {
                context.diagnostics().add_counter("query_bridge.endpoint_point_anchor_domain_rejects");
            } else {
                context.diagnostics().add_counter("query_bridge.endpoint_point_anchor_attempts");
                CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
                if (!checker.check_config(point)) {
                    const std::size_t boxes_before_anchor = boxes_.size();
                    BoxNode box;
                    box.id = next_box_id();
                    box.joint_intervals.reserve(static_cast<std::size_t>(point.size()));
                    for (int dim = 0; dim < point.size(); ++dim) {
                        box.joint_intervals.push_back(Interval{point[dim], point[dim]});
                    }
                    box.seed_config = point;
                    box.tree_id = kInvalidOracleNodeId;
                    box.parent_box_id = -1;
                    box.root_id = box.id;
                    box.safety_status = BoxSafetyStatus::CertifiedFree;
                    box.strict_audit_required = false;
                    box.compute_volume();
                    const int new_id = box.id;
                    boxes_.push_back(box);
                    raw_boxes_.push_back(box);
                    context.diagnostics().add_counter("query_bridge.endpoint_point_anchor_success");
                    context.diagnostics().add_counter("query_bridge.endpoint_anchor_boxes_added");
                    append_adaptive_partition_boxes(boxes_before_anchor,
                                                    &last_build_,
                                                    "query_bridge.endpoint_point_anchor");
                    invalidate_query_cache();
                    return new_id;
                }
                context.diagnostics().add_counter("query_bridge.endpoint_point_anchor_collision_rejects");
            }
        }
        const std::size_t boxes_before_anchor = boxes_.size();
        StageContext local_context = context;
        FindFreeBoxOptions anchor_options = options;
        anchor_options.materialize_result_node = false;
        auto result = find_free_box_in_domain(point,
                                              root_domain.joint_intervals,
                                              local_context,
                                              anchor_options);
        merge_diagnostic_snapshot(context.diagnostics(), local_context.diagnostics().snapshot());
        context.diagnostics().add_counter("query_bridge.endpoint_anchor_calls");
        if (!result.found ||
            !intervals_contain_point_local(result.intervals,
                                           point,
                                           config_.query.adjacency_tolerance)) {
            context.diagnostics().add_counter("query_bridge.endpoint_anchor_ffb_fail");
            context.diagnostics().add_counter(
                "query_bridge.endpoint_anchor_ffb_fail_code." +
                std::to_string(result.fail_code));
            context.diagnostics().set_value("query_bridge.endpoint_anchor_last_fail_code",
                                            static_cast<double>(result.fail_code));
            if (result.seed_collision) {
                context.diagnostics().add_counter("query_bridge.endpoint_anchor_seed_collision");
            }
            if (result.hit_unknown_depth_cap) {
                context.diagnostics().add_counter("query_bridge.endpoint_anchor_unknown_depth_cap");
            }
            if (result.hit_reserved_depth_cap) {
                context.diagnostics().add_counter("query_bridge.endpoint_anchor_reserved_depth_cap");
            }
            if (result.deadline_reached) {
                context.diagnostics().add_counter("query_bridge.endpoint_anchor_deadline");
            }
            return -1;
        }
        context.diagnostics().add_counter("query_bridge.endpoint_anchor_ffb_success");
        for (const auto& existing_box : boxes_) {
            if ((result.node != kInvalidOracleNodeId &&
                 existing_box.tree_id == result.node) ||
                intervals_equal_local(existing_box.joint_intervals,
                                      result.intervals,
                                      1e-12)) {
                context.diagnostics().add_counter("query_bridge.endpoint_anchor_duplicate_reuse");
                return existing_box.id;
            }
        }
        if (!allow_dynamic_commit(*oracle_, result, config_.grower.commit_policy)) {
            context.diagnostics().add_counter("query_bridge.endpoint_anchor_commit_rejects");
            return -1;
        }
        BoxNode box;
        box.id = next_box_id();
        box.joint_intervals = result.intervals;
        box.seed_config = point;
        box.tree_id = result.node;
        box.parent_box_id = -1;
        box.root_id = box.id;
        box.safety_status = result.validation_detail.safety_status;
        box.strict_audit_required = result.validation_detail.strict_audit_required;
        box.compute_volume();
        if (box.tree_id != kInvalidOracleNodeId) {
            oracle_->reserve_node(box.tree_id, box.id);
        }
        const int new_id = box.id;
        boxes_.push_back(box);
        raw_boxes_.push_back(box);
        context.diagnostics().add_counter("query_bridge.endpoint_anchor_boxes_added");
        append_adaptive_partition_boxes(boxes_before_anchor,
                                        &last_build_,
                                        "query_bridge.endpoint_anchor");
        return new_id;
    }

    BoxSpatialIndex box_index;
    box_index.rebuild(boxes_, config_.query.adjacency_tolerance);
    BuildDisjointSet dsu = make_dsu_from_graph(boxes_, adjacency_);
    int next_id = next_box_id();
    QueryRootGrowResult stats;
    auto find_in_domain = [this](const Eigen::VectorXd& seed,
                                 const std::vector<Interval>& domain,
                                 StageContext& local_context,
                                 const FindFreeBoxOptions& local_options) {
        return this->find_free_box_in_domain(seed, domain, local_context, local_options);
    };
    const std::size_t boxes_before_anchor = boxes_.size();
    const int new_id = commit_query_root_box(*oracle_,
                                             options,
                                             config_.grower.commit_policy,
                                             find_in_domain,
                                             point,
                                             root_domain,
                                             -1,
                                             -1,
                                             boxes_,
                                             raw_boxes_,
                                             adjacency_,
                                             box_index,
                                             dsu,
                                             next_id,
                                             context,
                                             stats,
                                             config_.query.adjacency_tolerance);
    context.diagnostics().add_counter("query_bridge.endpoint_anchor_calls");
    context.diagnostics().add_counter("query_bridge.endpoint_anchor_ffb_success", stats.ffb_success);
    context.diagnostics().add_counter("query_bridge.endpoint_anchor_ffb_fail", stats.ffb_fail);
    if (stats.ffb_fail > 0) {
        context.diagnostics().add_counter("query_bridge.endpoint_anchor_ffb_fail_code.unknown_legacy",
                                          stats.ffb_fail);
    }
    context.diagnostics().add_counter("query_bridge.endpoint_anchor_commit_rejects", stats.commit_rejects);
    context.diagnostics().add_counter("query_bridge.endpoint_anchor_domain_rejects", stats.domain_rejects);
    context.diagnostics().add_counter("query_bridge.endpoint_anchor_contained_rejects", stats.contained_rejects);
    context.diagnostics().add_counter("query_bridge.endpoint_anchor_adjacency_rejects", stats.adjacency_rejects);
    context.diagnostics().add_counter("query_bridge.endpoint_anchor_boxes_added", stats.boxes_added);
    if (new_id >= 0) {
        append_adaptive_partition_boxes(boxes_before_anchor,
                                        &last_build_,
                                        "query_bridge.endpoint_anchor");
        const BoxNode* anchor_box = find_box_by_id(boxes_, new_id);
        int best_target_id = -1;
        Eigen::VectorXd best_target_point = point;
        double best_dist2 = std::numeric_limits<double>::infinity();
        auto consider_target = [&](bool require_graph_degree) {
            for (const auto& candidate : boxes_) {
                if (candidate.id == new_id || candidate.n_dims() != point.size()) {
                    continue;
                }
                if (require_graph_degree) {
                    const auto graph_it = adjacency_.find(candidate.id);
                    if (graph_it == adjacency_.end() || graph_it->second.empty()) {
                        continue;
                    }
                }
                const Eigen::VectorXd target = closest_point_in_box(candidate, point);
                const double dist2 = (target - point).squaredNorm();
                if (dist2 < best_dist2) {
                    best_dist2 = dist2;
                    best_target_id = candidate.id;
                    best_target_point = target;
                }
            }
        };
        consider_target(true);
        if (best_target_id < 0) {
            consider_target(false);
        }
        const double max_shortlink_length =
            std::max(0.0, config_.endpoint_shortlink_max_length);
        if (anchor_box != nullptr &&
            best_target_id >= 0 &&
            best_dist2 > 1e-18 &&
            std::sqrt(best_dist2) <= max_shortlink_length) {
            context.diagnostics().add_counter("query_bridge.endpoint_anchor_shortlink_attempts");
            CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
            std::vector<Eigen::VectorXd> waypoints{point, best_target_point};
            const PathAuditCheck audit = audit_waypoint_path(waypoints,
                                                             checker,
                                                             config_.query.audit_resolution,
                                                             config_.query.audit_segment_step);
            if (audit.passed) {
                const int edge_id = add_segment_edge_partition_first(new_id,
                                                                     best_target_id,
                                                                     std::move(waypoints),
                                                                     SegmentEdgeType::QueryBridge,
                                                                     config_.query.audit_resolution,
                                                                     SegmentEdgeValidation::CollisionChecked,
                                                                     true,
                                                                     -1);
                if (edge_id >= 0) {
                    context.diagnostics().add_counter("query_bridge.endpoint_anchor_shortlink_success");
                    context.diagnostics().add_counter("query_bridge.endpoint_anchor_shortlink_length",
                                                      std::sqrt(best_dist2));
                }
            } else {
                context.diagnostics().add_counter("query_bridge.endpoint_anchor_shortlink_audit_fail");
            }
        }
        invalidate_query_cache();
    }
    return new_id;
}

int RBFPlanningForest::anchor_query_endpoint_box_with_diagnostics(
    const Eigen::Ref<const Eigen::VectorXd>& point) {
    StageContext anchor_context = StageContext::from_runtime(config_.runtime);
    const int box_id = anchor_query_endpoint_box(point, anchor_context);
    merge_diagnostic_snapshot(last_build_.diagnostics, anchor_context.diagnostics().snapshot());
    return box_id;
}

}  // namespace rbf
