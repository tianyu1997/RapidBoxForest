#include <SBF/safe_box_forest.h>

#include <SBF/scene.h>
#include <SBF/adaptive_grid_partition.h>
#include <SBF/box_graph.h>
#include <SBF/oracle.h>
#include <SBF/runtime.h>

#include <algorithm>
#include <chrono>
#include <string>
#include <vector>

#include "../planning_core/planning_forest_audit.h"
#include "../obb/planning_forest_obb.h"
#include "../query_runtime/planning_forest_query_utils.h"

namespace rbf {

int RBFPlanningForest::add_partition_box_corridor_overlay(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    const std::vector<Eigen::VectorXd>& waypoint_path,
    const char* diagnostic_prefix,
    bool anchor_endpoints,
    bool skip_if_connected,
    int query_index,
    BuildProfile* profile) {
    BuildProfile* out_profile = profile != nullptr ? profile : &last_build_;
    const std::string prefix =
        (diagnostic_prefix != nullptr && diagnostic_prefix[0] != '\0')
            ? std::string(diagnostic_prefix)
            : std::string("partition_box_corridor");
    auto& diagnostics = out_profile->diagnostics;
    diagnostics[prefix + ".partition_box_corridor_overlay_attempts"] += 1.0;

    if (!partition_native_mode()) {
        diagnostics[prefix + ".partition_box_corridor_overlay_not_partition_native"] += 1.0;
        return 0;
    }
    if (!adaptive_partition_query_enabled_ || !adaptive_partition_ || adaptive_partition_->empty()) {
        diagnostics[prefix + ".partition_box_corridor_overlay_missing_partition"] += 1.0;
        return 0;
    }
    if (waypoint_path.size() < 2) {
        diagnostics[prefix + ".partition_box_corridor_overlay_missing_waypoints"] += 1.0;
        return 0;
    }

    StageContext anchor_context = StageContext::from_runtime(config_.runtime);
    const std::size_t boxes_before_anchor = boxes_.size();
    int source_box_id = locate_box_partition_first(start, false);
    if (source_box_id < 0 && anchor_endpoints) {
        source_box_id = anchor_query_endpoint_box(start, anchor_context);
    }
    int target_box_id = locate_box_partition_first(goal, false);
    if (target_box_id < 0 && anchor_endpoints) {
        target_box_id = anchor_query_endpoint_box(goal, anchor_context);
    }
    for (const auto& [key, value] : anchor_context.diagnostics().snapshot()) {
        diagnostics[prefix + ".partition_box_corridor_anchor." + key] += value;
    }

    int anchors_added = 0;
    if (boxes_.size() > boxes_before_anchor) {
        const std::string anchor_prefix = prefix + ".partition_box_corridor_anchor";
        anchors_added = append_adaptive_partition_boxes(boxes_before_anchor,
                                                        out_profile,
                                                        anchor_prefix.c_str());
        source_box_id = locate_box_partition_first(start, false);
        target_box_id = locate_box_partition_first(goal, false);
    }
    if (anchors_added > 0) {
        diagnostics[prefix + ".partition_box_corridor_overlay_anchor_boxes_added"] +=
            static_cast<double>(anchors_added);
    }

    if (source_box_id < 0 || target_box_id < 0) {
        diagnostics[prefix + ".partition_box_corridor_overlay_missing_endpoint"] += 1.0;
        return anchors_added;
    }
    if (source_box_id == target_box_id ||
        (skip_if_connected && overlay_path_connected_partition_first(source_box_id, target_box_id))) {
        diagnostics[prefix + ".partition_box_corridor_overlay_already_connected"] += 1.0;
        return anchors_added;
    }

    CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
    const PathAuditCheck audit = audit_waypoint_path(waypoint_path,
                                                     checker,
                                                     config_.query.audit_resolution,
                                                     config_.query.audit_segment_step);
    if (!audit.passed) {
        diagnostics[prefix + ".partition_box_corridor_overlay_audit_fail"] += 1.0;
        return anchors_added;
    }

    const std::string edge_prefix = prefix + ".partition_native";
    const int edge_id = add_segment_edge_partition_first(source_box_id,
                                                         target_box_id,
                                                         waypoint_path,
                                                         SegmentEdgeType::BoxCorridor,
                                                         std::max(1, config_.query.audit_resolution),
                                                         SegmentEdgeValidation::CollisionChecked,
                                                         false,
                                                         query_index,
                                                         out_profile,
                                                         edge_prefix.c_str());
    if (edge_id < 0) {
        diagnostics[prefix + ".partition_box_corridor_overlay_edge_fail"] += 1.0;
        return anchors_added;
    }
    diagnostics[prefix + ".partition_box_corridor_overlay_added"] += 1.0;
    invalidate_query_cache();
    return anchors_added + 1;
}

int RBFPlanningForest::add_partition_portal_corridor_overlay(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    const std::vector<Eigen::VectorXd>& waypoint_path,
    const char* diagnostic_prefix,
    bool anchor_endpoints,
    bool skip_if_connected,
    int query_index,
    BuildProfile* profile) {
    BuildProfile* out_profile = profile != nullptr ? profile : &last_build_;
    const std::string prefix =
        (diagnostic_prefix != nullptr && diagnostic_prefix[0] != '\0')
            ? std::string(diagnostic_prefix)
            : std::string("partition_portal_corridor");
    auto& diagnostics = out_profile->diagnostics;
    diagnostics[prefix + ".portal_corridor_attempts"] += 1.0;
    const bool online_portal_prefix =
        prefix.find("hipac_online") != std::string::npos ||
        prefix.find("hipac_promote") != std::string::npos;

    if (!last_adaptive_partition_config_.hipac_portal_connectivity &&
        !(online_portal_prefix && last_adaptive_partition_config_.hipac_online_connectivity)) {
        diagnostics[prefix + ".portal_corridor_disabled"] += 1.0;
        return 0;
    }
    if (!partition_native_mode()) {
        diagnostics[prefix + ".portal_corridor_not_partition_native"] += 1.0;
        return 0;
    }
    if (!adaptive_partition_query_enabled_ || !adaptive_partition_ || adaptive_partition_->empty()) {
        diagnostics[prefix + ".portal_corridor_missing_partition"] += 1.0;
        return 0;
    }
    if (!oracle_ || waypoint_path.size() < 2) {
        diagnostics[prefix + ".portal_corridor_missing_waypoints"] += 1.0;
        return 0;
    }

    StageContext anchor_context = StageContext::from_runtime(config_.runtime);
    const std::size_t boxes_before_anchor = boxes_.size();
    int source_box_id = locate_box_partition_first(start, false);
    if (source_box_id < 0 && anchor_endpoints) {
        source_box_id = anchor_query_endpoint_box(start, anchor_context);
    }
    int target_box_id = locate_box_partition_first(goal, false);
    if (target_box_id < 0 && anchor_endpoints) {
        target_box_id = anchor_query_endpoint_box(goal, anchor_context);
    }
    for (const auto& [key, value] : anchor_context.diagnostics().snapshot()) {
        diagnostics[prefix + ".portal_corridor_anchor." + key] += value;
    }

    int anchors_added = 0;
    if (boxes_.size() > boxes_before_anchor) {
        const std::string anchor_prefix = prefix + ".portal_corridor_anchor";
        anchors_added = append_adaptive_partition_boxes(boxes_before_anchor,
                                                        out_profile,
                                                        anchor_prefix.c_str());
        source_box_id = locate_box_partition_first(start, false);
        target_box_id = locate_box_partition_first(goal, false);
    }
    if (anchors_added > 0) {
        diagnostics[prefix + ".portal_corridor_anchor_boxes_added"] +=
            static_cast<double>(anchors_added);
    }
    if (source_box_id < 0 || target_box_id < 0) {
        diagnostics[prefix + ".portal_corridor_missing_endpoint"] += 1.0;
        return anchors_added;
    }
    if (source_box_id == target_box_id ||
        (skip_if_connected && overlay_path_connected_partition_first(source_box_id, target_box_id))) {
        diagnostics[prefix + ".portal_corridor_already_connected"] += 1.0;
        return anchors_added;
    }

    const BoxNode* source_ptr = find_box_by_id(boxes_, source_box_id);
    const BoxNode* target_ptr = find_box_by_id(boxes_, target_box_id);
    if (source_ptr == nullptr || target_ptr == nullptr) {
        diagnostics[prefix + ".portal_corridor_missing_box"] += 1.0;
        return anchors_added;
    }
    const BoxNode source_box = *source_ptr;
    const BoxNode target_box = *target_ptr;

    const bool online_portal = online_portal_prefix;
    const auto domain = oracle_->planning_intervals();
    const bool transition_obb_prefix =
        prefix.find("hipac_promote_transition") != std::string::npos;
    if (transition_obb_prefix && last_adaptive_partition_config_.hipac_transition_obb_portal) {
        auto obb_t0 = std::chrono::steady_clock::now();
        std::vector<Eigen::VectorXd> obb_path;
        obb_path.reserve(waypoint_path.size() + 2U);
        auto append_unique = [&](const Eigen::VectorXd& waypoint) {
            if (waypoint.size() != start.size()) {
                return;
            }
            if (obb_path.empty() || (obb_path.back() - waypoint).norm() > 1e-12) {
                obb_path.push_back(waypoint);
            }
        };
        append_unique(start);
        for (const auto& waypoint : waypoint_path) {
            append_unique(waypoint);
        }
        append_unique(goal);

        ObbPortalValidationStats obb_stats;
        const double obb_safety_epsilon =
            std::max(0.0, last_adaptive_partition_config_.hipac_transition_obb_safety_epsilon);
        diagnostics[prefix + ".obb_zonotope_attempts"] += 1.0;
        Eigen::VectorXd obb_center;
        Eigen::MatrixXd obb_generators;
        const bool obb_ok = validate_obb_zonotope_portal(
            robot_,
            scene_,
            domain,
            obb_path,
            last_adaptive_partition_config_.hipac_transition_obb_lateral_radius,
            last_adaptive_partition_config_.hipac_transition_obb_longitudinal_margin,
            obb_safety_epsilon,
            last_adaptive_partition_config_.segment_edge_obb_grow_iterations,
            last_adaptive_partition_config_.segment_edge_obb_binary_iterations,
            last_adaptive_partition_config_.obb_max_validations_per_window,
            obb_stats,
            &obb_center,
            &obb_generators,
            obb_validation_options_from_config(last_adaptive_partition_config_));
        diagnostics[prefix + ".obb_zonotope_variables"] =
            static_cast<double>(obb_stats.variables);
        diagnostics[prefix + ".obb_zonotope_active_links"] =
            static_cast<double>(obb_stats.active_links);
        diagnostics[prefix + ".obb_zonotope_longitudinal_radius"] =
            obb_stats.longitudinal_radius;
        diagnostics[prefix + ".obb_zonotope_lateral_radius"] =
            obb_stats.lateral_radius;
        diagnostics[prefix + ".obb_zonotope_joint_limit_rejects"] +=
            static_cast<double>(obb_stats.joint_limit_rejects);
        diagnostics[prefix + ".obb_zonotope_degenerate_rejects"] +=
            static_cast<double>(obb_stats.degenerate_rejects);
        diagnostics[prefix + ".obb_zonotope_aabb_tests"] +=
            static_cast<double>(obb_stats.aabb_tests);
        diagnostics[prefix + ".obb_zonotope_aabb_rejects"] +=
            static_cast<double>(obb_stats.aabb_rejects);
        diagnostics[prefix + ".obb_zonotope_gjk_tests"] +=
            static_cast<double>(obb_stats.gjk_tests);
        diagnostics[prefix + ".obb_zonotope_gjk_rejects"] +=
            static_cast<double>(obb_stats.gjk_rejects);
        diagnostics[prefix + ".obb_zonotope_gjk_iterations"] +=
            static_cast<double>(obb_stats.gjk_iterations);
        diagnostics[prefix + ".obb_zonotope_maybe_pairs"] +=
            static_cast<double>(obb_stats.maybe_pairs);
        diagnostics[prefix + ".obb_zonotope_waypoints"] +=
            static_cast<double>(obb_path.size());
        const double obb_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - obb_t0).count();
        diagnostics[prefix + ".obb_zonotope_ms"] += obb_ms;
        if (obb_ok) {
            const int edge_id = append_certified_portal_corridor_edge(
                segment_edges_,
                source_box,
                target_box,
                std::move(obb_path),
                SegmentEdgeValidation::ConservativeObbZonotope,
                -1,
                query_index,
                &obb_center,
                &obb_generators,
                SegmentEdgeType::TransitionOBBCorridor);
            if (edge_id >= 0) {
                const std::string edge_prefix = prefix + ".partition_native_obb_zonotope_portal";
                sync_adaptive_partition_segment_edges(out_profile, edge_prefix.c_str());
                diagnostics[prefix + ".obb_zonotope_success"] += 1.0;
                diagnostics[prefix + ".portal_corridor_added"] += 1.0;
                diagnostics[prefix + ".portal_corridor_obb_zonotope_added"] += 1.0;
                invalidate_query_cache();
                return anchors_added + 1;
            }
            diagnostics[prefix + ".obb_zonotope_edge_fail"] += 1.0;
        } else {
            diagnostics[prefix + ".obb_zonotope_fail"] += 1.0;
        }
    }

    const int max_internal_boxes = online_portal
        ? std::max(0, last_adaptive_partition_config_.hipac_online_max_hidden_boxes_per_portal)
        : std::max(0, last_adaptive_partition_config_.hipac_portal_max_internal_boxes);
    const int max_recursion_depth =
        std::max(0, last_adaptive_partition_config_.hipac_portal_max_recursion_depth);
    if (max_internal_boxes <= 0) {
        diagnostics[prefix + ".portal_corridor_internal_cap_zero"] += 1.0;
        return anchors_added;
    }
    const int requested_depth = last_adaptive_partition_config_.hipac_portal_ffb_depth > 0
        ? last_adaptive_partition_config_.hipac_portal_ffb_depth
        : std::max({config_.query_bridge_pave_depth,
                    config_.connector.pave.find_free_box.max_depth,
                    last_adaptive_partition_config_.target_max_depth});

    StageContext context = StageContext::from_runtime(config_.runtime);
    const double tol = config_.query.adjacency_tolerance;
    std::vector<BoxNode> internal_boxes;
    internal_boxes.reserve(static_cast<std::size_t>(std::min(max_internal_boxes, 32)));
    int next_internal_id = -1000000;
    bool chain_ok = false;

    auto append_portal_if_ready = [&]() -> int {
        if (!chain_ok || internal_boxes.empty()) {
            diagnostics[prefix + ".portal_corridor_chain_fail"] += 1.0;
            return anchors_added;
        }

        const int edge_id = append_portal_corridor_edge(segment_edges_,
                                                        source_box,
                                                        target_box,
                                                        std::move(internal_boxes),
                                                        -1,
                                                        tol,
                                                        query_index);
        if (edge_id < 0) {
            diagnostics[prefix + ".portal_corridor_edge_fail"] += 1.0;
            return anchors_added;
        }

        const std::string edge_prefix = prefix + ".partition_native_portal";
        sync_adaptive_partition_segment_edges(out_profile, edge_prefix.c_str());
        diagnostics[prefix + ".portal_corridor_added"] += 1.0;
        invalidate_query_cache();
        return anchors_added + 1;
    };

    if (last_adaptive_partition_config_.hipac_portal_cell_native_validate) {
        chain_ok = build_cell_native_portal_corridor_chain(source_box,
                                                           target_box,
                                                           waypoint_path,
                                                           domain,
                                                           prefix,
                                                           requested_depth,
                                                           max_internal_boxes,
                                                           max_recursion_depth,
                                                           tol,
                                                           out_profile,
                                                           internal_boxes,
                                                           next_internal_id);
        if (chain_ok) {
            return append_portal_if_ready();
        }
        internal_boxes.clear();
    }

    const bool allow_ffb_resolver =
        !last_adaptive_partition_config_.hipac_portal_cell_native_validate;
    if (!allow_ffb_resolver) {
        return anchors_added;
    }

    chain_ok = build_ffb_portal_corridor_chain(source_box,
                                               target_box,
                                               waypoint_path,
                                               domain,
                                               prefix,
                                               requested_depth,
                                               max_internal_boxes,
                                               max_recursion_depth,
                                               tol,
                                               online_portal,
                                               out_profile,
                                               context,
                                               internal_boxes,
                                               next_internal_id);
    return append_portal_if_ready();
}

} // namespace rbf
