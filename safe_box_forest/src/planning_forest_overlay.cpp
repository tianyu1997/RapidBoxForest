#include <SBF/safe_box_forest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "planning_forest_audit.h"
#include "planning_forest_diagnostics.h"
#include "planning_forest_obb.h"
#include "planning_forest_obb_options.h"
#include "planning_forest_query_utils.h"

namespace rbf {

namespace {

ObbValidationOptions obb_validation_options_from_config(const AdaptiveLeafSweepConfig& config) {
    ObbValidationOptions options;
    options.fast_primary_orientation = config.obb_fast_primary_orientation;
    options.fallback_orientations_on_primary_fail =
        config.obb_fallback_orientations_on_primary_fail;
    return options;
}

}  // namespace

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

    auto build_cell_native_chain = [&]() -> bool {
        const auto& split_descriptor = oracle_->database().split_policy_descriptor();
        const int dims = oracle_->n_dims();
        const int max_cell_depth =
            std::max(1, std::min({requested_depth,
                                  config_.database.max_tree_depth,
                                  oracle_->max_tree_depth() - 1,
                                  60}));
        const int selected_build_depth =
            out_profile != nullptr
                ? static_cast<int>(std::round(
                      diagnostic_map_value(out_profile->diagnostics, "adaptive.selected_leaf_depth")))
                : 0;
        const int min_cell_depth = std::max(
            1,
            std::min(max_cell_depth,
                     selected_build_depth > 0
                         ? selected_build_depth
                         : std::max(1, last_adaptive_partition_config_.shallow_max_depth)));
        std::vector<int> candidate_depths;
        for (int depth = min_cell_depth; depth < max_cell_depth; depth += 4) {
            candidate_depths.push_back(depth);
        }
        if (candidate_depths.empty() || candidate_depths.back() != max_cell_depth) {
            candidate_depths.push_back(max_cell_depth);
        }
        struct NativeCellCacheEntry {
            bool free = false;
            OracleNodeId node = kInvalidOracleNodeId;
            std::vector<Interval> intervals;
            BoxSafetyStatus safety_status = BoxSafetyStatus::Unknown;
            bool strict_audit_required = false;
        };
        std::unordered_map<std::string, NativeCellCacheEntry> cell_cache;
        cell_cache.reserve(static_cast<std::size_t>(max_internal_boxes * 4 + 16));
        int cell_validations = 0;
        int cell_free = 0;
        int cell_not_free = 0;
        int cell_invalid = 0;
        int cell_cache_hits = 0;
        int non_adjacent = 0;
        int recursion_splits = 0;
        int internal_cap_hits = 0;

        auto cache_key = [](OracleNodeId node, const std::vector<Interval>& intervals) {
            return std::to_string(node) + ":" +
                   std::to_string(lect_database::fingerprint_intervals(intervals));
        };

        auto classify_cell_at_point_at_depth = [&](const Eigen::VectorXd& point,
                                                   int cell_depth,
                                                   BoxNode& candidate) -> bool {
            if (point.size() != dims ||
                !oracle_->contains_point(oracle_->root_node(), point) ||
                !intervals_contain_point_strict_local(domain, point, 1e-12)) {
                ++cell_invalid;
                return false;
            }
            Eigen::VectorXd tree_seed = oracle_->tree_configuration_for_query(point);
            if (tree_seed.size() != dims) {
                ++cell_invalid;
                return false;
            }
            OracleNodeId node = oracle_->root_node();
            std::vector<Interval> tree_intervals = oracle_->node_intervals(node);
            int changed_dim = -1;
            for (int level = 0; level < cell_depth; ++level) {
                int split_dim = -1;
                if (!split_descriptor.depth_dimensions.empty() &&
                    level < static_cast<int>(split_descriptor.depth_dimensions.size())) {
                    split_dim = split_descriptor.depth_dimensions[static_cast<std::size_t>(level)];
                } else {
                    split_dim = level % dims;
                }
                if (split_dim < 0 ||
                    split_dim >= dims ||
                    split_dim >= static_cast<int>(tree_intervals.size())) {
                    ++cell_invalid;
                    return false;
                }
                auto& interval = tree_intervals[static_cast<std::size_t>(split_dim)];
                const double split_value = interval.center();
                if (!(split_value > interval.lo && split_value < interval.hi)) {
                    ++cell_invalid;
                    return false;
                }
                const bool right_child = tree_seed[split_dim] > split_value;
                if (node > (std::numeric_limits<OracleNodeId>::max() - 2) / 2) {
                    ++cell_invalid;
                    return false;
                }
                node = static_cast<OracleNodeId>(2 * node + (right_child ? 2 : 1));
                if (right_child) {
                    interval.lo = split_value;
                } else {
                    interval.hi = split_value;
                }
                changed_dim = split_dim;
            }
            std::vector<Interval> native_intervals =
                oracle_->query_intervals_for_node(node, tree_intervals, point);
            if (native_intervals.size() != static_cast<std::size_t>(dims) ||
                !intervals_contain_point_strict_local(native_intervals, point, std::max(1e-12, tol))) {
                ++cell_invalid;
                return false;
            }

            const std::string key = cache_key(node, native_intervals);
            const auto cache_it = cell_cache.find(key);
            if (cache_it != cell_cache.end()) {
                ++cell_cache_hits;
                if (!cache_it->second.free) {
                    return false;
                }
                candidate = adaptive_make_box_from_intervals(cache_it->second.intervals,
                                                             cache_it->second.node,
                                                             next_internal_id--,
                                                             cache_it->second.safety_status,
                                                             cache_it->second.strict_audit_required);
                candidate.seed_config = point;
                return true;
            }

            ++cell_validations;
            const BoxValidation validation = oracle_->validate_node(node, native_intervals, changed_dim);
            const OracleValidationDetail detail = oracle_->last_validation_detail();
            NativeCellCacheEntry entry;
            entry.free = validation == BoxValidation::Free &&
                         detail.safety_status == BoxSafetyStatus::CertifiedFree &&
                         !detail.strict_audit_required;
            entry.node = node;
            entry.intervals = native_intervals;
            entry.safety_status = detail.safety_status;
            entry.strict_audit_required = detail.strict_audit_required;
            cell_cache.emplace(key, entry);
            if (!entry.free) {
                ++cell_not_free;
                return false;
            }
            ++cell_free;
            candidate = adaptive_make_box_from_intervals(native_intervals,
                                                         node,
                                                         next_internal_id--,
                                                         detail.safety_status,
                                                         detail.strict_audit_required);
            candidate.seed_config = point;
            return true;
        };
        auto classify_cell_at_point = [&](const Eigen::VectorXd& point,
                                          BoxNode& candidate) -> bool {
            for (int depth : candidate_depths) {
                if (classify_cell_at_point_at_depth(point, depth, candidate)) {
                    return true;
                }
            }
            return false;
        };

        auto boundary_seed_from_box = [&](const BoxNode& box,
                                          const Eigen::VectorXd& from,
                                          const Eigen::VectorXd& to) {
            if (box.n_dims() != from.size() || to.size() != from.size()) {
                return to;
            }
            const Eigen::VectorXd delta = to - from;
            const double norm = delta.norm();
            if (norm <= 1e-12) {
                return to;
            }
            double exit_param = 1.0;
            for (int dim = 0; dim < from.size(); ++dim) {
                const double d = delta[dim];
                if (std::abs(d) < 1e-15) {
                    continue;
                }
                const auto& interval = box.joint_intervals[static_cast<std::size_t>(dim)];
                const double boundary = d > 0.0 ? interval.hi : interval.lo;
                const double t = (boundary - from[dim]) / d;
                if (t > 1e-12 && t < exit_param) {
                    exit_param = t;
                }
            }
            const double face_epsilon = std::max(16.0 * std::max(0.0, tol), 1e-6);
            Eigen::VectorXd seed = from + std::clamp(exit_param, 0.0, 1.0) * delta +
                                   face_epsilon * (delta / norm);
            for (int dim = 0; dim < seed.size() &&
                              dim < static_cast<int>(domain.size()); ++dim) {
                seed[dim] = std::min(domain[static_cast<std::size_t>(dim)].hi,
                                     std::max(domain[static_cast<std::size_t>(dim)].lo,
                                              seed[dim]));
            }
            return seed;
        };

        std::function<bool(BoxNode&, const Eigen::VectorXd&, const Eigen::VectorXd&, int)> connect_to_point;
        connect_to_point = [&](BoxNode& current,
                               const Eigen::VectorXd& from,
                               const Eigen::VectorXd& to,
                               int depth) -> bool {
            if (current.contains(to, tol)) {
                return true;
            }
            if (target_box.contains(to, tol) && boxes_connected(current, target_box, tol)) {
                return true;
            }
            if (static_cast<int>(internal_boxes.size()) >= max_internal_boxes) {
                ++internal_cap_hits;
                return false;
            }
            const Eigen::VectorXd seed = boundary_seed_from_box(current, from, to);
            BoxNode candidate;
            if (classify_cell_at_point(seed, candidate)) {
                if (candidate.safety_status == BoxSafetyStatus::CertifiedFree &&
                    !candidate.strict_audit_required &&
                    boxes_connected(current, candidate, tol)) {
                    internal_boxes.push_back(candidate);
                    current = internal_boxes.back();
                    if (current.contains(to, tol) ||
                        (target_box.contains(to, tol) && boxes_connected(current, target_box, tol))) {
                        return true;
                    }
                    if ((seed - from).norm() <= 1e-12) {
                        return false;
                    }
                    return connect_to_point(current, seed, to, depth);
                }
                ++non_adjacent;
            }
            if (depth <= 0 || from.size() != to.size()) {
                return false;
            }
            ++recursion_splits;
            const Eigen::VectorXd midpoint = 0.5 * (from + to);
            if (!connect_to_point(current, from, midpoint, depth - 1)) {
                return false;
            }
            return connect_to_point(current, midpoint, to, depth - 1);
        };

        bool ok = true;
        BoxNode current = source_box;
        Eigen::VectorXd previous = source_box.center();
        for (const auto& waypoint : waypoint_path) {
            if (waypoint.size() != previous.size()) {
                ok = false;
                break;
            }
        }
        if (ok) {
            for (std::size_t index = 1; index < waypoint_path.size(); ++index) {
                if (!connect_to_point(current, previous, waypoint_path[index], max_recursion_depth)) {
                    ok = false;
                    break;
                }
                previous = waypoint_path[index];
            }
        }
        if (ok && !boxes_connected(current, target_box, tol)) {
            ok = connect_to_point(current, previous, target_box.center(), max_recursion_depth) &&
                 boxes_connected(current, target_box, tol);
        }

        diagnostics[prefix + ".portal_corridor_cell_native_min_depth"] =
            static_cast<double>(min_cell_depth);
        diagnostics[prefix + ".portal_corridor_cell_native_max_depth"] =
            static_cast<double>(max_cell_depth);
        diagnostics[prefix + ".portal_corridor_cell_native_depth_candidates"] =
            static_cast<double>(candidate_depths.size());
        diagnostics[prefix + ".portal_corridor_cell_native_validations"] +=
            static_cast<double>(cell_validations);
        diagnostics[prefix + ".portal_corridor_cell_native_free"] += static_cast<double>(cell_free);
        diagnostics[prefix + ".portal_corridor_cell_native_not_free"] +=
            static_cast<double>(cell_not_free);
        diagnostics[prefix + ".portal_corridor_cell_native_invalid"] +=
            static_cast<double>(cell_invalid);
        diagnostics[prefix + ".portal_corridor_cell_native_cache_hits"] +=
            static_cast<double>(cell_cache_hits);
        diagnostics[prefix + ".portal_corridor_cell_native_non_adjacent"] +=
            static_cast<double>(non_adjacent);
        diagnostics[prefix + ".portal_corridor_cell_native_recursion_splits"] +=
            static_cast<double>(recursion_splits);
        diagnostics[prefix + ".portal_corridor_cell_native_internal_cap_hit"] +=
            static_cast<double>(internal_cap_hits);
        diagnostics[prefix + ".portal_corridor_cell_native_internal_boxes"] +=
            static_cast<double>(internal_boxes.size());
        diagnostics[prefix + ".portal_corridor_internal_boxes"] +=
            static_cast<double>(internal_boxes.size());
        if (ok) {
            diagnostics[prefix + ".portal_corridor_cell_native_success"] += 1.0;
        } else {
            diagnostics[prefix + ".portal_corridor_cell_native_fail"] += 1.0;
        }
        return ok;
    };

    if (last_adaptive_partition_config_.hipac_portal_cell_native_validate) {
        chain_ok = build_cell_native_chain();
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

    FindFreeBoxOptions ffb_options = config_.connector.pave.find_free_box;
    ffb_options.max_depth = std::max(1, std::min(requested_depth, config_.database.max_tree_depth));
    ffb_options.skip_existing_cover_check = true;
    ffb_options.reject_seed_collision = false;
    ffb_options.deadline_ms =
        std::max(0.0, last_adaptive_partition_config_.hipac_portal_ffb_deadline_ms);
    const int max_ffb_calls = online_portal
        ? std::max(0, last_adaptive_partition_config_.hipac_online_max_ffb_calls_per_portal)
        : -1;
    int ffb_calls = 0;
    int ffb_success = 0;
    int ffb_fail = 0;
    int non_adjacent = 0;
    int non_certified = 0;
    int recursion_splits = 0;

    std::function<bool(BoxNode&, const Eigen::VectorXd&, const Eigen::VectorXd&, int)> connect_to_point_ffb;
    connect_to_point_ffb = [&](BoxNode& current,
                               const Eigen::VectorXd& from,
                               const Eigen::VectorXd& to,
                               int depth) -> bool {
        if (current.contains(to, tol)) {
            return true;
        }
        if (target_box.contains(to, tol) && boxes_connected(current, target_box, tol)) {
            return true;
        }
        if (static_cast<int>(internal_boxes.size()) >= max_internal_boxes) {
            diagnostics[prefix + ".portal_corridor_internal_cap_hit"] += 1.0;
            return false;
        }
        if (max_ffb_calls >= 0 && ffb_calls >= max_ffb_calls) {
            diagnostics[prefix + ".portal_corridor_ffb_cap_hit"] += 1.0;
            return false;
        }
        ++ffb_calls;
        FindFreeBoxResult found = find_free_box_in_domain(to, domain, context, ffb_options);
        if (found.found) {
            BoxNode candidate = adaptive_make_box_from_intervals(found.intervals,
                                                                 found.node,
                                                                 next_internal_id--,
                                                                 BoxSafetyStatus::CertifiedFree,
                                                                 false);
            candidate.seed_config = to;
            if (candidate.safety_status != BoxSafetyStatus::CertifiedFree ||
                candidate.strict_audit_required) {
                ++non_certified;
            } else if (boxes_connected(current, candidate, tol)) {
                ++ffb_success;
                internal_boxes.push_back(candidate);
                current = internal_boxes.back();
                return true;
            } else {
                ++non_adjacent;
            }
        } else {
            ++ffb_fail;
        }
        if (depth <= 0 || from.size() != to.size()) {
            return false;
        }
        ++recursion_splits;
        const Eigen::VectorXd midpoint = 0.5 * (from + to);
        if (!connect_to_point_ffb(current, from, midpoint, depth - 1)) {
            return false;
        }
        return connect_to_point_ffb(current, midpoint, to, depth - 1);
    };

    chain_ok = true;
    {
        BoxNode current = source_box;
        Eigen::VectorXd previous = source_box.center();
        for (const auto& waypoint : waypoint_path) {
            if (waypoint.size() != previous.size()) {
                chain_ok = false;
                break;
            }
        }
        if (chain_ok) {
            for (std::size_t index = 1; index < waypoint_path.size(); ++index) {
                if (!connect_to_point_ffb(current, previous, waypoint_path[index], max_recursion_depth)) {
                    chain_ok = false;
                    break;
                }
                previous = waypoint_path[index];
            }
        }
        if (chain_ok && !boxes_connected(current, target_box, tol)) {
            chain_ok = connect_to_point_ffb(current, previous, target_box.center(), max_recursion_depth) &&
                       boxes_connected(current, target_box, tol);
        }
    }

    diagnostics[prefix + ".portal_corridor_ffb_calls"] += static_cast<double>(ffb_calls);
    diagnostics[prefix + ".portal_corridor_ffb_success"] += static_cast<double>(ffb_success);
    diagnostics[prefix + ".portal_corridor_ffb_fail"] += static_cast<double>(ffb_fail);
    diagnostics[prefix + ".portal_corridor_non_adjacent"] += static_cast<double>(non_adjacent);
    diagnostics[prefix + ".portal_corridor_non_certified"] += static_cast<double>(non_certified);
    diagnostics[prefix + ".portal_corridor_recursion_splits"] += static_cast<double>(recursion_splits);
    diagnostics[prefix + ".portal_corridor_internal_boxes"] +=
        static_cast<double>(internal_boxes.size());

    for (const auto& [key, value] : context.diagnostics().snapshot()) {
        diagnostics[prefix + ".portal_corridor_context." + key] += value;
    }

    return append_portal_if_ready();
}

} // namespace rbf
