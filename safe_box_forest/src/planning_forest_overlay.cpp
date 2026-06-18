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

#include "env_config.h"
#include "planning_forest_audit.h"
#include "planning_forest_diagnostics.h"
#include "planning_forest_obb.h"
#include "planning_forest_query_bridge_batch_utils.h"
#include "planning_forest_query_utils.h"

namespace rbf {

using detail::env_double_list_or_empty;
using detail::env_double_or_default;
using detail::env_int_or_default;

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
        prefix.find("hipac_online_transition") != std::string::npos ||
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
            &obb_generators);
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
        !last_adaptive_partition_config_.hipac_portal_cell_native_validate ||
        (online_portal && last_adaptive_partition_config_.hipac_online_ffb_portal_fallback);
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

int RBFPlanningForest::add_segment_edge_partition_first(
    int source_box_id,
    int target_box_id,
    std::vector<Eigen::VectorXd> waypoints,
    SegmentEdgeType type,
    int segment_resolution,
    SegmentEdgeValidation validation,
    bool strict_audit_required,
    int query_index,
    BuildProfile* profile,
    const char* diagnostic_prefix) {
    const bool use_partition_overlay = partition_native_mode();
    BuildProfile* out_profile = profile != nullptr ? profile : &last_build_;
    const std::string prefix =
        (diagnostic_prefix != nullptr && diagnostic_prefix[0] != '\0')
            ? std::string(diagnostic_prefix)
            : std::string("partition_segment_edge");
    auto& diagnostics = out_profile->diagnostics;

    const bool path_is_rrt_bridge_like =
        type == SegmentEdgeType::RRTConnector || waypoints.size() > 2U;
    std::vector<Eigen::VectorXd> partial_obb_centers;
    std::vector<Eigen::MatrixXd> partial_obb_generators;
    double partial_obb_covered_length = 0.0;
    std::string partial_obb_diag;
    const bool eligible_for_obb_cover =
        (last_adaptive_partition_config_.segment_edge_obb_cover ||
         (path_is_rrt_bridge_like && last_adaptive_partition_config_.rrt_bridge_obb_cover)) &&
        validation == SegmentEdgeValidation::CollisionChecked &&
        counts_as_segment_edge(type) &&
        waypoints.size() >= 2U &&
        oracle_ != nullptr;
    const bool strict_obb_bridge_cover =
        eligible_for_obb_cover && last_adaptive_partition_config_.strict_obb_bridge_cover;
    const bool obb_metadata_only =
        eligible_for_obb_cover &&
        !strict_obb_bridge_cover &&
        env_int_or_default("RBF_OBB_METADATA_ONLY", 0) != 0;
    const bool obb_metadata_require_cover =
        obb_metadata_only &&
        env_int_or_default("RBF_OBB_METADATA_ONLY_REQUIRE_COVER", 0) != 0;
    if (eligible_for_obb_cover) {
        const bool greedy_bridge_cover =
            path_is_rrt_bridge_like &&
            (last_adaptive_partition_config_.rrt_bridge_obb_cover ||
             last_adaptive_partition_config_.segment_edge_obb_cover);
        const std::string obb_diag = greedy_bridge_cover
            ? std::string("rrt_bridge_obb_cover")
            : std::string("segment_obb_cover");
        diagnostics[prefix + "." + obb_diag + "_attempts"] += 1.0;
        const BoxNode* source_ptr = find_box_by_id(boxes_, source_box_id);
        const BoxNode* target_ptr = find_box_by_id(boxes_, target_box_id);
        if (source_ptr != nullptr && target_ptr != nullptr) {
            std::optional<CollisionChecker> obb_audit_checker;
            auto audit_obb_edge_path = [&](const std::vector<Eigen::VectorXd>& candidate_path,
                                           const char* label) -> bool {
                diagnostics[prefix + "." + obb_diag + "_centerline_audit_attempts"] += 1.0;
                if (candidate_path.size() < 2U) {
                    diagnostics[prefix + "." + obb_diag + "_centerline_audit_empty"] += 1.0;
                    return false;
                }
                const auto audit_t0 = std::chrono::steady_clock::now();
                if (!obb_audit_checker.has_value()) {
                    obb_audit_checker.emplace(make_audit_checker(audit_robot_, scene_, config_.query));
                }
                const PathAuditCheck centerline_audit =
                    audit_waypoint_path(candidate_path,
                                        *obb_audit_checker,
                                        config_.query.audit_resolution,
                                        config_.query.audit_segment_step);
                diagnostics[prefix + "." + obb_diag + "_centerline_audit_ms"] +=
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - audit_t0).count();
                if (!centerline_audit.passed) {
                    diagnostics[prefix + "." + obb_diag + "_centerline_audit_rejects"] += 1.0;
                    diagnostics[prefix + "." + obb_diag + "_centerline_audit_failed_segment"] =
                        static_cast<double>(centerline_audit.failed_segment_index);
                    if (label != nullptr && label[0] != '\0') {
                        diagnostics[prefix + "." + obb_diag + "_centerline_audit_reject_" +
                                    std::string(label)] += 1.0;
                    }
                    return false;
                }
                diagnostics[prefix + "." + obb_diag + "_centerline_audit_pass"] += 1.0;
                return true;
            };
            auto obb_t0 = std::chrono::steady_clock::now();
            const double obb_safety_epsilon =
                std::max(0.0, last_adaptive_partition_config_.segment_edge_obb_safety_epsilon);
            std::vector<Eigen::VectorXd> obb_centerline;
            ObbPathCoverResult cover = cover_segment_or_bridge_path_with_obbs(
                robot_,
                scene_,
                oracle_->planning_intervals(),
                waypoints,
                greedy_bridge_cover,
                last_adaptive_partition_config_.segment_edge_obb_split_depth,
                last_adaptive_partition_config_.obb_max_window_segments,
                last_adaptive_partition_config_.segment_edge_obb_lateral_radius,
                last_adaptive_partition_config_.segment_edge_obb_longitudinal_margin,
                obb_safety_epsilon,
                last_adaptive_partition_config_.segment_edge_obb_grow_iterations,
                last_adaptive_partition_config_.segment_edge_obb_binary_iterations,
                last_adaptive_partition_config_.obb_max_validations_per_window,
                obb_centerline);
            const ObbPortalValidationStats& obb_stats = cover.stats;
            diagnostics[prefix + "." + obb_diag + "_windows_attempted"] +=
                static_cast<double>(cover.windows_attempted);
            diagnostics[prefix + "." + obb_diag + "_windows_success"] +=
                static_cast<double>(cover.windows_success);
            diagnostics[prefix + "." + obb_diag + "_regions"] +=
                static_cast<double>(cover.regions.size());
            diagnostics[prefix + "." + obb_diag + "_recursive_splits"] +=
                static_cast<double>(cover.recursive_splits);
            diagnostics[prefix + "." + obb_diag + "_failed_leaf_windows"] +=
                static_cast<double>(cover.failed_leaf_windows);
            diagnostics[prefix + "." + obb_diag + "_failed_leaf_length_sum"] +=
                cover.failed_leaf_length_sum;
            diagnostics[prefix + "." + obb_diag + "_failed_leaf_length_max"] =
                std::max(diagnostics[prefix + "." + obb_diag + "_failed_leaf_length_max"],
                         cover.failed_leaf_length_max);
            if ((cover.has_first_failed_leaf || cover.failed_leaf_windows > 0) &&
                diagnostics[prefix + "." + obb_diag + "_first_failed_leaf_recorded"] <= 0.0) {
                const Eigen::VectorXd& failed_a =
                    cover.has_first_failed_leaf ? cover.first_failed_leaf_a : waypoints.front();
                const Eigen::VectorXd& failed_b =
                    cover.has_first_failed_leaf ? cover.first_failed_leaf_b : waypoints.back();
                diagnostics[prefix + "." + obb_diag + "_first_failed_leaf_recorded"] = 1.0;
                diagnostics[prefix + "." + obb_diag + "_first_failed_leaf_length"] =
                    (failed_b - failed_a).norm();
                diagnostics[prefix + "." + obb_diag + "_first_failed_leaf_exact"] =
                    cover.has_first_failed_leaf ? 1.0 : 0.0;
                const int dims = static_cast<int>(failed_a.size());
                diagnostics[prefix + "." + obb_diag + "_first_failed_leaf_dims"] =
                    static_cast<double>(dims);
                for (int dim = 0; dim < dims; ++dim) {
                    diagnostics[prefix + "." + obb_diag + "_first_failed_leaf_a_" + std::to_string(dim)] =
                        failed_a[dim];
                    diagnostics[prefix + "." + obb_diag + "_first_failed_leaf_b_" + std::to_string(dim)] =
                        failed_b[dim];
                }
            }
            diagnostics[prefix + "." + obb_diag + "_candidates"] +=
                static_cast<double>(obb_stats.candidates);
            diagnostics[prefix + "." + obb_diag + "_validations"] +=
                static_cast<double>(obb_stats.validations);
            diagnostics[prefix + "." + obb_diag + "_valid_candidates"] +=
                static_cast<double>(obb_stats.valid_candidates);
            diagnostics[prefix + "." + obb_diag + "_grow_attempts"] +=
                static_cast<double>(obb_stats.grow_attempts);
            diagnostics[prefix + "." + obb_diag + "_joint_limit_rejects"] +=
                static_cast<double>(obb_stats.joint_limit_rejects);
            diagnostics[prefix + "." + obb_diag + "_gjk_tests"] +=
                static_cast<double>(obb_stats.gjk_tests);
            diagnostics[prefix + "." + obb_diag + "_maybe_pairs"] +=
                static_cast<double>(obb_stats.maybe_pairs);
            diagnostics[prefix + "." + obb_diag + "_sampled_support_attempts"] +=
                static_cast<double>(obb_stats.sampled_support_attempts);
            diagnostics[prefix + "." + obb_diag + "_sampled_support_success"] +=
                static_cast<double>(obb_stats.sampled_support_success);
            diagnostics[prefix + "." + obb_diag + "_sampled_support_fail"] +=
                static_cast<double>(obb_stats.sampled_support_fail);
            diagnostics[prefix + "." + obb_diag + "_sampled_support_samples"] +=
                static_cast<double>(obb_stats.sampled_support_samples);
            diagnostics[prefix + "." + obb_diag + "_sampled_support_error_radius"] =
                std::max(diagnostics[prefix + "." + obb_diag + "_sampled_support_error_radius"],
                         obb_stats.sampled_support_error_radius);
            diagnostics[prefix + "." + obb_diag + "_clearance_support_attempts"] +=
                static_cast<double>(obb_stats.clearance_support_attempts);
            diagnostics[prefix + "." + obb_diag + "_clearance_support_success"] +=
                static_cast<double>(obb_stats.clearance_support_success);
            diagnostics[prefix + "." + obb_diag + "_clearance_support_fail"] +=
                static_cast<double>(obb_stats.clearance_support_fail);
            diagnostics[prefix + "." + obb_diag + "_clearance_support_samples"] +=
                static_cast<double>(obb_stats.clearance_support_samples);
            diagnostics[prefix + "." + obb_diag + "_clearance_support_error_radius"] =
                std::max(diagnostics[prefix + "." + obb_diag + "_clearance_support_error_radius"],
                         obb_stats.clearance_support_error_radius);
            if (std::isfinite(obb_stats.clearance_support_min_margin)) {
                const std::string margin_key =
                    prefix + "." + obb_diag + "_clearance_support_min_margin";
                const auto margin_it = diagnostics.find(margin_key);
                diagnostics[margin_key] =
                    margin_it == diagnostics.end()
                        ? obb_stats.clearance_support_min_margin
                        : std::min(margin_it->second, obb_stats.clearance_support_min_margin);
            }
            diagnostics[prefix + "." + obb_diag + "_longitudinal_radius"] =
                std::max(diagnostics[prefix + "." + obb_diag + "_longitudinal_radius"],
                         obb_stats.longitudinal_radius);
            diagnostics[prefix + "." + obb_diag + "_lateral_radius"] =
                std::max(diagnostics[prefix + "." + obb_diag + "_lateral_radius"],
                         obb_stats.lateral_radius);
            diagnostics[prefix + "." + obb_diag + "_region_volume_sum"] +=
                obb_stats.region_volume_sum;
            diagnostics[prefix + "." + obb_diag + "_region_volume_max"] =
                std::max(diagnostics[prefix + "." + obb_diag + "_region_volume_max"],
                         obb_stats.region_volume_max);
            diagnostics[prefix + "." + obb_diag + "_region_log_volume_sum"] +=
                obb_stats.region_log_volume_sum;
            diagnostics[prefix + "." + obb_diag + "_region_volume_count"] +=
                static_cast<double>(obb_stats.region_volume_count);
            diagnostics[prefix + "." + obb_diag + "_ms"] +=
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - obb_t0).count();
            auto try_clearance_retry_obb_edge = [&]() -> int {
                if (!greedy_bridge_cover || !strict_obb_bridge_cover || waypoints.size() < 2U) {
                    return -1;
                }
                const int retry_attempts =
                    std::max(0, env_int_or_default("RBF_OBB_CLEARANCE_RETRY_ATTEMPTS", 0));
                if (retry_attempts <= 0) {
                    return -1;
                }
                std::vector<double> clearances =
                    env_double_list_or_empty("RBF_OBB_CLEARANCE_RETRY_VALUES");
                if (clearances.empty()) {
                    const double fallback_clearance = query_bridge_rrt_clearance_from_env();
                    if (fallback_clearance > 0.0) {
                        clearances.push_back(fallback_clearance);
                    }
                }
                if (clearances.empty()) {
                    return -1;
                }
                CollisionChecker final_checker = make_audit_checker(audit_robot_, scene_, config_.query);
                RRTConnectConfig retry_config = config_.connector.rrt;
                retry_config.segment_resolution =
                    std::max(retry_config.segment_resolution, config_.query.audit_resolution);
                retry_config.segment_step = config_.query.audit_segment_step;
                retry_config.max_iters = std::max(
                    1,
                    env_int_or_default("RBF_OBB_CLEARANCE_RETRY_ITERS",
                                       std::max(1, retry_config.max_iters)));
                retry_config.timeout_ms = std::max(
                    0.0,
                    env_double_or_default("RBF_OBB_CLEARANCE_RETRY_TIMEOUT_MS",
                                          retry_config.timeout_ms));
                diagnostics[prefix + "." + obb_diag + "_clearance_retry_attempt_budget"] +=
                    static_cast<double>(retry_attempts);
                for (int attempt = 0; attempt < retry_attempts; ++attempt) {
                    const double clearance =
                        std::max(0.0, clearances[static_cast<std::size_t>(attempt) %
                                                  clearances.size()]);
                    if (!(clearance > 0.0)) {
                        continue;
                    }
                    diagnostics[prefix + "." + obb_diag + "_clearance_retry_attempts"] += 1.0;
                    Robot clearance_robot = make_sbf_clearance_robot(audit_robot_, clearance);
                    CollisionChecker clearance_checker(clearance_robot, scene_);
                    const int retry_seed = derived_planner_seed(
                        config_.grower.rng_seed,
                        kSeedQueryBridgeOffset,
                        attempt,
                        query_index < 0 ? 0 : query_index,
                        17017);
                    std::vector<Eigen::VectorXd> retry_path = rrt_connect(
                        waypoints.front(),
                        waypoints.back(),
                        clearance_checker,
                        clearance_robot,
                        retry_config,
                        retry_seed);
                    if (retry_path.empty()) {
                        diagnostics[prefix + "." + obb_diag + "_clearance_retry_no_path"] += 1.0;
                        continue;
                    }
                    const PathAuditCheck retry_audit =
                        audit_waypoint_path(retry_path,
                                            final_checker,
                                            config_.query.audit_resolution,
                                            config_.query.audit_segment_step);
                    if (!retry_audit.passed) {
                        diagnostics[prefix + "." + obb_diag + "_clearance_retry_audit_fail"] += 1.0;
                        continue;
                    }
                    std::vector<Eigen::VectorXd> retry_centerline;
                    ObbPathCoverResult retry_cover = cover_segment_or_bridge_path_with_obbs(
                        robot_,
                        scene_,
                        oracle_->planning_intervals(),
                        retry_path,
                        true,
                        last_adaptive_partition_config_.segment_edge_obb_split_depth,
                        last_adaptive_partition_config_.obb_max_window_segments,
                        last_adaptive_partition_config_.segment_edge_obb_lateral_radius,
                        last_adaptive_partition_config_.segment_edge_obb_longitudinal_margin,
                        obb_safety_epsilon,
                        last_adaptive_partition_config_.segment_edge_obb_grow_iterations,
                        last_adaptive_partition_config_.segment_edge_obb_binary_iterations,
                        last_adaptive_partition_config_.obb_max_validations_per_window,
                        retry_centerline);
                    obb_accumulate_stats(cover.stats, retry_cover.stats);
                    diagnostics[prefix + "." + obb_diag + "_clearance_retry_windows_attempted"] +=
                        static_cast<double>(retry_cover.windows_attempted);
                    diagnostics[prefix + "." + obb_diag + "_clearance_retry_windows_success"] +=
                        static_cast<double>(retry_cover.windows_success);
                    diagnostics[prefix + "." + obb_diag + "_clearance_retry_failed_leaf_windows"] +=
                        static_cast<double>(retry_cover.failed_leaf_windows);
                    diagnostics[prefix + "." + obb_diag + "_clearance_retry_failed_leaf_length_max"] =
                        std::max(diagnostics[prefix + "." + obb_diag +
                                             "_clearance_retry_failed_leaf_length_max"],
                                 retry_cover.failed_leaf_length_max);
                    if (!retry_cover.success || retry_cover.regions.empty()) {
                        diagnostics[prefix + "." + obb_diag + "_clearance_retry_cover_fail"] += 1.0;
                        continue;
                    }
                    std::vector<Eigen::VectorXd> obb_centers;
                    std::vector<Eigen::MatrixXd> obb_generators;
                    obb_centers.reserve(retry_cover.regions.size());
                    obb_generators.reserve(retry_cover.regions.size());
                    for (const auto& region : retry_cover.regions) {
                        obb_centers.push_back(region.center);
                        obb_generators.push_back(region.generators);
                    }
                    const std::vector<Eigen::VectorXd>& retry_edge_path =
                        retry_centerline.empty() ? retry_path : retry_centerline;
                    if (!audit_obb_edge_path(retry_edge_path, "clearance_retry")) {
                        diagnostics[prefix + "." + obb_diag +
                                    "_clearance_retry_centerline_audit_fail"] += 1.0;
                        continue;
                    }
                    const int retry_edge_id = append_certified_portal_corridor_edge(
                        segment_edges_,
                        *source_ptr,
                        *target_ptr,
                        retry_edge_path,
                        SegmentEdgeValidation::ConservativeObbZonotope,
                        -1,
                        query_index,
                        nullptr,
                        nullptr,
                        SegmentEdgeType::RRTBridgeOBBCorridor,
                        &obb_centers,
                        &obb_generators);
                    if (retry_edge_id >= 0) {
                        diagnostics[prefix + "." + obb_diag + "_clearance_retry_success"] += 1.0;
                        diagnostics[prefix + "." + obb_diag + "_success"] += 1.0;
                        diagnostics[prefix + "." + obb_diag + "_replaced_segments"] +=
                            static_cast<double>(std::max<std::size_t>(1U, retry_path.size() - 1U));
                        if (use_partition_overlay) {
                            sync_adaptive_partition_segment_edges(out_profile, prefix.c_str());
                        } else {
                            auto append_unique = [&](int a, int b) {
                                auto& neighbors = adjacency_[a];
                                if (std::find(neighbors.begin(), neighbors.end(), b) == neighbors.end()) {
                                    neighbors.push_back(b);
                                }
                            };
                            append_unique(source_box_id, target_box_id);
                            append_unique(target_box_id, source_box_id);
                            invalidate_query_cache();
                        }
                        return retry_edge_id;
                    }
                    diagnostics[prefix + "." + obb_diag + "_clearance_retry_edge_fail"] += 1.0;
                }
                return -1;
            };
            if (cover.success && !cover.regions.empty()) {
                std::vector<Eigen::VectorXd> obb_centers;
                std::vector<Eigen::MatrixXd> obb_generators;
                obb_centers.reserve(cover.regions.size());
                obb_generators.reserve(cover.regions.size());
                for (const auto& region : cover.regions) {
                    obb_centers.push_back(region.center);
                    obb_generators.push_back(region.generators);
                }
                const SegmentEdgeType obb_edge_type = greedy_bridge_cover
                    ? SegmentEdgeType::RRTBridgeOBBCorridor
                    : SegmentEdgeType::SegmentOBBCorridor;
                if (obb_metadata_only) {
                    partial_obb_diag = obb_diag;
                    partial_obb_covered_length = cover.covered_length;
                    partial_obb_centers = std::move(obb_centers);
                    partial_obb_generators = std::move(obb_generators);
                    diagnostics[prefix + "." + obb_diag + "_success"] += 1.0;
                    diagnostics[prefix + "." + obb_diag + "_metadata_only"] += 1.0;
                    diagnostics[prefix + "." + obb_diag + "_metadata_only_segments"] +=
                        static_cast<double>(std::max<std::size_t>(1U, waypoints.size() - 1U));
                } else {
                const std::vector<Eigen::VectorXd>& obb_edge_path =
                    obb_centerline.empty() ? waypoints : obb_centerline;
                if (!audit_obb_edge_path(obb_edge_path, "primary")) {
                    diagnostics[prefix + "." + obb_diag + "_centerline_audit_fail"] += 1.0;
                    if (strict_obb_bridge_cover) {
                        const int retry_edge_id = try_clearance_retry_obb_edge();
                        if (retry_edge_id >= 0) {
                            return retry_edge_id;
                        }
                        diagnostics[prefix + "." + obb_diag + "_strict_reject"] += 1.0;
                        return -1;
                    }
                } else {
                const int obb_edge_id = append_certified_portal_corridor_edge(
                    segment_edges_,
                    *source_ptr,
                    *target_ptr,
                    obb_edge_path,
                    SegmentEdgeValidation::ConservativeObbZonotope,
                    -1,
                    query_index,
                    nullptr,
                    nullptr,
                    obb_edge_type,
                    &obb_centers,
                    &obb_generators);
                if (obb_edge_id >= 0) {
                    diagnostics[prefix + "." + obb_diag + "_success"] += 1.0;
                    diagnostics[prefix + "." + obb_diag + "_replaced_segments"] +=
                        static_cast<double>(std::max<std::size_t>(1U, waypoints.size() - 1U));
                    if (use_partition_overlay) {
                        sync_adaptive_partition_segment_edges(out_profile, prefix.c_str());
                    } else {
                        auto append_unique = [&](int a, int b) {
                            auto& neighbors = adjacency_[a];
                            if (std::find(neighbors.begin(), neighbors.end(), b) == neighbors.end()) {
                                neighbors.push_back(b);
                            }
                        };
                        append_unique(source_box_id, target_box_id);
                        append_unique(target_box_id, source_box_id);
                        invalidate_query_cache();
                    }
                    return obb_edge_id;
                }
                diagnostics[prefix + "." + obb_diag + "_edge_fail"] += 1.0;
                if (strict_obb_bridge_cover) {
                    const int retry_edge_id = try_clearance_retry_obb_edge();
                    if (retry_edge_id >= 0) {
                        return retry_edge_id;
                    }
                    diagnostics[prefix + "." + obb_diag + "_strict_reject"] += 1.0;
                    return -1;
                }
                }
                }
            } else {
                diagnostics[prefix + "." + obb_diag + "_fail"] += 1.0;
                if (obb_metadata_require_cover) {
                    diagnostics[prefix + "." + obb_diag +
                                "_metadata_require_cover_reject"] += 1.0;
                    return -1;
                }
                if (!cover.regions.empty() && cover.covered_length > 0.0) {
                    partial_obb_diag = obb_diag;
                    partial_obb_covered_length = cover.covered_length;
                    partial_obb_centers.reserve(cover.regions.size());
                    partial_obb_generators.reserve(cover.regions.size());
                    for (const auto& region : cover.regions) {
                        partial_obb_centers.push_back(region.center);
                        partial_obb_generators.push_back(region.generators);
                    }
                    diagnostics[prefix + "." + obb_diag + "_partial_edges"] += 1.0;
                    diagnostics[prefix + "." + obb_diag + "_partial_regions"] +=
                        static_cast<double>(partial_obb_centers.size());
                    diagnostics[prefix + "." + obb_diag + "_partial_covered_length"] +=
                        partial_obb_covered_length;
                }
                if (strict_obb_bridge_cover) {
                    const int retry_edge_id = try_clearance_retry_obb_edge();
                    if (retry_edge_id >= 0) {
                        return retry_edge_id;
                    }
                    diagnostics[prefix + "." + obb_diag + "_strict_reject"] += 1.0;
                    return -1;
                }
            }
        } else {
            diagnostics[prefix + "." + obb_diag + "_missing_box"] += 1.0;
            if (obb_metadata_require_cover) {
                diagnostics[prefix + "." + obb_diag +
                            "_metadata_require_cover_reject"] += 1.0;
                return -1;
            }
            if (strict_obb_bridge_cover) {
                diagnostics[prefix + "." + obb_diag + "_strict_reject"] += 1.0;
                return -1;
            }
        }
    }

    const int edge_id = use_partition_overlay
        ? append_segment_edge(segment_edges_,
                              source_box_id,
                              target_box_id,
                              std::move(waypoints),
                              type,
                              segment_resolution,
                              validation,
                              strict_audit_required,
                              query_index)
        : add_segment_edge(segment_edges_,
                           adjacency_,
                           source_box_id,
                           target_box_id,
                           std::move(waypoints),
                           type,
                           segment_resolution,
                           validation,
                           strict_audit_required,
                           query_index);
    if (edge_id < 0) {
        return -1;
    }
    if (!partial_obb_centers.empty() && partial_obb_centers.size() == partial_obb_generators.size()) {
        auto edge_it = std::find_if(segment_edges_.begin(),
                                    segment_edges_.end(),
                                    [&](const SegmentEdge& edge) {
                                        return edge.id == edge_id;
                                    });
        if (edge_it != segment_edges_.end()) {
            edge_it->obb_centers = std::move(partial_obb_centers);
            edge_it->obb_generators = std::move(partial_obb_generators);
            edge_it->obb_covered_length =
                std::min(edge_it->length, std::max(0.0, partial_obb_covered_length));
            if (!partial_obb_diag.empty()) {
                diagnostics[prefix + "." + partial_obb_diag + "_partial_committed"] += 1.0;
            }
        }
    }
    if (use_partition_overlay) {
        sync_adaptive_partition_segment_edges(out_profile, prefix.c_str());
    } else {
        invalidate_query_cache();
    }
    return edge_id;
}


} // namespace rbf
