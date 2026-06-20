#include <SBF/safe_box_forest.h>

#include <algorithm>
#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include "planning_forest_audit.h"
#include "planning_forest_diagnostics.h"
#include "planning_forest_obb.h"
#include "planning_forest_obb_diagnostics.h"
#include "planning_forest_query_utils.h"

namespace rbf {

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
        last_adaptive_partition_config_.segment_edge_obb_metadata_only;
    const bool obb_metadata_require_cover =
        obb_metadata_only &&
        last_adaptive_partition_config_.segment_edge_obb_metadata_require_cover;
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
            const ObbValidationOptions obb_validation_options =
                obb_validation_options_from_config(last_adaptive_partition_config_);
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
                obb_centerline,
                obb_validation_options);
            record_obb_path_cover_diagnostics(diagnostics, prefix + "." + obb_diag, cover, waypoints);
            diagnostics[prefix + "." + obb_diag + "_ms"] +=
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - obb_t0).count();
            auto try_clearance_retry_obb_edge = [&]() -> int {
                return try_add_clearance_retry_obb_edge(source_box_id,
                                                        target_box_id,
                                                        *source_ptr,
                                                        *target_ptr,
                                                        waypoints,
                                                        obb_validation_options,
                                                        obb_safety_epsilon,
                                                        prefix,
                                                        obb_diag,
                                                        query_index,
                                                        out_profile,
                                                        cover);
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
