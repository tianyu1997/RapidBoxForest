#include <SBF/safe_box_forest.h>

#include <SBF/connector.h>
#include <SBF/oracle.h>

#include <algorithm>
#include <chrono>
#include <string>
#include <vector>

#include "planning_forest_audit.h"
#include "planning_forest_obb.h"
#include "planning_forest_query_bridge_rrt_utils.h"
#include "planning_forest_query_utils.h"

namespace rbf {

int RBFPlanningForest::try_add_clearance_retry_obb_edge(
    int source_box_id,
    int target_box_id,
    const BoxNode& source_box,
    const BoxNode& target_box,
    const std::vector<Eigen::VectorXd>& waypoints,
    const ObbValidationOptions& obb_validation_options,
    double obb_safety_epsilon,
    const std::string& prefix,
    const std::string& obb_diag,
    int query_index,
    BuildProfile* profile,
    ObbPathCoverResult& cover) {
    if (waypoints.size() < 2U) {
        return -1;
    }
    const int retry_attempts = std::max(0, obb_validation_options.clearance_retry_attempts);
    if (retry_attempts <= 0) {
        return -1;
    }
    std::vector<double> clearances = obb_validation_options.clearance_retry_values;
    if (clearances.empty()) {
        return -1;
    }

    BuildProfile* out_profile = profile != nullptr ? profile : &last_build_;
    auto& diagnostics = out_profile->diagnostics;
    auto audit_obb_edge_path = [&](const std::vector<Eigen::VectorXd>& candidate_path,
                                   const char* label) -> bool {
        diagnostics[prefix + "." + obb_diag + "_centerline_audit_attempts"] += 1.0;
        if (candidate_path.size() < 2U) {
            diagnostics[prefix + "." + obb_diag + "_centerline_audit_empty"] += 1.0;
            return false;
        }
        const auto audit_t0 = std::chrono::steady_clock::now();
        CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
        const PathAuditCheck centerline_audit =
            audit_waypoint_path(candidate_path,
                                checker,
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

    CollisionChecker final_checker = make_audit_checker(audit_robot_, scene_, config_.query);
    RRTConnectConfig retry_config = config_.connector.rrt;
    retry_config.segment_resolution =
        std::max(retry_config.segment_resolution, config_.query.audit_resolution);
    retry_config.segment_step = config_.query.audit_segment_step;
    retry_config.max_iters = std::max(
        1,
        obb_validation_options.clearance_retry_iters >= 0
            ? obb_validation_options.clearance_retry_iters
            : std::max(1, retry_config.max_iters));
    retry_config.timeout_ms = std::max(
        0.0,
        obb_validation_options.clearance_retry_timeout_ms >= 0.0
            ? obb_validation_options.clearance_retry_timeout_ms
            : retry_config.timeout_ms);
    diagnostics[prefix + "." + obb_diag + "_clearance_retry_attempt_budget"] +=
        static_cast<double>(retry_attempts);
    for (int attempt = 0; attempt < retry_attempts; ++attempt) {
        const double clearance =
            std::max(0.0, clearances[static_cast<std::size_t>(attempt) % clearances.size()]);
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
            retry_centerline,
            obb_validation_options);
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
            source_box,
            target_box,
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
            if (partition_native_mode()) {
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
}

}  // namespace rbf
