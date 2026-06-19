#include <SBF/safe_box_forest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "planning_forest_audit.h"
#include "planning_forest_diagnostics.h"
#include "planning_forest_obb.h"
#include "planning_forest_obb_options.h"
#include "planning_forest_query_bridge_rrt_utils.h"
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
        obb_metadata_only_from_env();
    const bool obb_metadata_require_cover =
        obb_metadata_only &&
        obb_metadata_only_require_cover_from_env();
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
                const int retry_attempts = obb_clearance_retry_attempts_from_env();
                if (retry_attempts <= 0) {
                    return -1;
                }
                std::vector<double> clearances = obb_clearance_retry_values_from_env();
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
                    obb_clearance_retry_iters_from_env(std::max(1, retry_config.max_iters)));
                retry_config.timeout_ms = std::max(
                    0.0,
                    obb_clearance_retry_timeout_ms_from_env(retry_config.timeout_ms));
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
