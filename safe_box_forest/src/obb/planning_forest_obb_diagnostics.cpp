#include "planning_forest_obb_diagnostics.h"

#include <SBF/runtime.h>

#include <algorithm>
#include <cmath>

namespace rbf {

void record_obb_path_cover_diagnostics(std::unordered_map<std::string, double>& diagnostics,
                                       const std::string& key_prefix,
                                       const ObbPathCoverResult& cover,
                                       const std::vector<Eigen::VectorXd>& fallback_waypoints) {
    const ObbPortalValidationStats& stats = cover.stats;
    diagnostics[key_prefix + "_windows_attempted"] += static_cast<double>(cover.windows_attempted);
    diagnostics[key_prefix + "_windows_success"] += static_cast<double>(cover.windows_success);
    diagnostics[key_prefix + "_regions"] += static_cast<double>(cover.regions.size());
    diagnostics[key_prefix + "_recursive_splits"] += static_cast<double>(cover.recursive_splits);
    diagnostics[key_prefix + "_failed_leaf_windows"] += static_cast<double>(cover.failed_leaf_windows);
    diagnostics[key_prefix + "_failed_leaf_length_sum"] += cover.failed_leaf_length_sum;
    diagnostics[key_prefix + "_failed_leaf_length_max"] =
        std::max(diagnostics[key_prefix + "_failed_leaf_length_max"], cover.failed_leaf_length_max);
    if ((cover.has_first_failed_leaf || cover.failed_leaf_windows > 0) &&
        diagnostics[key_prefix + "_first_failed_leaf_recorded"] <= 0.0 &&
        !fallback_waypoints.empty()) {
        const Eigen::VectorXd& failed_a =
            cover.has_first_failed_leaf ? cover.first_failed_leaf_a : fallback_waypoints.front();
        const Eigen::VectorXd& failed_b =
            cover.has_first_failed_leaf ? cover.first_failed_leaf_b : fallback_waypoints.back();
        diagnostics[key_prefix + "_first_failed_leaf_recorded"] = 1.0;
        diagnostics[key_prefix + "_first_failed_leaf_length"] = (failed_b - failed_a).norm();
        diagnostics[key_prefix + "_first_failed_leaf_exact"] = cover.has_first_failed_leaf ? 1.0 : 0.0;
        const int dims = static_cast<int>(failed_a.size());
        diagnostics[key_prefix + "_first_failed_leaf_dims"] = static_cast<double>(dims);
        for (int dim = 0; dim < dims; ++dim) {
            diagnostics[key_prefix + "_first_failed_leaf_a_" + std::to_string(dim)] = failed_a[dim];
            diagnostics[key_prefix + "_first_failed_leaf_b_" + std::to_string(dim)] = failed_b[dim];
        }
    }
    diagnostics[key_prefix + "_candidates"] += static_cast<double>(stats.candidates);
    diagnostics[key_prefix + "_validations"] += static_cast<double>(stats.validations);
    diagnostics[key_prefix + "_valid_candidates"] += static_cast<double>(stats.valid_candidates);
    diagnostics[key_prefix + "_grow_attempts"] += static_cast<double>(stats.grow_attempts);
    diagnostics[key_prefix + "_joint_limit_rejects"] += static_cast<double>(stats.joint_limit_rejects);
    diagnostics[key_prefix + "_gjk_tests"] += static_cast<double>(stats.gjk_tests);
    diagnostics[key_prefix + "_maybe_pairs"] += static_cast<double>(stats.maybe_pairs);
    diagnostics[key_prefix + "_sampled_support_attempts"] +=
        static_cast<double>(stats.sampled_support_attempts);
    diagnostics[key_prefix + "_sampled_support_success"] +=
        static_cast<double>(stats.sampled_support_success);
    diagnostics[key_prefix + "_sampled_support_fail"] +=
        static_cast<double>(stats.sampled_support_fail);
    diagnostics[key_prefix + "_sampled_support_samples"] +=
        static_cast<double>(stats.sampled_support_samples);
    diagnostics[key_prefix + "_sampled_support_error_radius"] =
        std::max(diagnostics[key_prefix + "_sampled_support_error_radius"],
                 stats.sampled_support_error_radius);
    diagnostics[key_prefix + "_clearance_support_attempts"] +=
        static_cast<double>(stats.clearance_support_attempts);
    diagnostics[key_prefix + "_clearance_support_success"] +=
        static_cast<double>(stats.clearance_support_success);
    diagnostics[key_prefix + "_clearance_support_fail"] +=
        static_cast<double>(stats.clearance_support_fail);
    diagnostics[key_prefix + "_clearance_support_samples"] +=
        static_cast<double>(stats.clearance_support_samples);
    diagnostics[key_prefix + "_clearance_support_error_radius"] =
        std::max(diagnostics[key_prefix + "_clearance_support_error_radius"],
                 stats.clearance_support_error_radius);
    if (std::isfinite(stats.clearance_support_min_margin)) {
        const std::string margin_key = key_prefix + "_clearance_support_min_margin";
        const auto margin_it = diagnostics.find(margin_key);
        diagnostics[margin_key] = margin_it == diagnostics.end()
            ? stats.clearance_support_min_margin
            : std::min(margin_it->second, stats.clearance_support_min_margin);
    }
    diagnostics[key_prefix + "_longitudinal_radius"] =
        std::max(diagnostics[key_prefix + "_longitudinal_radius"], stats.longitudinal_radius);
    diagnostics[key_prefix + "_lateral_radius"] =
        std::max(diagnostics[key_prefix + "_lateral_radius"], stats.lateral_radius);
    diagnostics[key_prefix + "_region_volume_sum"] += stats.region_volume_sum;
    diagnostics[key_prefix + "_region_volume_max"] =
        std::max(diagnostics[key_prefix + "_region_volume_max"], stats.region_volume_max);
    diagnostics[key_prefix + "_region_log_volume_sum"] += stats.region_log_volume_sum;
    diagnostics[key_prefix + "_region_volume_count"] += static_cast<double>(stats.region_volume_count);
}

}  // namespace rbf
