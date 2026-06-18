#pragma once

#include <SBF/safe_box_forest.h>

#include <Eigen/Core>

#include <limits>
#include <vector>

namespace rbf {

struct ObbPortalValidationStats {
    int joint_limit_rejects = 0;
    int degenerate_rejects = 0;
    int candidates = 0;
    int validations = 0;
    int valid_candidates = 0;
    int grow_attempts = 0;
    int aabb_tests = 0;
    int aabb_rejects = 0;
    int gjk_tests = 0;
    int gjk_rejects = 0;
    int gjk_iterations = 0;
    int maybe_pairs = 0;
    int sampled_support_attempts = 0;
    int sampled_support_success = 0;
    int sampled_support_fail = 0;
    int sampled_support_samples = 0;
    int clearance_support_attempts = 0;
    int clearance_support_success = 0;
    int clearance_support_fail = 0;
    int clearance_support_samples = 0;
    int active_links = 0;
    int variables = 0;
    double longitudinal_radius = 0.0;
    double lateral_radius = 0.0;
    double sampled_support_error_radius = 0.0;
    double clearance_support_min_margin = std::numeric_limits<double>::infinity();
    double clearance_support_error_radius = 0.0;
    double region_volume_sum = 0.0;
    double region_volume_max = 0.0;
    double region_log_volume_sum = 0.0;
    int region_volume_count = 0;
};

struct ObbPathCoverRegion {
    std::size_t begin = 0;
    std::size_t end = 0;
    Eigen::VectorXd center;
    Eigen::MatrixXd generators;
};

struct ObbPathCoverResult {
    bool success = false;
    std::vector<ObbPathCoverRegion> regions;
    ObbPortalValidationStats stats;
    double covered_length = 0.0;
    int windows_attempted = 0;
    int windows_success = 0;
    int recursive_splits = 0;
    int failed_leaf_windows = 0;
    double failed_leaf_length_sum = 0.0;
    double failed_leaf_length_max = 0.0;
    bool has_first_failed_leaf = false;
    Eigen::VectorXd first_failed_leaf_a;
    Eigen::VectorXd first_failed_leaf_b;
};

bool validate_obb_zonotope_portal(const Robot& robot,
                                  const Scene& scene,
                                  const std::vector<Interval>& domain,
                                  const std::vector<Eigen::VectorXd>& path,
                                  double lateral_radius,
                                  double longitudinal_margin,
                                  double safety_epsilon,
                                  int grow_iterations,
                                  int binary_iterations,
                                  int max_validations,
                                  ObbPortalValidationStats& stats,
                                  Eigen::VectorXd* out_center = nullptr,
                                  Eigen::MatrixXd* out_generators = nullptr);

void obb_accumulate_stats(ObbPortalValidationStats& dst,
                          const ObbPortalValidationStats& src);

ObbPathCoverResult cover_segment_or_bridge_path_with_obbs(
    const Robot& robot,
    const Scene& scene,
    const std::vector<Interval>& domain,
    const std::vector<Eigen::VectorXd>& path,
    bool greedy_bridge_cover,
    int segment_split_depth,
    int max_window_segments,
    double lateral_radius,
    double longitudinal_margin,
    double safety_epsilon,
    int grow_iterations,
    int binary_iterations,
    int max_validations,
    std::vector<Eigen::VectorXd>& out_centerline);

} // namespace rbf
