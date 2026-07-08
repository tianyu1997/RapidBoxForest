#pragma once

#include <SBF/adaptive_leaf_sweep_config.h>
#include <SBF/scene_types.h>

#include <Eigen/Core>

#include <cstddef>
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

struct ObbValidationOptions {
    bool fast_primary_orientation = true;
    bool fallback_orientations_on_primary_fail = false;
    bool sampled_support_enabled = false;
    bool clearance_sampled_support_enabled = true;
    double clearance_lateral_l1_max = 5e-3;
    int clearance_samples = 17;
    double clearance_dense_line_l1_threshold = 0.03;
    int clearance_dense_samples = 17;
    int clearance_fast_samples = 0;
    bool clearance_first = false;
    int clearance_retry_attempts = 0;
    std::vector<double> clearance_retry_values;
    int clearance_retry_iters = -1;
    double clearance_retry_timeout_ms = -1.0;
};

ObbValidationOptions obb_validation_options_from_config(const AdaptiveLeafSweepConfig& config);

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
                                  Eigen::MatrixXd* out_generators = nullptr,
                                  ObbValidationOptions options = {});

void obb_accumulate_stats(ObbPortalValidationStats& dst,
                          const ObbPortalValidationStats& src);

void obb_record_region_volume(ObbPortalValidationStats& stats,
                              const Eigen::MatrixXd& generators);

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
    std::vector<Eigen::VectorXd>& out_centerline,
    ObbValidationOptions options = {});

} // namespace rbf
