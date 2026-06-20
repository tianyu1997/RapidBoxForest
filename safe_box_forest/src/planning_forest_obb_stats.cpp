#include "planning_forest_obb.h"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>

namespace rbf {

namespace {

double obb_generator_parallelotope_log_volume(const Eigen::MatrixXd& generators) {
    const int rows = generators.rows();
    const int cols = generators.cols();
    if (rows <= 0 || cols < rows) {
        return -std::numeric_limits<double>::infinity();
    }
    Eigen::MatrixXd gram = generators * generators.transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(gram, Eigen::EigenvaluesOnly);
    if (solver.info() != Eigen::Success) {
        return -std::numeric_limits<double>::infinity();
    }
    double log_volume = static_cast<double>(rows) * std::log(2.0);
    for (int dim = 0; dim < rows; ++dim) {
        const double eig = solver.eigenvalues()[dim];
        if (!(eig > 0.0) || !std::isfinite(eig)) {
            return -std::numeric_limits<double>::infinity();
        }
        log_volume += 0.5 * std::log(eig);
    }
    return log_volume;
}

double obb_generator_parallelotope_volume(const Eigen::MatrixXd& generators) {
    const double log_volume = obb_generator_parallelotope_log_volume(generators);
    if (!std::isfinite(log_volume)) {
        return 0.0;
    }
    if (log_volume > std::log(std::numeric_limits<double>::max())) {
        return std::numeric_limits<double>::infinity();
    }
    if (log_volume < std::log(std::numeric_limits<double>::min())) {
        return 0.0;
    }
    return std::exp(log_volume);
}

}  // namespace

void obb_record_region_volume(ObbPortalValidationStats& stats,
                              const Eigen::MatrixXd& generators) {
    const double volume = obb_generator_parallelotope_volume(generators);
    const double log_volume = obb_generator_parallelotope_log_volume(generators);
    stats.region_volume_sum += volume;
    stats.region_volume_max = std::max(stats.region_volume_max, volume);
    if (std::isfinite(log_volume)) {
        stats.region_log_volume_sum += log_volume;
    }
    stats.region_volume_count += 1;
}

void obb_accumulate_stats(ObbPortalValidationStats& dst,
                          const ObbPortalValidationStats& src) {
    dst.joint_limit_rejects += src.joint_limit_rejects;
    dst.degenerate_rejects += src.degenerate_rejects;
    dst.candidates += src.candidates;
    dst.validations += src.validations;
    dst.valid_candidates += src.valid_candidates;
    dst.grow_attempts += src.grow_attempts;
    dst.aabb_tests += src.aabb_tests;
    dst.aabb_rejects += src.aabb_rejects;
    dst.gjk_tests += src.gjk_tests;
    dst.gjk_rejects += src.gjk_rejects;
    dst.gjk_iterations += src.gjk_iterations;
    dst.maybe_pairs += src.maybe_pairs;
    dst.sampled_support_attempts += src.sampled_support_attempts;
    dst.sampled_support_success += src.sampled_support_success;
    dst.sampled_support_fail += src.sampled_support_fail;
    dst.sampled_support_samples += src.sampled_support_samples;
    dst.clearance_support_attempts += src.clearance_support_attempts;
    dst.clearance_support_success += src.clearance_support_success;
    dst.clearance_support_fail += src.clearance_support_fail;
    dst.clearance_support_samples += src.clearance_support_samples;
    dst.active_links = std::max(dst.active_links, src.active_links);
    dst.variables = std::max(dst.variables, src.variables);
    dst.longitudinal_radius = std::max(dst.longitudinal_radius, src.longitudinal_radius);
    dst.lateral_radius = std::max(dst.lateral_radius, src.lateral_radius);
    dst.sampled_support_error_radius =
        std::max(dst.sampled_support_error_radius, src.sampled_support_error_radius);
    dst.clearance_support_error_radius =
        std::max(dst.clearance_support_error_radius, src.clearance_support_error_radius);
    dst.clearance_support_min_margin =
        std::min(dst.clearance_support_min_margin, src.clearance_support_min_margin);
    dst.region_volume_sum += src.region_volume_sum;
    dst.region_volume_max = std::max(dst.region_volume_max, src.region_volume_max);
    dst.region_log_volume_sum += src.region_log_volume_sum;
    dst.region_volume_count += src.region_volume_count;
}

}  // namespace rbf
