#include "planning_forest_obb.h"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace rbf {

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

void obb_append_centerline_waypoint(std::vector<Eigen::VectorXd>& centerline,
                                    const Eigen::VectorXd& waypoint) {
    if (centerline.empty() || (centerline.back() - waypoint).norm() > 1e-12) {
        centerline.push_back(waypoint);
    }
}

void obb_commit_segment_region(ObbPathCoverResult& result,
                               std::vector<Eigen::VectorXd>& centerline,
                               const Eigen::VectorXd& a,
                               const Eigen::VectorXd& b,
                               Eigen::VectorXd center,
                               Eigen::MatrixXd generators) {
    ObbPathCoverRegion region;
    region.begin = centerline.empty() ? 0U : centerline.size() - 1U;
    region.end = region.begin + 1U;
    region.center = std::move(center);
    region.generators = std::move(generators);
    obb_record_region_volume(result.stats, region.generators);
    result.regions.push_back(std::move(region));
    result.covered_length += (b - a).norm();
    obb_append_centerline_waypoint(centerline, a);
    obb_append_centerline_waypoint(centerline, b);
}

std::vector<Eigen::VectorXd> obb_path_slice(const std::vector<Eigen::VectorXd>& path,
                                            std::size_t begin,
                                            std::size_t end) {
    std::vector<Eigen::VectorXd> out;
    if (path.empty() || begin >= path.size() || end >= path.size() || begin >= end) {
        return out;
    }
    out.reserve(end - begin + 1U);
    for (std::size_t index = begin; index <= end; ++index) {
        out.push_back(path[index]);
    }
    return out;
}

bool obb_validate_path_window(const Robot& robot,
                              const Scene& scene,
                              const std::vector<Interval>& domain,
                              const std::vector<Eigen::VectorXd>& path,
                              std::size_t begin,
                              std::size_t end,
                              double lateral_radius,
                              double longitudinal_margin,
                              double safety_epsilon,
                              int grow_iterations,
                              int binary_iterations,
                              int max_validations,
                              ObbPathCoverResult& result,
                              Eigen::VectorXd& center,
                              Eigen::MatrixXd& generators,
                              ObbValidationOptions options) {
    ++result.windows_attempted;
    std::vector<Eigen::VectorXd> window = obb_path_slice(path, begin, end);
    ObbPortalValidationStats local_stats;
    const bool ok = validate_obb_zonotope_portal(robot,
                                                 scene,
                                                 domain,
                                                 window,
                                                 lateral_radius,
                                                 longitudinal_margin,
                                                 safety_epsilon,
                                                 grow_iterations,
                                                 binary_iterations,
                                                 max_validations,
                                                 local_stats,
                                                 &center,
                                                 &generators,
                                                 options);
    obb_accumulate_stats(result.stats, local_stats);
    if (ok) {
        ++result.windows_success;
    }
    return ok;
}

bool obb_cover_segment_recursive(const Robot& robot,
                                 const Scene& scene,
                                 const std::vector<Interval>& domain,
                                 const Eigen::VectorXd& a,
                                 const Eigen::VectorXd& b,
                                 int depth_remaining,
                                 double lateral_radius,
                                 double longitudinal_margin,
                                 double safety_epsilon,
                                 int grow_iterations,
                                 int binary_iterations,
                                 int max_validations,
                                 ObbPathCoverResult& result,
                                 std::vector<Eigen::VectorXd>& centerline,
                                 ObbValidationOptions options) {
    std::vector<Eigen::VectorXd> segment_path{a, b};
    Eigen::VectorXd center;
    Eigen::MatrixXd generators;
    if (obb_validate_path_window(robot,
                                 scene,
                                 domain,
                                 segment_path,
                                 0,
                                 1,
                                 lateral_radius,
                                 longitudinal_margin,
                                 safety_epsilon,
                                 grow_iterations,
                                 binary_iterations,
                                 max_validations,
                                 result,
                                 center,
                                 generators,
                                 options)) {
        obb_commit_segment_region(result,
                                  centerline,
                                  a,
                                  b,
                                  std::move(center),
                                  std::move(generators));
        return true;
    }
    if (depth_remaining <= 0) {
        std::vector<Eigen::VectorXd> segment_path{a, b};
        Eigen::VectorXd fallback_center;
        Eigen::MatrixXd fallback_generators;
        const int fallback_budget = max_validations > 0 ? std::max(max_validations, 16) : 16;
        if (obb_validate_path_window(robot,
                                     scene,
                                     domain,
                                     segment_path,
                                     0,
                                     1,
                                     0.0,
                                     0.0,
                                     safety_epsilon,
                                     0,
                                     0,
                                     fallback_budget,
                                     result,
                                     fallback_center,
                                     fallback_generators,
                                     options)) {
            Eigen::VectorXd best_center = fallback_center;
            Eigen::MatrixXd best_generators = fallback_generators;
            double lo = 0.0;
            double hi = std::max(0.0, lateral_radius);
            for (int iter = 0; iter < std::max(0, binary_iterations) && hi > 0.0; ++iter) {
                const double mid = 0.5 * (lo + hi);
                Eigen::VectorXd mid_center;
                Eigen::MatrixXd mid_generators;
                if (obb_validate_path_window(robot,
                                             scene,
                                             domain,
                                             segment_path,
                                             0,
                                             1,
                                             mid,
                                             0.0,
                                             safety_epsilon,
                                             0,
                                             0,
                                             fallback_budget,
                                             result,
                                             mid_center,
                                             mid_generators,
                                             options)) {
                    lo = mid;
                    best_center = std::move(mid_center);
                    best_generators = std::move(mid_generators);
                } else {
                    hi = mid;
                }
            }
            obb_commit_segment_region(result,
                                      centerline,
                                      a,
                                      b,
                                      std::move(best_center),
                                      std::move(best_generators));
            return true;
        }
        ++result.failed_leaf_windows;
        const double failed_length = (b - a).norm();
        result.failed_leaf_length_sum += failed_length;
        result.failed_leaf_length_max = std::max(result.failed_leaf_length_max, failed_length);
        if (!result.has_first_failed_leaf) {
            result.has_first_failed_leaf = true;
            result.first_failed_leaf_a = a;
            result.first_failed_leaf_b = b;
        }
        obb_append_centerline_waypoint(centerline, a);
        obb_append_centerline_waypoint(centerline, b);
        return false;
    }
    ++result.recursive_splits;
    const Eigen::VectorXd mid = 0.5 * (a + b);
    const bool left_ok = obb_cover_segment_recursive(robot,
                                                     scene,
                                                     domain,
                                                     a,
                                                     mid,
                                                     depth_remaining - 1,
                                                     lateral_radius,
                                                     longitudinal_margin,
                                                     safety_epsilon,
                                                     grow_iterations,
                                                     binary_iterations,
                                                     max_validations,
                                                     result,
                                                     centerline,
                                                     options);
    const bool right_ok = obb_cover_segment_recursive(robot,
                                                      scene,
                                                      domain,
                                                      mid,
                                                      b,
                                                      depth_remaining - 1,
                                                      lateral_radius,
                                                      longitudinal_margin,
                                                      safety_epsilon,
                                                      grow_iterations,
                                                      binary_iterations,
                                                      max_validations,
                                                      result,
                                                      centerline,
                                                      options);
    return left_ok && right_ok;
}

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
    ObbValidationOptions options) {
    ObbPathCoverResult result;
    out_centerline.clear();
    if (path.size() < 2U) {
        ++result.stats.degenerate_rejects;
        return result;
    }
    if (!greedy_bridge_cover || path.size() == 2U) {
        result.success = obb_cover_segment_recursive(robot,
                                                     scene,
                                                     domain,
                                                     path.front(),
                                                     path.back(),
                                                     std::max(0, segment_split_depth),
                                                     lateral_radius,
                                                     longitudinal_margin,
                                                     safety_epsilon,
                                                     grow_iterations,
                                                     binary_iterations,
                                                     max_validations,
                                                     result,
                                                     out_centerline,
                                                     options);
        return result;
    }

    out_centerline.push_back(path.front());
    const std::size_t last = path.size() - 1U;
    const int window_cap = std::max(1, max_window_segments);
    std::size_t begin = 0;
    while (begin < last) {
        const std::size_t max_end =
            std::min(last, begin + static_cast<std::size_t>(window_cap));
        std::size_t good_end = begin;
        Eigen::VectorXd good_center;
        Eigen::MatrixXd good_generators;
        std::size_t step = 1;
        std::size_t first_fail = 0;
        while (begin + step <= max_end) {
            Eigen::VectorXd center;
            Eigen::MatrixXd generators;
            const std::size_t end = begin + step;
            if (obb_validate_path_window(robot,
                                         scene,
                                         domain,
                                         path,
                                         begin,
                                         end,
                                         lateral_radius,
                                         longitudinal_margin,
                                         safety_epsilon,
                                         grow_iterations,
                                         binary_iterations,
                                         max_validations,
                                         result,
                                         center,
                                         generators,
                                         options)) {
                good_end = end;
                good_center = std::move(center);
                good_generators = std::move(generators);
                step *= 2U;
            } else {
                first_fail = end;
                break;
            }
        }
        if (good_end == max_end) {
            first_fail = 0;
        } else if (first_fail == 0 && begin + step > max_end) {
            first_fail = max_end + 1U;
        }
        if (first_fail > good_end + 1U && good_end > begin) {
            std::size_t lo = good_end + 1U;
            std::size_t hi = std::min(first_fail - 1U, max_end);
            while (lo <= hi) {
                const std::size_t mid = lo + (hi - lo) / 2U;
                Eigen::VectorXd center;
                Eigen::MatrixXd generators;
                if (obb_validate_path_window(robot,
                                             scene,
                                             domain,
                                             path,
                                             begin,
                                             mid,
                                             lateral_radius,
                                             longitudinal_margin,
                                             safety_epsilon,
                                             grow_iterations,
                                             binary_iterations,
                                             max_validations,
                                             result,
                                             center,
                                             generators,
                                             options)) {
                    good_end = mid;
                    good_center = std::move(center);
                    good_generators = std::move(generators);
                    lo = mid + 1U;
                } else {
                    if (mid == 0U) {
                        break;
                    }
                    hi = mid - 1U;
                }
            }
        }
        if (good_end <= begin) {
            std::vector<Eigen::VectorXd> split_line;
            const bool split_ok = obb_cover_segment_recursive(robot,
                                                              scene,
                                                              domain,
                                                              path[begin],
                                                              path[begin + 1U],
                                                              std::max(0, segment_split_depth),
                                                              lateral_radius,
                                                              longitudinal_margin,
                                                              safety_epsilon,
                                                              grow_iterations,
                                                              binary_iterations,
                                                              max_validations,
                                                              result,
                                                              split_line,
                                                              options);
            for (const auto& waypoint : split_line) {
                if (out_centerline.empty() || (out_centerline.back() - waypoint).norm() > 1e-12) {
                    out_centerline.push_back(waypoint);
                }
            }
            if (!split_ok) {
                for (std::size_t index = begin + 1U; index <= last; ++index) {
                    if ((out_centerline.back() - path[index]).norm() > 1e-12) {
                        out_centerline.push_back(path[index]);
                    }
                }
                result.success = false;
                return result;
            }
            begin += 1U;
            continue;
        }
        ObbPathCoverRegion region;
        region.begin = begin;
        region.end = good_end;
        region.center = std::move(good_center);
        region.generators = std::move(good_generators);
        obb_record_region_volume(result.stats, region.generators);
        result.regions.push_back(std::move(region));
        for (std::size_t index = begin + 1U; index <= good_end; ++index) {
            result.covered_length += (path[index] - path[index - 1U]).norm();
        }
        if ((out_centerline.back() - path[good_end]).norm() > 1e-12) {
            out_centerline.push_back(path[good_end]);
        }
        begin = good_end;
    }
    result.success = !result.regions.empty() &&
                     !out_centerline.empty() &&
                     (out_centerline.back() - path.back()).norm() <= 1e-10;
    return result;
}

}  // namespace rbf
