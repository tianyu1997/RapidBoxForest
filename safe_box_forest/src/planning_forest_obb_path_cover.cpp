#include "planning_forest_obb.h"
#include "planning_forest_obb_path_cover_internal.h"

#include <algorithm>
#include <utility>
#include <vector>

namespace rbf {

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

void obb_commit_path_window_region(ObbPathCoverResult& result,
                                   const std::vector<Eigen::VectorXd>& path,
                                   std::size_t begin,
                                   std::size_t end,
                                   Eigen::VectorXd center,
                                   Eigen::MatrixXd generators,
                                   std::vector<Eigen::VectorXd>& centerline) {
    ObbPathCoverRegion region;
    region.begin = begin;
    region.end = end;
    region.center = std::move(center);
    region.generators = std::move(generators);
    obb_record_region_volume(result.stats, region.generators);
    result.regions.push_back(std::move(region));
    for (std::size_t index = begin + 1U; index <= end; ++index) {
        result.covered_length += (path[index] - path[index - 1U]).norm();
    }
    obb_append_centerline_waypoint(centerline, path[end]);
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

bool obb_apply_greedy_window_split_fallback(
    const Robot& robot,
    const Scene& scene,
    const std::vector<Interval>& domain,
    const std::vector<Eigen::VectorXd>& path,
    std::size_t begin,
    std::size_t last,
    int segment_split_depth,
    double lateral_radius,
    double longitudinal_margin,
    double safety_epsilon,
    int grow_iterations,
    int binary_iterations,
    int max_validations,
    ObbPathCoverResult& result,
    std::vector<Eigen::VectorXd>& out_centerline,
    ObbValidationOptions options) {
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
        obb_append_centerline_waypoint(out_centerline, waypoint);
    }
    if (!split_ok) {
        for (std::size_t index = begin + 1U; index <= last; ++index) {
            obb_append_centerline_waypoint(out_centerline, path[index]);
        }
        result.success = false;
        return false;
    }
    return true;
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
        ObbGreedyWindowSearchResult window =
            obb_find_greedy_path_window(robot,
                                        scene,
                                        domain,
                                        path,
                                        begin,
                                        max_end,
                                        lateral_radius,
                                        longitudinal_margin,
                                        safety_epsilon,
                                        grow_iterations,
                                        binary_iterations,
                                        max_validations,
                                        result,
                                        options);
        if (window.good_end <= begin) {
            if (!obb_apply_greedy_window_split_fallback(robot,
                                                        scene,
                                                        domain,
                                                        path,
                                                        begin,
                                                        last,
                                                        segment_split_depth,
                                                        lateral_radius,
                                                        longitudinal_margin,
                                                        safety_epsilon,
                                                        grow_iterations,
                                                        binary_iterations,
                                                        max_validations,
                                                        result,
                                                        out_centerline,
                                                        options)) {
                return result;
            }
            begin += 1U;
            continue;
        }
        obb_commit_path_window_region(result,
                                      path,
                                      begin,
                                      window.good_end,
                                      std::move(window.center),
                                      std::move(window.generators),
                                      out_centerline);
        begin = window.good_end;
    }
    result.success = !result.regions.empty() &&
                     !out_centerline.empty() &&
                     (out_centerline.back() - path.back()).norm() <= 1e-10;
    return result;
}

}  // namespace rbf
