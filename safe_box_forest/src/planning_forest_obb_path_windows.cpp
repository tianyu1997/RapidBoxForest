#include "planning_forest_obb_path_cover_internal.h"

#include <algorithm>
#include <utility>
#include <vector>

namespace rbf {

namespace {

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

}  // namespace

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

ObbGreedyWindowSearchResult obb_find_greedy_path_window(
    const Robot& robot,
    const Scene& scene,
    const std::vector<Interval>& domain,
    const std::vector<Eigen::VectorXd>& path,
    std::size_t begin,
    std::size_t max_end,
    double lateral_radius,
    double longitudinal_margin,
    double safety_epsilon,
    int grow_iterations,
    int binary_iterations,
    int max_validations,
    ObbPathCoverResult& result,
    ObbValidationOptions options) {
    ObbGreedyWindowSearchResult search;
    search.good_end = begin;
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
            search.good_end = end;
            search.center = std::move(center);
            search.generators = std::move(generators);
            step *= 2U;
        } else {
            first_fail = end;
            break;
        }
    }
    if (search.good_end == max_end) {
        first_fail = 0;
    } else if (first_fail == 0 && begin + step > max_end) {
        first_fail = max_end + 1U;
    }
    if (first_fail > search.good_end + 1U && search.good_end > begin) {
        std::size_t lo = search.good_end + 1U;
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
                search.good_end = mid;
                search.center = std::move(center);
                search.generators = std::move(generators);
                lo = mid + 1U;
            } else {
                if (mid == 0U) {
                    break;
                }
                hi = mid - 1U;
            }
        }
    }
    return search;
}

}  // namespace rbf
