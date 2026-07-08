#pragma once

#include "planning_forest_obb.h"

namespace rbf {

struct ObbGreedyWindowSearchResult {
    std::size_t good_end = 0;
    Eigen::VectorXd center;
    Eigen::MatrixXd generators;
};

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
                              ObbValidationOptions options);

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
    ObbValidationOptions options);

}  // namespace rbf
