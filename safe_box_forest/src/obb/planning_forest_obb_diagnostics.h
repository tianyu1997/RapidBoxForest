#pragma once

#include "planning_forest_obb.h"

#include <Eigen/Core>

#include <string>
#include <unordered_map>
#include <vector>

namespace rbf {

void record_obb_path_cover_diagnostics(std::unordered_map<std::string, double>& diagnostics,
                                       const std::string& key_prefix,
                                       const ObbPathCoverResult& cover,
                                       const std::vector<Eigen::VectorXd>& fallback_waypoints);

}  // namespace rbf
