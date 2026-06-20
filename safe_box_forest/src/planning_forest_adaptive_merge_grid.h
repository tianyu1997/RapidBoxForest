#pragma once

#include <SBF/safe_box_forest.h>

#include <chrono>
#include <vector>

namespace rbf {

int grid_line_merge_leaf(DatabaseBoxOracle& oracle,
                         std::vector<BoxNode>& boxes,
                         double tolerance,
                         int max_rounds,
                         int& rounds);

int tree_sibling_merge_leaf(BoxOracle& oracle,
                            std::vector<BoxNode>& boxes,
                            double tolerance,
                            int max_rounds,
                            const std::chrono::steady_clock::time_point* deadline,
                            bool& timed_out,
                            int& rounds);

}  // namespace rbf
