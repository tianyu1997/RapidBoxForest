#pragma once

#include <SBF/safe_box_forest.h>

#include <chrono>

namespace rbf {

struct VirtualSparseBinaryFfbAttempt {
    bool supported = false;
    bool completed = false;
    FindFreeBoxResult result;
};

VirtualSparseBinaryFfbAttempt try_virtual_sparse_binary_ffb(
    BoxOracle& oracle,
    const Eigen::Ref<const Eigen::VectorXd>& seed,
    const std::vector<Interval>& domain,
    StageContext& context,
    const FindFreeBoxOptions& options,
    const OracleSplitOptions& split_options,
    int effective_max_depth,
    std::chrono::steady_clock::time_point start);

}  // namespace rbf
