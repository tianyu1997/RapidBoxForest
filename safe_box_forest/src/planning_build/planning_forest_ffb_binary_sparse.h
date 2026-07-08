#pragma once

#include <SBF/find_free_box_types.h>
#include <SBF/runtime.h>
#include <LECTDatabase/sbf/oracle_types.h>

#include <Eigen/Core>

#include <chrono>
#include <vector>

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
