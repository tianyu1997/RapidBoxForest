#include "../connector/connector_chain_pave_internal.h"

#include <SBF/debug.h>
#include <SBF/oracle.h>

#include <utility>

namespace rbf {

void record_chain_pave_boundary_debug_failure(const FindFreeBoxResult& result,
                                              const Eigen::VectorXd& seed,
                                              BoxOracle& oracle,
                                              const ChainPaveConfig& config) {
    if (config.debug_boundary_failures == nullptr ||
        (!result.hit_unknown_depth_cap && !result.hit_reserved_depth_cap)) {
        return;
    }

    DebugBoundaryFfbFailure failure;
    failure.seed.assign(seed.data(), seed.data() + seed.size());
    failure.intervals = result.intervals;
    failure.validation_detail = result.validation_detail;
    failure.node = result.node;
    failure.depth = result.node >= 0 ? oracle.depth(result.node) : -1;
    failure.changed_dim = result.changed_dim;
    failure.fail_code = result.fail_code;
    failure.hit_unknown_depth_cap = result.hit_unknown_depth_cap;
    failure.hit_reserved_depth_cap = result.hit_reserved_depth_cap;
    config.debug_boundary_failures->push_back(std::move(failure));
}

}  // namespace rbf
