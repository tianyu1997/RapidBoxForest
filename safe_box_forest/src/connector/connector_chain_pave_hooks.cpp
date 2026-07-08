#include "connector_chain_pave_internal.h"

namespace rbf {

#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API
void record_chain_pave_boundary_debug_failure(const FindFreeBoxResult& result,
                                              const Eigen::VectorXd& seed,
                                              BoxOracle& oracle,
                                              const ChainPaveConfig& config);
#endif

void record_optional_chain_pave_boundary_failure_payload(const FindFreeBoxResult& result,
                                                         const Eigen::VectorXd& seed,
                                                         BoxOracle& oracle,
                                                         const ChainPaveConfig& config) {
#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API
    record_chain_pave_boundary_debug_failure(result, seed, oracle, config);
#else
    (void)result;
    (void)seed;
    (void)oracle;
    (void)config;
#endif
}

}  // namespace rbf
