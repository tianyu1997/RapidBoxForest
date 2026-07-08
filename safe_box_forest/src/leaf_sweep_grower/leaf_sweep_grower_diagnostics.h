#pragma once

#include <SBF/leaf_sweep_types.h>
#include <SBF/runtime_fwd.h>

#include <LECTDatabase/sbf/oracle_types.h>

namespace rbf {

void record_leaf_sweep_worker_oracle_counters(LeafSweepResult& result,
											  StageContext& context,
											  const OracleCounters& counters);

}  // namespace rbf
