#pragma once

#include <SBF/leaf_sweep_grower.h>
#include <SBF/oracle.h>

namespace rbf {

void record_leaf_sweep_worker_oracle_counters(LeafSweepResult& result,
											  StageContext& context,
											  const OracleCounters& counters);

}  // namespace rbf
