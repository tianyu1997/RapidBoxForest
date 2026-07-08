#pragma once

#include <SBF/grower.h>

namespace rbf {

int select_depth_stage_index(const GrowerConfig& config, int box_count);

const GrowerConfig::DepthStage* depth_stage_or_null(const GrowerConfig& config,
                                                    int stage_index);

FindFreeBoxOptions staged_ffb_options(const GrowerConfig& config, int stage_index);

FindFreeBoxOptions component_connect_ffb_options(const GrowerConfig& config,
                                                 StageContext& context,
                                                 const FindFreeBoxOptions& base_options,
                                                 int stage_index,
                                                 int pair_unknown_failures);

void record_grower_ffb_failure(StageContext& context,
                               const FindFreeBoxResult& result);

void record_worker_oracle_counters(StageContext& context,
                                   const OracleCounters& counters);

}  // namespace rbf
