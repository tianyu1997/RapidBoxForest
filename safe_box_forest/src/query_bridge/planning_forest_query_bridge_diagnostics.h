#pragma once

#include <SBF/api.h>
#include <SBF/runtime.h>
#include <LECTDatabase/sbf/oracle_types.h>

#include "planning_forest_query_bridge_options.h"
#include "planning_forest_query_bridge_task.h"

#include <cstddef>
#include <string>

namespace rbf {

void add_query_bridge_oracle_counter_delta(BuildProfile& profile,
                                           const OracleCounters& before,
                                           const OracleCounters& after);

std::string query_bridge_task_key(std::size_t index, const std::string& suffix);

class QueryBridgeTaskDiagnostics {
public:
    QueryBridgeTaskDiagnostics(StageContext& context, int task_index);
    QueryBridgeTaskDiagnostics(StageContext& context, std::size_t task_index);

    void add_counter(const std::string& suffix, double value = 1.0) const;
    void set_value(const std::string& suffix, double value) const;

private:
    StageContext& context_;
    std::size_t task_index_ = 0;
    bool enabled_ = false;
};

void query_bridge_mark_task_skip(BuildProfile& profile,
                                 std::size_t index,
                                 double code,
                                 const char* reason);

void record_query_bridge_batch_task_no_path(StageContext& context,
                                            std::size_t index,
                                            double total_ms);

void record_query_bridge_batch_task_already_satisfied(
    StageContext& context,
    const QueryBridgeSearchTask& task,
    double probe_ms);

void record_query_bridge_batch_task_skipped_after_rrt(StageContext& context,
                                                      std::size_t index,
                                                      bool forced_task,
                                                      double probe_ms,
                                                      double total_ms);

void record_query_bridge_batch_task_skipped_by_hipac_after_rrt(
    StageContext& context,
    std::size_t index,
    double total_ms);

void record_query_bridge_forced_attempts(StageContext& context,
                                         std::size_t index,
                                         bool forced_task,
                                         int attempts);

void accumulate_query_bridge_direct_corridor_totals(const BuildProfile& profile,
                                                    StageContext& context,
                                                    std::size_t task_index);

}  // namespace rbf
