#include <SBF/safe_box_forest.h>

#include <SBF/runtime.h>

#include <SBF/oracle.h>

#include "planning_forest_query_bridge_diagnostics.h"

#include <algorithm>
#include <vector>

namespace rbf {

std::vector<int> RBFPlanningForest::finish_query_bridge_batch_result(
    const std::vector<int>& added_by_query,
    std::size_t partition_refresh_base,
    std::size_t segment_edges_before_partition_refresh,
    bool oracle_counters_before_valid,
    const OracleCounters& oracle_counters_before) {
    if (oracle_counters_before_valid && oracle_) {
        const auto after = oracle_->counters();
        add_query_bridge_oracle_counter_delta(last_build_,
                                             oracle_counters_before,
                                             after);
    }
    const bool changed =
        boxes_.size() != partition_refresh_base ||
        segment_edges_.size() != segment_edges_before_partition_refresh ||
        std::any_of(added_by_query.begin(),
                    added_by_query.end(),
                    [](int added) { return added > 0; });
    if (boxes_.size() > partition_refresh_base) {
        append_adaptive_partition_boxes(partition_refresh_base,
                                        &last_build_,
                                        "query_bridge.batch");
    } else if (changed) {
        sync_adaptive_partition_segment_edges(&last_build_, "query_bridge.batch");
        refresh_adaptive_partition_diagnostics(&last_build_);
    }
    return added_by_query;
}

}  // namespace rbf
