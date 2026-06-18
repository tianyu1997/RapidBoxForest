#include <SBF/safe_box_forest.h>

#include <SBF/connector.h>

namespace rbf {

int RBFPlanningForest::run_query_bridge_chain_pave(
    const std::vector<Eigen::VectorXd>& waypoint_path,
    int start_box_id,
    int& next_id,
    StageContext& context,
    const ChainPaveConfig& pave_config,
    const char* partition_prefix) {
    if (!oracle_) {
        return 0;
    }
    const std::size_t boxes_before = boxes_.size();
    const int added = chain_pave_along_path(waypoint_path,
                                            start_box_id,
                                            boxes_,
                                            *oracle_,
                                            adjacency_,
                                            next_id,
                                            context,
                                            pave_config);
    if (added > 0) {
        append_adaptive_partition_boxes(boxes_before,
                                        &last_build_,
                                        partition_prefix);
        context.diagnostics().add_counter(
            "query_bridge.full_adjacency_rebuilds_avoided");
        invalidate_query_cache();
    }
    return added;
}

} // namespace rbf
