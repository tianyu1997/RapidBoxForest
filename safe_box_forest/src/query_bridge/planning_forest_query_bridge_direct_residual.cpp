#include <SBF/safe_box_forest.h>

#include "planning_forest_audit.h"
#include "planning_forest_query_bridge_corridor_graph.h"
#include "planning_forest_query_bridge_corridor_repair.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <utility>
#include <vector>

namespace rbf {

void RBFPlanningForest::run_query_bridge_direct_corridor_residual_segments(
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<std::vector<int>>& sample_layers,
    const std::vector<int>& final_bad,
    const RRTConnectConfig& bridge_rrt,
    CollisionChecker& checker,
    StageContext& context,
    int bridge_edge_query_index,
    QueryBridgeLocalDsu& dsu,
    int& local_segment_edges_added,
    int& local_segment_gap_samples_max,
    double& residual_segment_audit_ms) {
    using Clock = std::chrono::steady_clock;
    auto insert_residual_segment = [&](int lhs_index,
                                       int rhs_index,
                                       const Eigen::VectorXd& lhs_point,
                                       const Eigen::VectorXd& rhs_point,
                                       int sample_gap) {
        if (lhs_index < 0 || rhs_index < 0 ||
            lhs_index >= static_cast<int>(boxes_.size()) ||
            rhs_index >= static_cast<int>(boxes_.size())) {
            return false;
        }
        if (dsu.find(lhs_index) == dsu.find(rhs_index)) {
            return false;
        }
        std::vector<Eigen::VectorXd> gap_path{lhs_point, rhs_point};
        const auto segment_audit_t0 = Clock::now();
        const PathAuditCheck gap_audit =
            audit_waypoint_path(gap_path,
                                checker,
                                config_.query.audit_resolution,
                                config_.query.audit_segment_step);
        residual_segment_audit_ms +=
            std::chrono::duration<double, std::milli>(Clock::now() -
                                                      segment_audit_t0).count();
        if (!gap_audit.passed) {
            context.diagnostics().add_counter(
                "query_bridge.direct_corridor_segment_audit_rejects");
            return false;
        }
        const int edge_id = add_segment_edge_partition_first(
            boxes_[static_cast<std::size_t>(lhs_index)].id,
            boxes_[static_cast<std::size_t>(rhs_index)].id,
            std::move(gap_path),
            SegmentEdgeType::QueryBridge,
            bridge_rrt.segment_resolution,
            SegmentEdgeValidation::CollisionChecked,
            true,
            bridge_edge_query_index);
        if (edge_id < 0) {
            return false;
        }
        local_segment_edges_added += 1;
        local_segment_gap_samples_max =
            std::max(local_segment_gap_samples_max, sample_gap);
        dsu.unite(lhs_index, rhs_index);
        return true;
    };
    query_bridge_run_residual_segment_gap_pass(context,
                                               samples,
                                               sample_layers,
                                               final_bad,
                                               insert_residual_segment);
}

}  // namespace rbf
