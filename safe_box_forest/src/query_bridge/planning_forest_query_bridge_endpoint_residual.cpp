#include <SBF/safe_box_forest.h>

#include <SBF/adaptive_grid_partition.h>
#include <SBF/box_graph.h>

#include "planning_forest_audit.h"
#include "planning_forest_query_utils.h"

#include <vector>

namespace rbf {

bool RBFPlanningForest::try_add_endpoint_main_residual_segment_edge(
    int front_box_id,
    int target_box_id,
    const Eigen::Ref<const Eigen::VectorXd>& target_point,
    bool max_depth_ffb_failed,
    double max_segment_length) {
    if (!max_depth_ffb_failed || max_segment_length <= 0.0) {
        return false;
    }
    Eigen::VectorXd front_point;
    Eigen::VectorXd main_point;
    const bool use_partition_endpoint_index =
        partition_native_mode() &&
        adaptive_partition_query_enabled_ &&
        adaptive_partition_;
    if (use_partition_endpoint_index &&
        adaptive_partition_->closest_point_for_box(front_box_id,
                                                   target_point,
                                                   front_point) &&
        adaptive_partition_->closest_point_for_box(target_box_id,
                                                   front_point,
                                                   main_point)) {
    } else {
        const BoxNode* front_box = find_box_by_id(boxes_, front_box_id);
        const BoxNode* target_box = find_box_by_id(boxes_, target_box_id);
        if (front_box == nullptr || target_box == nullptr) {
            return false;
        }
        front_point = closest_point_in_box(*front_box, target_point);
        main_point = closest_point_in_box(*target_box, front_point);
    }
    const double length = (main_point - front_point).norm();
    if (length > max_segment_length) {
        return false;
    }
    CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
    std::vector<Eigen::VectorXd> waypoints{front_point, main_point};
    const PathAuditCheck audit = audit_waypoint_path(waypoints,
                                                     checker,
                                                     config_.query.audit_resolution,
                                                     config_.query.audit_segment_step);
    if (!audit.passed) {
        return false;
    }
    const int edge_id = add_segment_edge_partition_first(front_box_id,
                                                         target_box_id,
                                                         std::move(waypoints),
                                                         SegmentEdgeType::QueryBridge,
                                                         config_.query.audit_resolution,
                                                         SegmentEdgeValidation::CollisionChecked,
                                                         true,
                                                         -1);
    if (edge_id < 0) {
        return false;
    }
    last_build_.diagnostics["endpoint_main.residual_segment_edges"] += 1.0;
    return true;
}

}  // namespace rbf
