#include <SBF/safe_box_forest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "planning_forest_audit.h"

namespace rbf {

int RBFPlanningForest::add_partition_box_corridor_overlay(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    const std::vector<Eigen::VectorXd>& waypoint_path,
    const char* diagnostic_prefix,
    bool anchor_endpoints,
    bool skip_if_connected,
    int query_index,
    BuildProfile* profile) {
    BuildProfile* out_profile = profile != nullptr ? profile : &last_build_;
    const std::string prefix =
        (diagnostic_prefix != nullptr && diagnostic_prefix[0] != '\0')
            ? std::string(diagnostic_prefix)
            : std::string("partition_box_corridor");
    auto& diagnostics = out_profile->diagnostics;
    diagnostics[prefix + ".partition_box_corridor_overlay_attempts"] += 1.0;

    if (!partition_native_mode()) {
        diagnostics[prefix + ".partition_box_corridor_overlay_not_partition_native"] += 1.0;
        return 0;
    }
    if (!adaptive_partition_query_enabled_ || !adaptive_partition_ || adaptive_partition_->empty()) {
        diagnostics[prefix + ".partition_box_corridor_overlay_missing_partition"] += 1.0;
        return 0;
    }
    if (waypoint_path.size() < 2) {
        diagnostics[prefix + ".partition_box_corridor_overlay_missing_waypoints"] += 1.0;
        return 0;
    }

    StageContext anchor_context = StageContext::from_runtime(config_.runtime);
    const std::size_t boxes_before_anchor = boxes_.size();
    int source_box_id = locate_box_partition_first(start, false);
    if (source_box_id < 0 && anchor_endpoints) {
        source_box_id = anchor_query_endpoint_box(start, anchor_context);
    }
    int target_box_id = locate_box_partition_first(goal, false);
    if (target_box_id < 0 && anchor_endpoints) {
        target_box_id = anchor_query_endpoint_box(goal, anchor_context);
    }
    for (const auto& [key, value] : anchor_context.diagnostics().snapshot()) {
        diagnostics[prefix + ".partition_box_corridor_anchor." + key] += value;
    }

    int anchors_added = 0;
    if (boxes_.size() > boxes_before_anchor) {
        const std::string anchor_prefix = prefix + ".partition_box_corridor_anchor";
        anchors_added = append_adaptive_partition_boxes(boxes_before_anchor,
                                                        out_profile,
                                                        anchor_prefix.c_str());
        source_box_id = locate_box_partition_first(start, false);
        target_box_id = locate_box_partition_first(goal, false);
    }
    if (anchors_added > 0) {
        diagnostics[prefix + ".partition_box_corridor_overlay_anchor_boxes_added"] +=
            static_cast<double>(anchors_added);
    }

    if (source_box_id < 0 || target_box_id < 0) {
        diagnostics[prefix + ".partition_box_corridor_overlay_missing_endpoint"] += 1.0;
        return anchors_added;
    }
    if (source_box_id == target_box_id ||
        (skip_if_connected && overlay_path_connected_partition_first(source_box_id, target_box_id))) {
        diagnostics[prefix + ".partition_box_corridor_overlay_already_connected"] += 1.0;
        return anchors_added;
    }

    CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
    const PathAuditCheck audit = audit_waypoint_path(waypoint_path,
                                                     checker,
                                                     config_.query.audit_resolution,
                                                     config_.query.audit_segment_step);
    if (!audit.passed) {
        diagnostics[prefix + ".partition_box_corridor_overlay_audit_fail"] += 1.0;
        return anchors_added;
    }

    const std::string edge_prefix = prefix + ".partition_native";
    const int edge_id = add_segment_edge_partition_first(source_box_id,
                                                         target_box_id,
                                                         waypoint_path,
                                                         SegmentEdgeType::BoxCorridor,
                                                         std::max(1, config_.query.audit_resolution),
                                                         SegmentEdgeValidation::CollisionChecked,
                                                         false,
                                                         query_index,
                                                         out_profile,
                                                         edge_prefix.c_str());
    if (edge_id < 0) {
        diagnostics[prefix + ".partition_box_corridor_overlay_edge_fail"] += 1.0;
        return anchors_added;
    }
    diagnostics[prefix + ".partition_box_corridor_overlay_added"] += 1.0;
    invalidate_query_cache();
    return anchors_added + 1;
}

} // namespace rbf
