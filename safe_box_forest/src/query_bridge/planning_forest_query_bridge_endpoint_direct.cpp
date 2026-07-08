#include <SBF/safe_box_forest.h>

#include <SBF/scene.h>
#include <SBF/runtime.h>

#include <SBF/adaptive_grid_partition.h>
#include <SBF/box_graph.h>

#include "../planning_core/planning_forest_audit.h"
#include "../planning_core/planning_forest_diagnostics.h"
#include "../query_runtime/planning_forest_query_utils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace rbf {

int RBFPlanningForest::connect_query_endpoint_to_main_island(
    const Eigen::Ref<const Eigen::VectorXd>& point,
    double max_segment_length) {
    if (boxes_.empty() || !oracle_) {
        return 0;
    }
    const auto add_direct_diag = [&](const std::string& suffix, double value = 1.0) {
        last_build_.diagnostics["query_bridge.endpoint_to_main_direct_" + suffix] += value;
    };
    record_portal_membership_policy(last_build_.diagnostics, config_.portal_membership_policy);
    const std::size_t boxes_before_endpoint_main = boxes_.size();
    std::vector<int> pre_anchor_main_island_storage;
    if (adaptive_partition_query_enabled_ && adaptive_partition_ && !adaptive_partition_->empty()) {
        pre_anchor_main_island_storage = adaptive_partition_->largest_component_box_ids_with_overlay();
    }
    int source_box_id = locate_box_partition_first(point, config_.query.nearest_if_outside);

    std::vector<int> main_island_storage =
        endpoint_main_largest_island_partition_first(pre_anchor_main_island_storage);
    if (main_island_storage.empty()) {
        if (partition_native_mode()) {
            add_direct_diag("partition_missing_no_graph_fallback");
            return 0;
        }
        return 0;
    }
    const auto& main_island = main_island_storage;
    if (source_box_id >= 0 &&
        std::find(main_island.begin(), main_island.end(), source_box_id) != main_island.end()) {
        add_direct_diag("already_main");
        return 0;
    }

    int target_box_id = -1;
    Eigen::VectorXd target_point = point;
    double best_dist2 = std::numeric_limits<double>::infinity();
    if (adaptive_partition_query_enabled_ && adaptive_partition_ && !adaptive_partition_->empty()) {
        const auto nearest = adaptive_partition_->nearest_boxes(point, main_island, 1);
        if (!nearest.empty()) {
            target_box_id = nearest.front().box_id;
            target_point = nearest.front().closest_point;
            best_dist2 = nearest.front().distance_sq;
            add_direct_diag("partition_nearest");
        }
    } else {
        for (int box_id : main_island) {
            const BoxNode* box = find_box_by_id(boxes_, box_id);
            if (box == nullptr || box->n_dims() != point.size()) {
                continue;
            }
            const Eigen::VectorXd candidate = closest_point_in_box(*box, point);
            const double dist2 = (candidate - point).squaredNorm();
            if (dist2 < best_dist2) {
                best_dist2 = dist2;
                target_box_id = box_id;
                target_point = candidate;
            }
        }
    }
    if (target_box_id < 0 || best_dist2 <= 1e-18) {
        add_direct_diag("missing_target");
        return 0;
    }
    const double length = std::sqrt(best_dist2);
    if (max_segment_length > 0.0 && length > max_segment_length) {
        add_direct_diag("too_long");
        add_direct_diag("too_long_length", length);
        if (source_box_id < 0) {
            add_direct_diag("too_long_anchor_skipped");
        }
        return 0;
    }

    if (source_box_id < 0) {
        StageContext context = StageContext::from_runtime(config_.runtime);
        source_box_id = anchor_query_endpoint_box(point, context);
        merge_diagnostic_snapshot(last_build_.diagnostics, context.diagnostics().snapshot());
        if (partition_native_mode() &&
            adaptive_partition_query_enabled_ &&
            adaptive_partition_ &&
            boxes_.size() > boxes_before_endpoint_main) {
            append_adaptive_partition_boxes(boxes_before_endpoint_main,
                                            &last_build_,
                                            "query_bridge.endpoint_to_main_direct_anchor");
            source_box_id = locate_box_partition_first(point, false);
        }
    }
    if (source_box_id < 0) {
        add_direct_diag("missing_source");
        return 0;
    }

    CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
    std::vector<Eigen::VectorXd> waypoints{point, target_point};
    const PathAuditCheck audit = audit_waypoint_path(waypoints,
                                                     checker,
                                                     config_.query.audit_resolution,
                                                     config_.query.audit_segment_step);
    add_direct_diag("attempts");
    if (!audit.passed) {
        add_direct_diag("audit_fail");
        return 0;
    }
    const int edge_id = add_segment_edge_partition_first(source_box_id,
                                                         target_box_id,
                                                         std::move(waypoints),
                                                         SegmentEdgeType::QueryBridge,
                                                         config_.query.audit_resolution,
                                                         SegmentEdgeValidation::CollisionChecked,
                                                         true,
                                                         -1);
    if (edge_id < 0) {
        add_direct_diag("add_fail");
        return 0;
    }
    add_direct_diag("success");
    add_direct_diag("length", length);
    sync_adaptive_partition_segment_edges(&last_build_, "query_bridge.endpoint_to_main_direct");
    invalidate_query_cache();
    return 1;
}

} // namespace rbf
