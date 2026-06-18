#include "planning_forest_audit.h"

#include <algorithm>
#include <cmath>

namespace rbf {

CollisionChecker make_audit_checker(const Robot& robot,
                                    const Scene& scene,
                                    const QueryConfig& query_config) {
    CollisionChecker checker(robot, scene);
    checker.set_collision_tolerance(query_config.audit_collision_tolerance);
    return checker;
}

int effective_audit_segment_resolution(const Eigen::VectorXd& start,
                                       const Eigen::VectorXd& goal,
                                       int min_resolution,
                                       double segment_step) {
    const int safe_resolution = std::max(1, min_resolution);
    if (!(segment_step > 0.0) || !std::isfinite(segment_step)) {
        return safe_resolution;
    }
    const double distance = (goal - start).norm();
    if (!(distance > 0.0) || !std::isfinite(distance)) {
        return safe_resolution;
    }
    const int step_resolution = std::max(2, static_cast<int>(std::ceil(distance / segment_step)));
    return std::max(safe_resolution, step_resolution);
}

PathAuditCheck audit_waypoint_path(const std::vector<Eigen::VectorXd>& path,
                                   const CollisionChecker& checker,
                                   int resolution,
                                   double segment_step) {
    PathAuditCheck audit;
    if (path.empty()) {
        audit.failed_segment_index = 0;
        return audit;
    }
    const int safe_resolution = std::max(1, resolution);
    for (std::size_t index = 0; index < path.size(); ++index) {
        if (checker.check_config(path[index])) {
            audit.failed_segment_index = index == 0 ? 0 : static_cast<int>(index - 1);
            return audit;
        }
    }
    for (std::size_t index = 0; index + 1 < path.size(); ++index) {
        const int segment_resolution = effective_audit_segment_resolution(
            path[index],
            path[index + 1],
            safe_resolution,
            segment_step);
        if (checker.check_segment(path[index], path[index + 1], segment_resolution)) {
            audit.failed_segment_index = static_cast<int>(index);
            return audit;
        }
    }
    audit.passed = true;
    return audit;
}

bool audit_waypoint_path_passes(const std::vector<Eigen::VectorXd>& path,
                                const CollisionChecker& checker,
                                int resolution,
                                double segment_step) {
    return audit_waypoint_path(path, checker, resolution, segment_step).passed;
}

bool segment_edge_survives_scene(const SegmentEdge& edge,
                                 const CollisionChecker& checker,
                                 int audit_resolution,
                                 double audit_segment_step) {
    if (edge.waypoints.size() < 2) {
        return false;
    }
    const int resolution = std::max({1, audit_resolution, edge.segment_resolution});
    return audit_waypoint_path(edge.waypoints, checker, resolution, audit_segment_step).passed;
}

} // namespace rbf
