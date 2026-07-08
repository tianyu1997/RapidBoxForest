#pragma once

#include <SBF/query_config.h>
#include <SBF/scene_types.h>
#include <SBF/segment_edge_fwd.h>

#include <Eigen/Core>

#include <vector>

namespace rbf {

struct PathAuditCheck {
    bool passed = false;
    int failed_segment_index = -1;
};

CollisionChecker make_audit_checker(const Robot& robot,
                                    const Scene& scene,
                                    const QueryConfig& query_config);

int effective_audit_segment_resolution(const Eigen::VectorXd& start,
                                       const Eigen::VectorXd& goal,
                                       int min_resolution,
                                       double segment_step);

PathAuditCheck audit_waypoint_path(const std::vector<Eigen::VectorXd>& path,
                                   const CollisionChecker& checker,
                                   int resolution,
                                   double segment_step);

bool audit_waypoint_path_passes(const std::vector<Eigen::VectorXd>& path,
                                const CollisionChecker& checker,
                                int resolution,
                                double segment_step);

bool segment_edge_survives_scene(const SegmentEdge& edge,
                                 const CollisionChecker& checker,
                                 int audit_resolution,
                                 double audit_segment_step);

} // namespace rbf
