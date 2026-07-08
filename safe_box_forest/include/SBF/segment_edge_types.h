#pragma once

#include <SBF/segment_edge_fwd.h>

#include <rbf/core.h>

#include <Eigen/Core>

#include <vector>

namespace rbf {

struct SegmentEdge {
    int id = -1;
    int source_box_id = -1;
    int target_box_id = -1;
    std::vector<Eigen::VectorXd> waypoints;
    // For SegmentEdgeType::PortalCorridor with ConservativeBoxChain, this is
    // the hidden conservative internal box-chain certificate. These boxes are
    // not global graph vertices; path extraction lazily expands them only if
    // the edge is used. Other conservative portal certificates, such as
    // ConservativeObbZonotope, keep their certified centerline in waypoints.
    std::vector<BoxNode> internal_boxes;
    // For SegmentEdgeValidation::ConservativeObbZonotope, each region is a
    // certified C-space OBB stored as q = center + generators * xi,
    // xi in [-1,1]^k. These regions are hidden bridge volumes rather than
    // global graph vertices.
    std::vector<Eigen::VectorXd> obb_centers;
    std::vector<Eigen::MatrixXd> obb_generators;
    double obb_covered_length = 0.0;
    SegmentEdgeType type = SegmentEdgeType::Unknown;
    SegmentEdgeValidation validation = SegmentEdgeValidation::Unknown;
    int segment_resolution = 0;
    double length = 0.0;
    bool strict_audit_required = false;
    int query_index = -1;
    int portal_domain_id = -1;
    bool conservative_certificate = false;
};

}  // namespace rbf
