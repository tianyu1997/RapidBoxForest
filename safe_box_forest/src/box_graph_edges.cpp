#include <SBF/box_graph.h>

#include <algorithm>
#include <cstdint>

namespace rbf {
namespace {

bool graph_has_edge(const AdjacencyGraph& graph, int lhs, int rhs) {
    auto it = graph.find(lhs);
    if (it == graph.end()) {
        return false;
    }
    return std::find(it->second.begin(), it->second.end(), rhs) != it->second.end();
}

void append_graph_edge(AdjacencyGraph& graph, int lhs, int rhs) {
    if (lhs < 0 || rhs < 0 || lhs == rhs) {
        return;
    }
    if (!graph_has_edge(graph, lhs, rhs)) {
        graph[lhs].push_back(rhs);
    }
    if (!graph_has_edge(graph, rhs, lhs)) {
        graph[rhs].push_back(lhs);
    }
}

std::uint64_t edge_pair_key(int lhs, int rhs) {
    const std::uint32_t lo = static_cast<std::uint32_t>(std::min(lhs, rhs));
    const std::uint32_t hi = static_cast<std::uint32_t>(std::max(lhs, rhs));
    return (static_cast<std::uint64_t>(lo) << 32) | static_cast<std::uint64_t>(hi);
}

double waypoint_path_length(const std::vector<Eigen::VectorXd>& waypoints) {
    double total = 0.0;
    for (std::size_t index = 1; index < waypoints.size(); ++index) {
        total += (waypoints[index] - waypoints[index - 1]).norm();
    }
    return total;
}

}  // namespace

int append_segment_edge(SegmentEdgeList& edges,
                        int source_box_id,
                        int target_box_id,
                        std::vector<Eigen::VectorXd> waypoints,
                        SegmentEdgeType type,
                        int segment_resolution,
                        SegmentEdgeValidation validation,
                        bool strict_audit_required,
                        int query_index) {
    if (source_box_id < 0 || target_box_id < 0) {
        return -1;
    }
    int next_id = 0;
    for (const auto& edge : edges) {
        next_id = std::max(next_id, edge.id + 1);
    }
    SegmentEdge edge;
    edge.id = next_id;
    edge.source_box_id = source_box_id;
    edge.target_box_id = target_box_id;
    edge.waypoints = std::move(waypoints);
    edge.type = type;
    edge.validation = validation;
    edge.segment_resolution = segment_resolution;
    edge.length = waypoint_path_length(edge.waypoints);
    edge.strict_audit_required = strict_audit_required;
    edge.query_index = query_index;
    edges.push_back(std::move(edge));
    return next_id;
}

int add_segment_edge(SegmentEdgeList& edges,
                     AdjacencyGraph& graph,
                     int source_box_id,
                     int target_box_id,
                     std::vector<Eigen::VectorXd> waypoints,
                     SegmentEdgeType type,
                     int segment_resolution,
                     SegmentEdgeValidation validation,
                     bool strict_audit_required,
                     int query_index) {
    const int next_id = append_segment_edge(edges,
                                            source_box_id,
                                            target_box_id,
                                            std::move(waypoints),
                                            type,
                                            segment_resolution,
                                            validation,
                                            strict_audit_required,
                                            query_index);
    if (next_id < 0) {
        return -1;
    }
    append_graph_edge(graph, source_box_id, target_box_id);
    return next_id;
}

bool validate_portal_corridor_certificate(const BoxNode& source,
                                          const BoxNode& target,
                                          const std::vector<BoxNode>& internal_boxes,
                                          double tolerance) {
    if (internal_boxes.empty()) {
        return false;
    }
    const BoxNode* previous = &source;
    for (const auto& internal : internal_boxes) {
        if (internal.safety_status != BoxSafetyStatus::CertifiedFree ||
            internal.strict_audit_required ||
            !boxes_connected(*previous, internal, tolerance)) {
            return false;
        }
        previous = &internal;
    }
    return boxes_connected(*previous, target, tolerance);
}

std::vector<Eigen::VectorXd> portal_corridor_centerline(const BoxNode& source,
                                                        const BoxNode& target,
                                                        const std::vector<BoxNode>& internal_boxes) {
    std::vector<Eigen::VectorXd> waypoints;
    waypoints.reserve(internal_boxes.size() + 2U);
    waypoints.push_back(source.center());
    for (const auto& internal : internal_boxes) {
        waypoints.push_back(internal.center());
    }
    waypoints.push_back(target.center());
    return waypoints;
}

int append_portal_corridor_edge(SegmentEdgeList& edges,
                                const BoxNode& source,
                                const BoxNode& target,
                                std::vector<BoxNode> internal_boxes,
                                int portal_domain_id,
                                double tolerance,
                                int query_index) {
    if (!validate_portal_corridor_certificate(source, target, internal_boxes, tolerance)) {
        return -1;
    }
    const int next_id = append_segment_edge(edges,
                                            source.id,
                                            target.id,
                                            portal_corridor_centerline(source, target, internal_boxes),
                                            SegmentEdgeType::PortalCorridor,
                                            0,
                                            SegmentEdgeValidation::ConservativeBoxChain,
                                            false,
                                            query_index);
    if (next_id < 0) {
        return -1;
    }
    auto& edge = edges.back();
    edge.internal_boxes = std::move(internal_boxes);
    edge.portal_domain_id = portal_domain_id;
    edge.conservative_certificate = true;
    return next_id;
}

int append_certified_portal_corridor_edge(SegmentEdgeList& edges,
                                          const BoxNode& source,
                                          const BoxNode& target,
                                          std::vector<Eigen::VectorXd> waypoints,
                                          SegmentEdgeValidation validation,
                                          int portal_domain_id,
                                          int query_index,
                                          const Eigen::VectorXd* obb_center,
                                          const Eigen::MatrixXd* obb_generators,
                                          SegmentEdgeType edge_type,
                                          const std::vector<Eigen::VectorXd>* obb_centers,
                                          const std::vector<Eigen::MatrixXd>* obb_generators_list) {
    if (validation != SegmentEdgeValidation::ConservativeObbZonotope ||
        waypoints.size() < 2U) {
        return -1;
    }
    if (edge_type != SegmentEdgeType::PortalCorridor &&
        edge_type != SegmentEdgeType::SegmentOBBCorridor &&
        edge_type != SegmentEdgeType::RRTBridgeOBBCorridor &&
        edge_type != SegmentEdgeType::TransitionOBBCorridor) {
        return -1;
    }
    const int next_id = append_segment_edge(edges,
                                            source.id,
                                            target.id,
                                            std::move(waypoints),
                                            edge_type,
                                            0,
                                            validation,
                                            false,
                                            query_index);
    if (next_id < 0) {
        return -1;
    }
    auto& edge = edges.back();
    edge.portal_domain_id = portal_domain_id;
    edge.conservative_certificate = true;
    if (obb_centers != nullptr && obb_generators_list != nullptr &&
        obb_centers->size() == obb_generators_list->size()) {
        for (std::size_t index = 0; index < obb_centers->size(); ++index) {
            const Eigen::VectorXd& center = (*obb_centers)[index];
            const Eigen::MatrixXd& generators = (*obb_generators_list)[index];
            if (center.size() > 0 && generators.rows() == center.size()) {
                edge.obb_centers.push_back(center);
                edge.obb_generators.push_back(generators);
            }
        }
    }
    if (obb_center != nullptr && obb_generators != nullptr &&
        obb_center->size() > 0 && obb_generators->rows() == obb_center->size()) {
        edge.obb_centers.push_back(*obb_center);
        edge.obb_generators.push_back(*obb_generators);
    }
    if (!edge.obb_centers.empty()) {
        edge.obb_covered_length = edge.length;
    }
    return next_id;
}

int add_portal_corridor_edge(SegmentEdgeList& edges,
                             AdjacencyGraph& graph,
                             const BoxNode& source,
                             const BoxNode& target,
                             std::vector<BoxNode> internal_boxes,
                             int portal_domain_id,
                             double tolerance,
                             int query_index) {
    const int next_id = append_portal_corridor_edge(edges,
                                                    source,
                                                    target,
                                                    std::move(internal_boxes),
                                                    portal_domain_id,
                                                    tolerance,
                                                    query_index);
    if (next_id < 0) {
        return -1;
    }
    append_graph_edge(graph, source.id, target.id);
    return next_id;
}

void apply_segment_edges_to_adjacency(const SegmentEdgeList& edges,
                                      AdjacencyGraph& graph) {
    for (const auto& edge : edges) {
        append_graph_edge(graph, edge.source_box_id, edge.target_box_id);
    }
}

const SegmentEdge* find_segment_edge(const SegmentEdgeList& edges,
                                     int source_box_id,
                                     int target_box_id) {
    const SegmentEdge* best = nullptr;
    for (const auto& edge : edges) {
        const bool matches = (edge.source_box_id == source_box_id && edge.target_box_id == target_box_id) ||
                             (edge.source_box_id == target_box_id && edge.target_box_id == source_box_id);
        if (!matches) {
            continue;
        }
        if (best == nullptr || edge.length < best->length) {
            best = &edge;
        }
    }
    return best;
}

const SegmentEdge* find_segment_edge(const QueryGraphCache& cache,
                                     int source_box_id,
                                     int target_box_id) {
    if (cache.segment_edges == nullptr) {
        return nullptr;
    }
    const auto it = cache.segment_edge_index_by_pair.find(edge_pair_key(source_box_id, target_box_id));
    if (it == cache.segment_edge_index_by_pair.end() || it->second >= cache.segment_edges->size()) {
        return nullptr;
    }
    return &(*cache.segment_edges)[it->second];
}

bool counts_as_segment_edge(SegmentEdgeType type) {
    return type != SegmentEdgeType::BoxCorridor &&
           type != SegmentEdgeType::PortalCorridor &&
           type != SegmentEdgeType::SegmentOBBCorridor &&
           type != SegmentEdgeType::RRTBridgeOBBCorridor &&
           type != SegmentEdgeType::TransitionOBBCorridor;
}

bool counts_as_query_repair_edge(SegmentEdgeType type) {
    return type == SegmentEdgeType::QueryBridge ||
           type == SegmentEdgeType::SegmentOBBCorridor ||
           type == SegmentEdgeType::RRTBridgeOBBCorridor ||
           type == SegmentEdgeType::TransitionOBBCorridor;
}


}  // namespace rbf
