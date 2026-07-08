#pragma once

#include <SBF/box_adjacency_types.h>
#include <SBF/query_graph_cache_types.h>
#include <SBF/query_graph_types.h>
#include <SBF/segment_edge_fwd.h>

#include <Eigen/Core>

#include <rbf/core.h>

#include <unordered_set>
#include <vector>

namespace rbf {

bool boxes_connected(const BoxNode& lhs, const BoxNode& rhs, double tolerance = 1e-9);
AdjacencyGraph compute_adjacency_reference(const std::vector<BoxNode>& boxes,
										   double tolerance = 1e-9,
										   int max_degree = 0,
										   double gap_tolerance = 0.0);
AdjacencyGraph compute_adjacency(const std::vector<BoxNode>& boxes,
								 double tolerance = 1e-9,
								 int max_degree = 0,
								 double gap_tolerance = 0.0);
AdjacencyBuildStats last_adjacency_build_stats();
QueryGraphCache build_query_graph_cache(const std::vector<BoxNode>& boxes,
										const AdjacencyGraph& graph,
										const SegmentEdgeList& segment_edges = {});
int append_segment_edge(SegmentEdgeList& edges,
						int source_box_id,
						int target_box_id,
						std::vector<Eigen::VectorXd> waypoints,
						SegmentEdgeType type,
						int segment_resolution,
						SegmentEdgeValidation validation,
						bool strict_audit_required = false,
						int query_index = -1);
int add_segment_edge(SegmentEdgeList& edges,
					 AdjacencyGraph& graph,
					 int source_box_id,
					 int target_box_id,
					 std::vector<Eigen::VectorXd> waypoints,
						 SegmentEdgeType type,
						 int segment_resolution,
						 SegmentEdgeValidation validation,
						 bool strict_audit_required = false,
						 int query_index = -1);
bool validate_portal_corridor_certificate(const BoxNode& source,
										  const BoxNode& target,
										  const std::vector<BoxNode>& internal_boxes,
										  double tolerance = 1e-9);
int append_portal_corridor_edge(SegmentEdgeList& edges,
								const BoxNode& source,
								const BoxNode& target,
								std::vector<BoxNode> internal_boxes,
								int portal_domain_id = -1,
								double tolerance = 1e-9,
								int query_index = -1);
int append_certified_portal_corridor_edge(SegmentEdgeList& edges,
										  const BoxNode& source,
										  const BoxNode& target,
										  std::vector<Eigen::VectorXd> waypoints,
										  SegmentEdgeValidation validation,
										  int portal_domain_id = -1,
										  int query_index = -1,
										  const Eigen::VectorXd* obb_center = nullptr,
										  const Eigen::MatrixXd* obb_generators = nullptr,
										  SegmentEdgeType edge_type = SegmentEdgeType::PortalCorridor,
										  const std::vector<Eigen::VectorXd>* obb_centers = nullptr,
										  const std::vector<Eigen::MatrixXd>* obb_generators_list = nullptr);
int add_portal_corridor_edge(SegmentEdgeList& edges,
							 AdjacencyGraph& graph,
							 const BoxNode& source,
							 const BoxNode& target,
							 std::vector<BoxNode> internal_boxes,
							 int portal_domain_id = -1,
							 double tolerance = 1e-9,
							 int query_index = -1);
void apply_segment_edges_to_adjacency(const SegmentEdgeList& edges, AdjacencyGraph& graph);
const SegmentEdge* find_segment_edge(const SegmentEdgeList& edges, int source_box_id, int target_box_id);
const SegmentEdge* find_segment_edge(const QueryGraphCache& cache, int source_box_id, int target_box_id);
bool counts_as_segment_edge(SegmentEdgeType type);
bool counts_as_query_repair_edge(SegmentEdgeType type);
std::vector<std::vector<int>> find_islands(const AdjacencyGraph& graph);
std::unordered_set<int> find_articulation_points(const AdjacencyGraph& graph);
DijkstraResult dijkstra_search(const AdjacencyGraph& graph,
							   const std::vector<BoxNode>& boxes,
							   int start_box_id,
							   int goal_box_id,
							   const Eigen::VectorXd& goal_point = {});
DijkstraResult dijkstra_search(const AdjacencyGraph& graph,
							   const std::vector<BoxNode>& boxes,
							   const SegmentEdgeList& segment_edges,
							   int start_box_id,
							   int goal_box_id,
							   const Eigen::VectorXd& goal_point = {});
DijkstraResult dijkstra_search(const QueryGraphCache& cache,
							   int start_box_id,
							   int goal_box_id,
							   const Eigen::VectorXd& goal_point = {});
DijkstraResult dijkstra_search(const QueryGraphCache& cache,
							   int start_box_id,
							   int goal_box_id,
							   const Eigen::VectorXd& start_point,
							   const Eigen::VectorXd& goal_point);
DijkstraResult dijkstra_search(const QueryGraphCache& cache,
							   int start_box_id,
							   int goal_box_id,
							   const Eigen::VectorXd& start_point,
							   const Eigen::VectorXd& goal_point,
							   const QueryGraphCostOptions& cost_options);
std::vector<int> shortcut_box_sequence(const std::vector<int>& sequence, const AdjacencyGraph& graph);
std::vector<int> shortcut_box_sequence(const std::vector<int>& sequence, const QueryGraphCache& cache);
std::vector<int> shortcut_box_sequence(const std::vector<int>& sequence,
									   const QueryGraphCache& cache,
									   const QueryShortcutCostOptions& shortcut_options);
std::vector<Eigen::VectorXd> extract_waypoints(const std::vector<int>& box_sequence,
											   const std::vector<BoxNode>& boxes,
											   const Eigen::Ref<const Eigen::VectorXd>& start,
											   const Eigen::Ref<const Eigen::VectorXd>& goal);
std::vector<Eigen::VectorXd> extract_waypoints(const std::vector<int>& box_sequence,
											   const std::vector<BoxNode>& boxes,
											   const SegmentEdgeList& segment_edges,
											   const Eigen::Ref<const Eigen::VectorXd>& start,
											   const Eigen::Ref<const Eigen::VectorXd>& goal);
std::vector<Eigen::VectorXd> extract_waypoints(const std::vector<int>& box_sequence,
											   const QueryGraphCache& cache,
											   const Eigen::Ref<const Eigen::VectorXd>& start,
											   const Eigen::Ref<const Eigen::VectorXd>& goal);
double path_length(const std::vector<Eigen::VectorXd>& path);
int locate_containing_box(const std::vector<BoxNode>& boxes,
						  const Eigen::Ref<const Eigen::VectorXd>& q,
						  bool nearest_if_outside = false);
int locate_containing_box(const QueryGraphCache& cache,
						  const Eigen::Ref<const Eigen::VectorXd>& q,
						  bool nearest_if_outside = false);

}  // namespace rbf
