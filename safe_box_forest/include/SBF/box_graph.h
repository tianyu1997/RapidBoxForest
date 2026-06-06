#pragma once

#include <rbf/core.h>

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rbf {

using AdjacencyGraph = std::unordered_map<int, std::vector<int>>;

enum class SegmentEdgeType : std::uint8_t {
	Unknown = 0,
	PointValidatedGap = 1,
	RRTConnector = 2,
	QueryBridge = 3,
	BoxCorridor = 4,
};

enum class SegmentEdgeValidation : std::uint8_t {
	Unknown = 0,
	CollisionChecked = 1,
};

struct SegmentEdge {
	int id = -1;
	int source_box_id = -1;
	int target_box_id = -1;
	std::vector<Eigen::VectorXd> waypoints;
	SegmentEdgeType type = SegmentEdgeType::Unknown;
	SegmentEdgeValidation validation = SegmentEdgeValidation::Unknown;
	int segment_resolution = 0;
	double length = 0.0;
	bool strict_audit_required = false;
	int query_index = -1;
};

using SegmentEdgeList = std::vector<SegmentEdge>;

struct QueryGraphCache {
	const std::vector<BoxNode>* boxes = nullptr;
	const AdjacencyGraph* graph = nullptr;
	const SegmentEdgeList* segment_edges = nullptr;
	std::unordered_map<int, std::size_t> box_index_by_id;
	std::unordered_map<std::uint64_t, std::size_t> segment_edge_index_by_pair;
	std::unordered_map<int, std::unordered_set<int>> adjacency_sets;
	int point_index_dim = -1;
	double point_bin_width = 1.0;
	double point_bin_origin = 0.0;
	std::unordered_map<long long, std::vector<int>> point_bins;
};

struct DijkstraResult {
	bool found = false;
	std::vector<int> box_sequence;
	std::vector<int> segment_edge_sequence;
	double total_cost = 0.0;
};

bool boxes_connected(const BoxNode& lhs, const BoxNode& rhs, double tolerance = 1e-9);
AdjacencyGraph compute_adjacency_reference(const std::vector<BoxNode>& boxes,
										   double tolerance = 1e-9,
										   int max_degree = 0,
										   double gap_tolerance = 0.0);
AdjacencyGraph compute_adjacency(const std::vector<BoxNode>& boxes,
								 double tolerance = 1e-9,
								 int max_degree = 0,
								 double gap_tolerance = 0.0);
QueryGraphCache build_query_graph_cache(const std::vector<BoxNode>& boxes,
										const AdjacencyGraph& graph,
										const SegmentEdgeList& segment_edges = {});
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
void apply_segment_edges_to_adjacency(const SegmentEdgeList& edges, AdjacencyGraph& graph);
const SegmentEdge* find_segment_edge(const SegmentEdgeList& edges, int source_box_id, int target_box_id);
const SegmentEdge* find_segment_edge(const QueryGraphCache& cache, int source_box_id, int target_box_id);
bool counts_as_segment_edge(SegmentEdgeType type);
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
std::vector<int> shortcut_box_sequence(const std::vector<int>& sequence, const AdjacencyGraph& graph);
std::vector<int> shortcut_box_sequence(const std::vector<int>& sequence, const QueryGraphCache& cache);
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
