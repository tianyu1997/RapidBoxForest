#pragma once

#include <SBF/query_graph_types.h>

#include <rbf/core.h>

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace rbf {

enum class PartitionCellState : std::uint8_t {
	Unknown = 0,
	Free = 1,
	Deferred = 2,
	Blocked = 3,
	Unresolved = 4,
	FreeMerged = 5,
	NonGridFree = 6,
};

struct CellPath {
	std::vector<std::uint64_t> words;
	int bit_count = 0;

	bool empty() const noexcept { return bit_count == 0; }
	void push_child(bool right_child);
	bool bit(int index) const;
};

struct GridRange {
	int root_index = -1;
	std::vector<std::uint64_t> lo;
	std::vector<std::uint64_t> hi;

	bool valid() const noexcept;
};

struct PartitionCell {
	int cell_id = -1;
	int box_id = -1;
	std::size_t box_index = 0;
	CellPath path;
	GridRange grid;
	std::vector<Interval> intervals;
	PartitionCellState state = PartitionCellState::Unknown;
	bool grid_aligned = false;
	int island_id = -1;
};

struct AdaptiveGridPartitionSparseCellRecord {
	int cell_id = -1;
	int box_id = -1;
	int root_index = -1;
	std::vector<std::uint64_t> lo;
	std::vector<std::uint64_t> hi;
	std::vector<int> split_counts;
	std::vector<Interval> intervals;
	PartitionCellState state = PartitionCellState::Unknown;
	bool grid_aligned = false;
	bool exact_interval_lookup_eligible = false;
	int address_depth = 0;
	std::uint64_t ancestor_refs_avoided = 0;
	std::uint64_t interval_fingerprint = 0;
	std::uint64_t split_policy_hash = 0;
};

struct AdaptiveGridPartitionStats {
	int cells = 0;
	int grid_cells = 0;
	int non_grid_cells = 0;
	int face_index_entries = 0;
	int point_index_dims = 0;
	int point_index_entries = 0;
	int point_index_overflow_cells = 0;
	int sparse_virtual_cells = 0;
	int sparse_virtual_grid_cells = 0;
	int sparse_virtual_non_grid_cells = 0;
	int sparse_virtual_exact_index_entries = 0;
	int sparse_virtual_max_address_depth = 0;
	std::uint64_t sparse_virtual_ancestor_refs_avoided = 0;
	int islands = 0;
	int largest_island = 0;
	std::uint64_t adjacency_candidates = 0;
	std::uint64_t adjacency_tests = 0;
	std::uint64_t adjacency_edges = 0;
	int overlay_edges = 0;
	double build_ms = 0.0;
	double index_rebuild_ms = 0.0;
	double sparse_virtual_index_ms = 0.0;
	double face_index_ms = 0.0;
	double point_index_ms = 0.0;
	double neighbor_cache_ms = 0.0;
	double island_rebuild_ms = 0.0;
};

struct AdaptiveGridPartitionMergeOptions {
	int max_rounds = 1;
	double max_ms = 0.0;
	bool grid_line_merge = true;
	bool containment_prune = true;
};

struct AdaptiveGridPartitionMergeResult {
	int input_boxes = 0;
	int output_boxes = 0;
	int grid_merges = 0;
	int containment_pruned = 0;
	int containment_skipped = 0;
	std::uint64_t containment_bucket_entries = 0;
	std::uint64_t containment_candidates = 0;
	std::uint64_t containment_tests = 0;
	int containment_overflow = 0;
	int rounds = 0;
	int stop_reason = 0;
	double total_ms = 0.0;
	double containment_ms = 0.0;
	double line_merge_ms = 0.0;
	std::vector<int> released_box_ids;
};

struct AdaptiveGridPartitionDeltaResult {
	int boxes_removed = 0;
	int boxes_appended = 0;
	double update_ms = 0.0;
};

struct AdaptiveGridPartitionQueryOptions {
	bool nearest_if_outside = false;
	bool shortcut_boxes = true;
	QueryShortcutCostOptions shortcut_cost;
	int max_expansions = 0;
	double adjacency_tolerance = 1e-9;
	QueryGraphCostOptions graph_cost;
};

struct AdaptiveGridPartitionQueryResult {
	bool found = false;
	int start_box_id = -1;
	int goal_box_id = -1;
	std::vector<int> box_sequence;
	std::vector<int> segment_edge_sequence;
	std::vector<Eigen::VectorXd> path;
	double total_cost = 0.0;
	double search_ms = 0.0;
	int cells_expanded = 0;
	int adjacency_candidates = 0;
	int non_grid_cells_used = 0;
	int overlay_edges_used = 0;
};

struct AdaptiveGridPartitionNearestBox {
	int box_id = -1;
	Eigen::VectorXd closest_point;
	double distance_sq = 0.0;
};

struct AdaptiveGridPartitionComponentPair {
	int source_box_id = -1;
	int target_box_id = -1;
	int source_component_index = -1;
	int target_component_index = 0;
	int source_component_size = 0;
	int target_component_size = 0;
	Eigen::VectorXd source_point;
	Eigen::VectorXd target_point;
	double distance_sq = 0.0;
};

struct AdaptiveGridPartitionConnectivityDominance {
	int adjacent_box_count = 0;
	int adjacent_component_count = 0;
	bool covered_by_existing = false;
	bool adjacent_to_largest_component = false;
	bool connector_candidate = false;
	bool single_component = false;
	bool isolated = true;
	double priority_delta = 0.0;
};

struct AdaptiveGridPartitionLandmark {
	int box_id = -1;
	int component_index = -1;
	Eigen::VectorXd center;
	double volume = 0.0;
};

}  // namespace rbf
