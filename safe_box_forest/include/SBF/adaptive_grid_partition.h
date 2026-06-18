#pragma once

#include <SBF/api.h>

#include <rbf/lect_database/split_policy.h>

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
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
	int max_expansions = 0;
	double adjacency_tolerance = 1e-9;
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

class AdaptiveGridPartition {
public:
	bool rebuild(const std::vector<Interval>& root_intervals,
				 const lect_database::SplitPolicyDescriptor& split_policy,
				 int root_depth,
				 int target_depth,
				 const std::vector<BoxNode>& boxes,
				 double tolerance);
	bool rebuild(const std::vector<std::vector<Interval>>& root_interval_copies,
				 const lect_database::SplitPolicyDescriptor& split_policy,
				 int root_depth,
				 int target_depth,
				 const std::vector<BoxNode>& boxes,
				 double tolerance);

	void clear();
	bool empty() const noexcept { return cells_.empty(); }
	const AdaptiveGridPartitionStats& stats() const noexcept { return stats_; }

	bool append_box(const BoxNode& box, double tolerance);
	int append_boxes(const std::vector<BoxNode>& boxes,
					 std::size_t first_index,
					 double tolerance);
	int remove_box_ids(const std::unordered_set<int>& box_ids);
	AdaptiveGridPartitionDeltaResult replace_box_ids_with_boxes(
		const std::unordered_set<int>& box_ids,
		const std::vector<BoxNode>& boxes,
		std::size_t first_index,
		double tolerance);
	void rebuild_indices();
	AdaptiveGridPartitionMergeResult merge_boxes(std::vector<BoxNode>& boxes,
												 const AdaptiveGridPartitionMergeOptions& options,
												 double tolerance);
	bool contains_box_id(int box_id) const;
	bool intervals_for_box(int box_id, std::vector<Interval>& intervals) const;
	bool center_for_box(int box_id, Eigen::VectorXd& center) const;
	bool closest_point_for_box(int box_id,
							   const Eigen::Ref<const Eigen::VectorXd>& q,
							   Eigen::VectorXd& closest,
							   double* distance_sq = nullptr) const;
	bool box_contains_point(int box_id,
							const Eigen::Ref<const Eigen::VectorXd>& q,
							double tolerance) const;
	bool box_adjacent_to_box(int box_id,
							 const BoxNode& box,
							 double tolerance) const;
	int sparse_virtual_cell_for_intervals(const std::vector<Interval>& intervals,
										  double tolerance) const;
	std::optional<AdaptiveGridPartitionSparseCellRecord> sparse_virtual_record_for_intervals(
		const std::vector<Interval>& intervals,
		double tolerance) const;
	std::vector<AdaptiveGridPartitionSparseCellRecord> sparse_virtual_records() const;
	bool boxes_are_neighbors(int lhs_box_id, int rhs_box_id) const;
	int island_id_for_box(int box_id) const;
	bool same_island(int lhs_box_id, int rhs_box_id) const;
	bool same_component_with_overlay(int lhs_box_id, int rhs_box_id) const;
	int component_count_with_overlay() const;
	std::vector<std::vector<int>> component_box_ids_with_overlay() const;
	std::vector<std::vector<int>> island_box_ids() const;
	std::vector<int> largest_island_box_ids() const;
	std::vector<int> largest_component_box_ids_with_overlay() const;
	bool box_adjacent_to_any(const BoxNode& box,
							 const std::unordered_set<int>& box_ids,
							 double tolerance) const;
	std::vector<int> adjacent_box_ids(const BoxNode& box,
									  double tolerance) const;
	AdaptiveGridPartitionConnectivityDominance classify_connectivity_dominance(
		const BoxNode& box,
		double tolerance) const;
	int sync_segment_edges(const SegmentEdgeList& edges);
	bool append_segment_edge(const SegmentEdge& edge);

	int locate_containing_box(const Eigen::Ref<const Eigen::VectorXd>& q,
							  bool nearest_if_outside,
							  double tolerance) const;
	std::vector<int> covering_box_ids(const Eigen::Ref<const Eigen::VectorXd>& q,
									  double tolerance) const;
	std::vector<int> covering_box_indices(const Eigen::Ref<const Eigen::VectorXd>& q,
										  double tolerance) const;
	std::vector<AdaptiveGridPartitionNearestBox> nearest_boxes(
		const Eigen::Ref<const Eigen::VectorXd>& q,
		const std::vector<int>& candidate_box_ids,
		int limit) const;
	std::vector<AdaptiveGridPartitionComponentPair> nearest_component_pairs_to_largest(
		int max_pairs_per_component,
		int candidate_cap) const;
	std::vector<AdaptiveGridPartitionLandmark> landmarks(bool largest_component_only,
														 int limit) const;
	std::vector<int> neighbor_box_ids(int box_id) const;
	AdaptiveGridPartitionQueryResult query(const Eigen::Ref<const Eigen::VectorXd>& start,
										   const Eigen::Ref<const Eigen::VectorXd>& goal,
										   const AdaptiveGridPartitionQueryOptions& options) const;

private:
	struct FaceKey {
		int root_index = -1;
		int dim = -1;
		std::uint64_t coord = 0;
		bool upper = false;
		bool operator==(const FaceKey& other) const noexcept {
			return root_index == other.root_index &&
				   dim == other.dim &&
				   coord == other.coord &&
				   upper == other.upper;
		}
	};
	struct FaceKeyHash {
		std::size_t operator()(const FaceKey& key) const noexcept;
	};
	struct PointBinKey {
		std::array<long long, 3> bins{0, 0, 0};
		bool operator==(const PointBinKey& other) const noexcept {
			return bins == other.bins;
		}
	};
	struct PointBinKeyHash {
		std::size_t operator()(const PointBinKey& key) const noexcept;
	};
	struct SparseCellKey {
		int root_index = -1;
		std::vector<std::uint64_t> lo;
		std::vector<std::uint64_t> hi;
		bool operator==(const SparseCellKey& other) const noexcept {
			return root_index == other.root_index && lo == other.lo && hi == other.hi;
		}
	};
	struct SparseCellKeyHash {
		std::size_t operator()(const SparseCellKey& key) const noexcept;
	};
	struct OverlayEdge {
		int edge_id = -1;
		int source_cell = -1;
		int target_cell = -1;
		int source_box_id = -1;
		int target_box_id = -1;
		double length = 0.0;
		SegmentEdgeType type = SegmentEdgeType::Unknown;
		SegmentEdgeValidation validation = SegmentEdgeValidation::Unknown;
		bool strict_audit_required = false;
		int query_index = -1;
		std::vector<Eigen::VectorXd> waypoints;
	};

	bool make_grid_range(const std::vector<Interval>& intervals,
						 GridRange& range,
						 double tolerance) const;
	bool add_cell_from_box(const BoxNode& box,
						   std::size_t box_index,
						   PartitionCellState state,
						   double tolerance);
	void rebuild_cells_from_boxes_only(const std::vector<BoxNode>& boxes,
									   double tolerance);
	bool grid_ranges_adjacent(const GridRange& lhs, const GridRange& rhs) const;
	bool grid_ranges_overlap_except_dim(const GridRange& lhs,
										const GridRange& rhs,
										int dim) const;
	bool grid_range_contains(const GridRange& outer, const GridRange& inner) const;
	void append_cell_to_indices(int cell_index);
	void update_islands_after_incremental_cell(int cell_index,
											   const std::vector<int>& neighbors);
	std::vector<int> compute_neighbor_cell_indices(int cell_index) const;
	std::vector<int> neighbor_cell_indices(int cell_index) const;
	std::vector<int> shortcut_sequence(const std::vector<int>& sequence,
									   const std::unordered_map<int, int>& cell_by_box_id) const;
	void clear_runtime_indices();
	void rebuild_point_index();
	SparseCellKey sparse_key_for_grid_range(const GridRange& range) const;
	int sparse_address_depth(const GridRange& range) const;
	void rebuild_sparse_virtual_index();
	void append_sparse_virtual_cell(int cell_index);
	std::vector<int> point_candidate_cells(const Eigen::Ref<const Eigen::VectorXd>& q) const;
	void rebuild_face_index();
	void rebuild_neighbor_cache();
	void rebuild_islands();
	void clear_overlay_edges();
	std::vector<std::vector<int>> component_cell_indices_with_overlay() const;
	std::vector<int> interval_candidate_cells(const std::vector<Interval>& intervals) const;
	void reset_overlay_components();
	void ensure_overlay_parent_size();
	int overlay_component_root(int cell_index) const;
	int overlay_component_root_mutable(int cell_index);
	void union_overlay_components(int lhs_cell, int rhs_cell);

	std::vector<Interval> root_intervals_;
	std::vector<std::vector<Interval>> root_interval_copies_;
	lect_database::SplitPolicyDescriptor split_policy_;
	int root_depth_ = 0;
	int target_depth_ = 0;
	double tolerance_ = 1e-9;
	std::vector<int> split_counts_;
	std::vector<PartitionCell> cells_;
	std::unordered_map<int, int> cell_by_box_id_;
	std::unordered_map<FaceKey, std::vector<int>, FaceKeyHash> face_index_;
	std::vector<std::vector<int>> neighbor_cache_;
	std::unordered_map<int, std::vector<OverlayEdge>> overlay_edges_by_cell_;
	std::unordered_set<int> overlay_edge_ids_;
	std::vector<int> overlay_parent_;
	std::vector<int> point_index_dims_;
	std::array<double, 3> point_bin_widths_{1.0, 1.0, 1.0};
	std::array<double, 3> point_bin_origins_{0.0, 0.0, 0.0};
	std::unordered_map<PointBinKey, std::vector<int>, PointBinKeyHash> point_bins_;
	std::vector<int> point_overflow_cells_;
	std::unordered_map<SparseCellKey, int, SparseCellKeyHash> sparse_virtual_index_;
	AdaptiveGridPartitionStats stats_;
};

}  // namespace rbf
