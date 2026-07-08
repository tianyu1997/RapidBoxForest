#pragma once

#include <SBF/adaptive_grid_partition_types.h>
#include <SBF/segment_edge_fwd.h>

#include <rbf/lect_database/split_policy.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rbf {

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
									   const std::unordered_map<int, int>& cell_by_box_id,
									   const QueryShortcutCostOptions& shortcut_options) const;
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
