#pragma once

#include <SBF/adaptive_grid_partition_types.h>
#include <SBF/segment_edge_fwd.h>

#include <cstdint>
#include <vector>

namespace rbf::partition_detail {

double interval_width_safe(const Interval& interval);
long long interval_bin(double value, double origin, double width);
double choose_bin_width(const std::vector<PartitionCell>& cells, int dim);
std::vector<int> choose_point_index_dims(const std::vector<PartitionCell>& cells, int max_dims);
double point_to_box_distance_sq(const Eigen::Ref<const Eigen::VectorXd>& point,
								const std::vector<Interval>& intervals);
Eigen::VectorXd box_center(const std::vector<Interval>& intervals);
Eigen::VectorXd closest_point_in_intervals(const std::vector<Interval>& intervals,
										   const Eigen::Ref<const Eigen::VectorXd>& point);
double interval_box_distance_sq(const std::vector<Interval>& lhs,
								const std::vector<Interval>& rhs);
bool closest_points_between_interval_boxes(const std::vector<Interval>& lhs,
										   const std::vector<Interval>& rhs,
										   Eigen::VectorXd& lhs_point,
										   Eigen::VectorXd& rhs_point,
										   double& distance_sq);
std::vector<Interval> interval_hull_for_cells(const std::vector<PartitionCell>& cells,
											  const std::vector<int>& cell_indices);
std::vector<int> closest_cells_to_hull(const std::vector<PartitionCell>& cells,
									   const std::vector<int>& candidates,
									   const std::vector<Interval>& hull,
									   int limit);
Eigen::VectorXd shared_face_center(const std::vector<Interval>& lhs,
								   const std::vector<Interval>& rhs);
bool interval_boxes_connected(const std::vector<Interval>& lhs,
							  const std::vector<Interval>& rhs,
							  double tolerance);
bool interval_box_subset(const std::vector<Interval>& inner,
						 const std::vector<Interval>& outer,
						 double tolerance);
bool partition_counts_as_segment_edge(SegmentEdgeType type);
bool partition_counts_as_query_repair_edge(SegmentEdgeType type);
Eigen::VectorXd transition_waypoint_toward_goal(const std::vector<Interval>& lhs,
												const std::vector<Interval>& rhs,
												const Eigen::Ref<const Eigen::VectorXd>& from,
												const Eigen::Ref<const Eigen::VectorXd>& goal,
												double tolerance);
double distance_to_line(const Eigen::Ref<const Eigen::VectorXd>& point,
						const Eigen::Ref<const Eigen::VectorXd>& start,
						const Eigen::Ref<const Eigen::VectorXd>& goal);
bool intervals_contain_point(const std::vector<Interval>& intervals,
							 const Eigen::Ref<const Eigen::VectorXd>& point,
							 double tolerance);
std::uint64_t max_coord_for_splits(int split_count);
double box_volume_from_intervals(const std::vector<Interval>& intervals);
std::uint64_t packed_pair_key(int lhs, int rhs);

}  // namespace rbf::partition_detail
