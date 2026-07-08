#include "adaptive_grid_partition_geometry.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace rbf::partition_detail {

double interval_width_safe(const Interval& interval) {
	return std::max(0.0, interval.width());
}

long long interval_bin(double value, double origin, double width) {
	if (width <= 0.0) {
		return 0;
	}
	return static_cast<long long>(std::floor((value - origin) / width));
}

double choose_bin_width(const std::vector<PartitionCell>& cells, int dim) {
	if (cells.empty() || dim < 0) {
		return 1.0;
	}
	double total_width = 0.0;
	double min_lo = std::numeric_limits<double>::infinity();
	double max_hi = -std::numeric_limits<double>::infinity();
	for (const auto& cell : cells) {
		if (dim >= static_cast<int>(cell.intervals.size())) {
			continue;
		}
		const auto& iv = cell.intervals[static_cast<std::size_t>(dim)];
		total_width += interval_width_safe(iv);
		min_lo = std::min(min_lo, iv.lo);
		max_hi = std::max(max_hi, iv.hi);
	}
	if (!std::isfinite(min_lo) || !std::isfinite(max_hi) || max_hi <= min_lo) {
		return 1.0;
	}
	const double avg_width = total_width / static_cast<double>(std::max<std::size_t>(1, cells.size()));
	const double span = max_hi - min_lo;
	return std::max(span / 256.0, std::max(avg_width, 1e-9));
}

std::vector<int> choose_point_index_dims(const std::vector<PartitionCell>& cells, int max_dims) {
	if (cells.empty()) {
		return {};
	}
	const int dims = static_cast<int>(cells.front().intervals.size());
	std::vector<std::pair<double, int>> scored;
	scored.reserve(static_cast<std::size_t>(dims));
	for (int dim = 0; dim < dims; ++dim) {
		double min_lo = std::numeric_limits<double>::infinity();
		double max_hi = -std::numeric_limits<double>::infinity();
		double total_width = 0.0;
		int counted = 0;
		for (const auto& cell : cells) {
			if (dim >= static_cast<int>(cell.intervals.size())) {
				continue;
			}
			const auto& iv = cell.intervals[static_cast<std::size_t>(dim)];
			min_lo = std::min(min_lo, iv.lo);
			max_hi = std::max(max_hi, iv.hi);
			total_width += interval_width_safe(iv);
			++counted;
		}
		const double span = max_hi - min_lo;
		if (!std::isfinite(span) || span <= 0.0 || counted <= 0) {
			continue;
		}
		const double avg_width = total_width / static_cast<double>(counted);
		const double score = span / std::max(avg_width, 1e-12);
		scored.emplace_back(score, dim);
	}
	std::sort(scored.begin(), scored.end(), [](const auto& lhs, const auto& rhs) {
		if (std::abs(lhs.first - rhs.first) > 1e-12) {
			return lhs.first > rhs.first;
		}
		return lhs.second < rhs.second;
	});
	std::vector<int> selected;
	const int limit = std::min<int>({3, std::max(1, max_dims), static_cast<int>(scored.size())});
	selected.reserve(static_cast<std::size_t>(limit));
	for (int index = 0; index < limit; ++index) {
		selected.push_back(scored[static_cast<std::size_t>(index)].second);
	}
	return selected;
}

double point_to_box_distance_sq(const Eigen::Ref<const Eigen::VectorXd>& point,
								const std::vector<Interval>& intervals) {
	double dist = 0.0;
	for (int dim = 0; dim < point.size() && dim < static_cast<int>(intervals.size()); ++dim) {
		const auto& iv = intervals[static_cast<std::size_t>(dim)];
		double delta = 0.0;
		if (point[dim] < iv.lo) {
			delta = iv.lo - point[dim];
		} else if (point[dim] > iv.hi) {
			delta = point[dim] - iv.hi;
		}
		dist += delta * delta;
	}
	return dist;
}

Eigen::VectorXd box_center(const std::vector<Interval>& intervals) {
	Eigen::VectorXd center(static_cast<int>(intervals.size()));
	for (int dim = 0; dim < center.size(); ++dim) {
		center[dim] = intervals[static_cast<std::size_t>(dim)].center();
	}
	return center;
}

Eigen::VectorXd closest_point_in_intervals(const std::vector<Interval>& intervals,
										   const Eigen::Ref<const Eigen::VectorXd>& point) {
	Eigen::VectorXd closest(static_cast<int>(intervals.size()));
	for (int dim = 0; dim < closest.size(); ++dim) {
		const auto& interval = intervals[static_cast<std::size_t>(dim)];
		closest[dim] = std::clamp(point[dim], interval.lo, interval.hi);
	}
	return closest;
}

double interval_box_distance_sq(const std::vector<Interval>& lhs,
								const std::vector<Interval>& rhs) {
	if (lhs.size() != rhs.size()) {
		return std::numeric_limits<double>::infinity();
	}
	double dist = 0.0;
	for (std::size_t dim = 0; dim < lhs.size(); ++dim) {
		double delta = 0.0;
		if (lhs[dim].hi < rhs[dim].lo) {
			delta = rhs[dim].lo - lhs[dim].hi;
		} else if (rhs[dim].hi < lhs[dim].lo) {
			delta = lhs[dim].lo - rhs[dim].hi;
		}
		dist += delta * delta;
	}
	return dist;
}

bool closest_points_between_interval_boxes(const std::vector<Interval>& lhs,
										   const std::vector<Interval>& rhs,
										   Eigen::VectorXd& lhs_point,
										   Eigen::VectorXd& rhs_point,
										   double& distance_sq) {
	if (lhs.size() != rhs.size()) {
		return false;
	}
	lhs_point.resize(static_cast<int>(lhs.size()));
	rhs_point.resize(static_cast<int>(rhs.size()));
	distance_sq = 0.0;
	for (std::size_t dim = 0; dim < lhs.size(); ++dim) {
		if (lhs[dim].hi < rhs[dim].lo) {
			lhs_point[static_cast<int>(dim)] = lhs[dim].hi;
			rhs_point[static_cast<int>(dim)] = rhs[dim].lo;
		} else if (rhs[dim].hi < lhs[dim].lo) {
			lhs_point[static_cast<int>(dim)] = lhs[dim].lo;
			rhs_point[static_cast<int>(dim)] = rhs[dim].hi;
		} else {
			const double lo = std::max(lhs[dim].lo, rhs[dim].lo);
			const double hi = std::min(lhs[dim].hi, rhs[dim].hi);
			const double mid = 0.5 * (lo + hi);
			lhs_point[static_cast<int>(dim)] = mid;
			rhs_point[static_cast<int>(dim)] = mid;
		}
		const double delta = lhs_point[static_cast<int>(dim)] -
							 rhs_point[static_cast<int>(dim)];
		distance_sq += delta * delta;
	}
	return true;
}

std::vector<Interval> interval_hull_for_cells(const std::vector<PartitionCell>& cells,
											  const std::vector<int>& cell_indices) {
	std::vector<Interval> hull;
	for (int cell_index : cell_indices) {
		if (cell_index < 0 || cell_index >= static_cast<int>(cells.size())) {
			continue;
		}
		const auto& intervals = cells[static_cast<std::size_t>(cell_index)].intervals;
		if (hull.empty()) {
			hull = intervals;
			continue;
		}
		if (hull.size() != intervals.size()) {
			continue;
		}
		for (std::size_t dim = 0; dim < hull.size(); ++dim) {
			hull[dim] = hull[dim].hull(intervals[dim]);
		}
	}
	return hull;
}

std::vector<int> closest_cells_to_hull(const std::vector<PartitionCell>& cells,
									   const std::vector<int>& candidates,
									   const std::vector<Interval>& hull,
									   int limit) {
	if (limit <= 0 || static_cast<int>(candidates.size()) <= limit) {
		return candidates;
	}
	std::vector<std::pair<double, int>> scored;
	scored.reserve(candidates.size());
	for (int cell_index : candidates) {
		if (cell_index < 0 || cell_index >= static_cast<int>(cells.size())) {
			continue;
		}
		const double score =
			interval_box_distance_sq(cells[static_cast<std::size_t>(cell_index)].intervals,
									 hull);
		scored.emplace_back(score, cell_index);
	}
	const int keep = std::min<int>(limit, static_cast<int>(scored.size()));
	std::partial_sort(scored.begin(),
					  scored.begin() + keep,
					  scored.end(),
					  [](const auto& lhs, const auto& rhs) {
						  if (std::abs(lhs.first - rhs.first) > 1e-18) {
							  return lhs.first < rhs.first;
						  }
						  return lhs.second < rhs.second;
					  });
	std::vector<int> out;
	out.reserve(static_cast<std::size_t>(keep));
	for (int index = 0; index < keep; ++index) {
		out.push_back(scored[static_cast<std::size_t>(index)].second);
	}
	return out;
}

Eigen::VectorXd shared_face_center(const std::vector<Interval>& lhs,
								   const std::vector<Interval>& rhs) {
	const int nd = static_cast<int>(lhs.size());
	if (nd != static_cast<int>(rhs.size())) {
		return (box_center(lhs) + box_center(rhs)) * 0.5;
	}
	Eigen::VectorXd center(nd);
	int face_dim = -1;
	for (int dim = 0; dim < nd; ++dim) {
		const double overlap_lo = std::max(lhs[static_cast<std::size_t>(dim)].lo,
										   rhs[static_cast<std::size_t>(dim)].lo);
		const double overlap_hi = std::min(lhs[static_cast<std::size_t>(dim)].hi,
										   rhs[static_cast<std::size_t>(dim)].hi);
		if (overlap_hi < overlap_lo - 1e-9) {
			return (box_center(lhs) + box_center(rhs)) * 0.5;
		}
		if (std::abs(overlap_hi - overlap_lo) <= 1e-9) {
			if (face_dim >= 0) {
				return (box_center(lhs) + box_center(rhs)) * 0.5;
			}
			face_dim = dim;
		}
		center[dim] = 0.5 * (overlap_lo + overlap_hi);
	}
	return face_dim >= 0 ? center : (box_center(lhs) + box_center(rhs)) * 0.5;
}

bool interval_boxes_connected(const std::vector<Interval>& lhs,
							  const std::vector<Interval>& rhs,
							  double tolerance) {
	const int nd = static_cast<int>(lhs.size());
	if (nd != static_cast<int>(rhs.size())) {
		return false;
	}
	int shared_dims = 0;
	int overlap_dims = 0;
	for (int dim = 0; dim < nd; ++dim) {
		const double overlap_lo = std::max(lhs[static_cast<std::size_t>(dim)].lo,
										   rhs[static_cast<std::size_t>(dim)].lo);
		const double overlap_hi = std::min(lhs[static_cast<std::size_t>(dim)].hi,
										   rhs[static_cast<std::size_t>(dim)].hi);
		if (overlap_hi < overlap_lo - tolerance) {
			return false;
		}
		if (overlap_hi - overlap_lo < tolerance) {
			shared_dims += 1;
		} else {
			overlap_dims += 1;
		}
	}
	return shared_dims >= 1 || overlap_dims == nd;
}

bool interval_box_subset(const std::vector<Interval>& inner,
						 const std::vector<Interval>& outer,
						 double tolerance) {
	if (inner.size() != outer.size()) {
		return false;
	}
	for (std::size_t dim = 0; dim < inner.size(); ++dim) {
		if (inner[dim].lo < outer[dim].lo - tolerance ||
			inner[dim].hi > outer[dim].hi + tolerance) {
			return false;
		}
	}
	return true;
}

bool partition_counts_as_segment_edge(SegmentEdgeType type) {
	return type != SegmentEdgeType::BoxCorridor &&
	       type != SegmentEdgeType::PortalCorridor &&
	       type != SegmentEdgeType::SegmentOBBCorridor &&
	       type != SegmentEdgeType::RRTBridgeOBBCorridor &&
	       type != SegmentEdgeType::TransitionOBBCorridor;
}

bool partition_counts_as_query_repair_edge(SegmentEdgeType type) {
	return type == SegmentEdgeType::QueryBridge ||
	       type == SegmentEdgeType::SegmentOBBCorridor ||
	       type == SegmentEdgeType::RRTBridgeOBBCorridor ||
	       type == SegmentEdgeType::TransitionOBBCorridor;
}

Eigen::VectorXd transition_waypoint_toward_goal(const std::vector<Interval>& lhs,
												const std::vector<Interval>& rhs,
												const Eigen::Ref<const Eigen::VectorXd>& from,
												const Eigen::Ref<const Eigen::VectorXd>& goal,
												double tolerance) {
	const int nd = static_cast<int>(lhs.size());
	if (nd != static_cast<int>(rhs.size()) || from.size() != nd) {
		return shared_face_center(lhs, rhs);
	}
	if (!interval_boxes_connected(lhs, rhs, tolerance)) {
		return shared_face_center(lhs, rhs);
	}
	Eigen::VectorXd target;
	if (goal.size() == nd) {
		target = goal;
	} else {
		target = box_center(rhs);
	}
	Eigen::VectorXd overlap_mid(nd);
	Eigen::VectorXd overlap_lo(nd);
	Eigen::VectorXd overlap_hi(nd);
	for (int dim = 0; dim < nd; ++dim) {
		overlap_lo[dim] = std::max(lhs[static_cast<std::size_t>(dim)].lo,
								   rhs[static_cast<std::size_t>(dim)].lo);
		overlap_hi[dim] = std::min(lhs[static_cast<std::size_t>(dim)].hi,
								   rhs[static_cast<std::size_t>(dim)].hi);
		overlap_mid[dim] = 0.5 * (overlap_lo[dim] + overlap_hi[dim]);
	}
	const Eigen::VectorXd local_delta = target - from;
	const double denom = local_delta.squaredNorm();
	if (denom <= 1e-18) {
		return overlap_mid;
	}
	const double t = std::clamp((overlap_mid - from).dot(local_delta) / denom, 0.0, 1.0);
	Eigen::VectorXd waypoint = from + t * local_delta;
	for (int dim = 0; dim < nd; ++dim) {
		waypoint[dim] = std::clamp(waypoint[dim], overlap_lo[dim], overlap_hi[dim]);
	}
	return waypoint;
}

double distance_to_line(const Eigen::Ref<const Eigen::VectorXd>& point,
						const Eigen::Ref<const Eigen::VectorXd>& start,
						const Eigen::Ref<const Eigen::VectorXd>& goal) {
	if (point.size() != start.size() || start.size() != goal.size() || point.size() <= 0) {
		return 0.0;
	}
	const Eigen::VectorXd delta = goal - start;
	const double denom = delta.squaredNorm();
	if (denom <= 1e-18) {
		return 0.0;
	}
	const double u = std::clamp((point - start).dot(delta) / denom, 0.0, 1.0);
	return (point - (start + u * delta)).norm();
}

bool intervals_contain_point(const std::vector<Interval>& intervals,
							 const Eigen::Ref<const Eigen::VectorXd>& point,
							 double tolerance) {
	if (point.size() != static_cast<int>(intervals.size())) {
		return false;
	}
	for (int dim = 0; dim < point.size(); ++dim) {
		if (!intervals[static_cast<std::size_t>(dim)].contains(point[dim], tolerance)) {
			return false;
		}
	}
	return true;
}

std::uint64_t max_coord_for_splits(int split_count) {
	if (split_count < 0 || split_count >= 63) {
		throw std::runtime_error("AdaptiveGridPartition split count exceeds uint64 grid capacity");
	}
	return std::uint64_t{1} << split_count;
}

double box_volume_from_intervals(const std::vector<Interval>& intervals) {
	double volume = 1.0;
	for (const auto& interval : intervals) {
		volume *= std::max(0.0, interval.width());
	}
	return volume;
}

std::uint64_t packed_pair_key(int lhs, int rhs) {
	const std::uint32_t a = static_cast<std::uint32_t>(std::min(lhs, rhs));
	const std::uint32_t b = static_cast<std::uint32_t>(std::max(lhs, rhs));
	return (static_cast<std::uint64_t>(a) << 32) | static_cast<std::uint64_t>(b);
}

}  // namespace rbf::partition_detail
