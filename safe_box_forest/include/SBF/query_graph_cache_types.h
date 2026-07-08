#pragma once

#include <SBF/box_adjacency_types.h>
#include <SBF/segment_edge_fwd.h>

#include <rbf/core.h>

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rbf {

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

}  // namespace rbf
