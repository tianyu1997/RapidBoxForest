#pragma once

#include <vector>

namespace rbf {

struct DijkstraResult {
	bool found = false;
	std::vector<int> box_sequence;
	std::vector<int> segment_edge_sequence;
	double total_cost = 0.0;
};

struct QueryGraphCostOptions {
	double box_transition_penalty = 0.0;
	double box_nonprogress_penalty = 0.0;
	double box_line_deviation_penalty = 0.0;
	double query_bridge_penalty = 0.0;
	int active_query_index = -1;
	double foreign_query_edge_penalty = 0.0;
};

struct QueryShortcutCostOptions {
	bool cost_aware = true;
	double cost_factor = 1.05;
};

}  // namespace rbf
