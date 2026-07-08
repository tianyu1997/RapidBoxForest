#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace rbf {

using AdjacencyGraph = std::unordered_map<int, std::vector<int>>;

struct AdjacencyBuildStats {
	int boxes = 0;
	int selected_dims = 0;
	int primary_dim = -1;
	std::uint64_t candidate_pairs = 0;
	std::uint64_t exact_tests = 0;
	std::uint64_t edges = 0;
	double build_ms = 0.0;
};

}  // namespace rbf
