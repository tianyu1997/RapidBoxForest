#pragma once

#include <SBF/adaptive_grid_partition_types.h>

#include <array>
#include <cstdint>
#include <vector>

namespace rbf {

struct GridMergeLineKey {
	int root_index = -1;
	int merge_dim = -1;
	std::vector<std::uint64_t> coordinates;
	bool operator==(const GridMergeLineKey& other) const noexcept {
		return root_index == other.root_index &&
			   merge_dim == other.merge_dim &&
			   coordinates == other.coordinates;
	}
};

struct GridMergeLineKeyHash {
	std::size_t operator()(const GridMergeLineKey& key) const noexcept {
		std::uint64_t hash = 1469598103934665603ull;
		auto mix = [&](std::uint64_t value) {
			hash ^= value;
			hash *= 1099511628211ull;
		};
		mix(static_cast<std::uint64_t>(key.root_index + 1));
		mix(static_cast<std::uint64_t>(key.merge_dim + 1));
		mix(static_cast<std::uint64_t>(key.coordinates.size()));
		for (std::uint64_t value : key.coordinates) {
			mix(value + 0x9e3779b97f4a7c15ull);
		}
		return static_cast<std::size_t>(hash);
	}
};

inline GridMergeLineKey make_grid_merge_line_key(const GridRange& range, int merge_dim) {
	GridMergeLineKey key;
	key.root_index = range.root_index;
	key.merge_dim = merge_dim;
	key.coordinates.reserve(range.lo.size() > 0 ? 2 * (range.lo.size() - 1) : 0);
	for (std::size_t dim = 0; dim < range.lo.size(); ++dim) {
		if (static_cast<int>(dim) == merge_dim) {
			continue;
		}
		key.coordinates.push_back(range.lo[dim]);
		key.coordinates.push_back(range.hi[dim]);
	}
	return key;
}

struct GridBroadphaseKey {
	int root_index = -1;
	std::array<std::uint64_t, 3> coord{0, 0, 0};
	bool operator==(const GridBroadphaseKey& other) const noexcept {
		return root_index == other.root_index && coord == other.coord;
	}
};

struct GridBroadphaseKeyHash {
	std::size_t operator()(const GridBroadphaseKey& key) const noexcept {
		std::uint64_t hash = 1469598103934665603ull;
		auto mix = [&](std::uint64_t value) {
			hash ^= value;
			hash *= 1099511628211ull;
		};
		mix(static_cast<std::uint64_t>(key.root_index + 1));
		for (std::uint64_t value : key.coord) {
			mix(value + 0x9e3779b97f4a7c15ull);
		}
		return static_cast<std::size_t>(hash);
	}
};

struct GridAdjacencyKey {
	int root_index = -1;
	std::array<std::uint64_t, 7> coord{0, 0, 0, 0, 0, 0, 0};
	bool operator==(const GridAdjacencyKey& other) const noexcept {
		return root_index == other.root_index && coord == other.coord;
	}
};

struct GridAdjacencyKeyHash {
	std::size_t operator()(const GridAdjacencyKey& key) const noexcept {
		std::uint64_t hash = 1469598103934665603ull;
		auto mix = [&](std::uint64_t value) {
			hash ^= value;
			hash *= 1099511628211ull;
		};
		mix(static_cast<std::uint64_t>(key.root_index + 1));
		for (std::uint64_t value : key.coord) {
			mix(value + 0x9e3779b97f4a7c15ull);
		}
		return static_cast<std::size_t>(hash);
	}
};

}  // namespace rbf
