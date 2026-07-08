#include <SBF/adaptive_grid_partition.h>

#include <algorithm>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "adaptive_grid_partition_options.h"

namespace rbf {

void AdaptiveGridPartition::reset_overlay_components() {
	if (static_cast<int>(cells_.size()) < partition_overlay_dsu_min_cells()) {
		overlay_parent_.clear();
		return;
	}
	overlay_parent_.resize(cells_.size());
	for (std::size_t index = 0; index < overlay_parent_.size(); ++index) {
		overlay_parent_[index] = static_cast<int>(index);
	}
	for (int cell_index = 0; cell_index < static_cast<int>(neighbor_cache_.size()); ++cell_index) {
		for (int neighbor : neighbor_cache_[static_cast<std::size_t>(cell_index)]) {
			union_overlay_components(cell_index, neighbor);
		}
	}
	for (const auto& [source_cell, edges] : overlay_edges_by_cell_) {
		for (const auto& edge : edges) {
			union_overlay_components(source_cell, edge.target_cell);
		}
	}
}

void AdaptiveGridPartition::ensure_overlay_parent_size() {
	if (static_cast<int>(cells_.size()) < partition_overlay_dsu_min_cells()) {
		overlay_parent_.clear();
		return;
	}
	if (overlay_parent_.empty() && !cells_.empty()) {
		reset_overlay_components();
		return;
	}
	const std::size_t old_size = overlay_parent_.size();
	if (old_size >= cells_.size()) {
		return;
	}
	overlay_parent_.resize(cells_.size());
	for (std::size_t index = old_size; index < overlay_parent_.size(); ++index) {
		overlay_parent_[index] = static_cast<int>(index);
	}
}

int AdaptiveGridPartition::overlay_component_root(int cell_index) const {
	if (cell_index < 0 || cell_index >= static_cast<int>(overlay_parent_.size())) {
		return -1;
	}
	int root = cell_index;
	int guard = 0;
	while (root >= 0 &&
		   root < static_cast<int>(overlay_parent_.size()) &&
		   overlay_parent_[static_cast<std::size_t>(root)] != root) {
		root = overlay_parent_[static_cast<std::size_t>(root)];
		if (++guard > static_cast<int>(overlay_parent_.size())) {
			return -1;
		}
	}
	return root;
}

int AdaptiveGridPartition::overlay_component_root_mutable(int cell_index) {
	if (cell_index < 0 || cell_index >= static_cast<int>(overlay_parent_.size())) {
		return -1;
	}
	int root = cell_index;
	while (overlay_parent_[static_cast<std::size_t>(root)] != root) {
		root = overlay_parent_[static_cast<std::size_t>(root)];
	}
	while (overlay_parent_[static_cast<std::size_t>(cell_index)] != cell_index) {
		const int next = overlay_parent_[static_cast<std::size_t>(cell_index)];
		overlay_parent_[static_cast<std::size_t>(cell_index)] = root;
		cell_index = next;
	}
	return root;
}

void AdaptiveGridPartition::union_overlay_components(int lhs_cell, int rhs_cell) {
	if (lhs_cell < 0 || rhs_cell < 0 ||
		lhs_cell >= static_cast<int>(cells_.size()) ||
		rhs_cell >= static_cast<int>(cells_.size())) {
		return;
	}
	ensure_overlay_parent_size();
	if (overlay_parent_.empty()) {
		return;
	}
	const int lhs_root = overlay_component_root_mutable(lhs_cell);
	const int rhs_root = overlay_component_root_mutable(rhs_cell);
	if (lhs_root >= 0 && rhs_root >= 0 && lhs_root != rhs_root) {
		overlay_parent_[static_cast<std::size_t>(rhs_root)] = lhs_root;
	}
}

bool AdaptiveGridPartition::same_component_with_overlay(int lhs_box_id, int rhs_box_id) const {
	if (lhs_box_id == rhs_box_id && contains_box_id(lhs_box_id)) {
		return true;
	}
	const auto lhs_it = cell_by_box_id_.find(lhs_box_id);
	const auto rhs_it = cell_by_box_id_.find(rhs_box_id);
	if (lhs_it == cell_by_box_id_.end() || rhs_it == cell_by_box_id_.end()) {
		return false;
	}
	const int lhs_cell = lhs_it->second;
	const int rhs_cell = rhs_it->second;
	if (overlay_parent_.size() == cells_.size()) {
		const int lhs_root = overlay_component_root(lhs_cell);
		const int rhs_root = overlay_component_root(rhs_cell);
		return lhs_root >= 0 && lhs_root == rhs_root;
	}
	std::queue<int> queue;
	std::unordered_set<int> seen;
	queue.push(lhs_cell);
	seen.insert(lhs_cell);
	while (!queue.empty()) {
		const int current = queue.front();
		queue.pop();
		if (current == rhs_cell) {
			return true;
		}
		for (int next : neighbor_cell_indices(current)) {
			if (seen.insert(next).second) {
				queue.push(next);
			}
		}
		const auto overlay_it = overlay_edges_by_cell_.find(current);
		if (overlay_it == overlay_edges_by_cell_.end()) {
			continue;
		}
		for (const auto& edge : overlay_it->second) {
			if (edge.target_cell >= 0 &&
				edge.target_cell < static_cast<int>(cells_.size()) &&
				seen.insert(edge.target_cell).second) {
				queue.push(edge.target_cell);
			}
		}
	}
	return false;
}

int AdaptiveGridPartition::component_count_with_overlay() const {
	if (cells_.empty()) {
		return 0;
	}
	if (overlay_parent_.size() == cells_.size()) {
		std::unordered_set<int> roots;
		roots.reserve(cells_.size());
		for (int cell = 0; cell < static_cast<int>(cells_.size()); ++cell) {
			const int root = overlay_component_root(cell);
			if (root >= 0) {
				roots.insert(root);
			}
		}
		return static_cast<int>(roots.size());
	}
	std::vector<unsigned char> seen(cells_.size(), 0);
	int components = 0;
	for (int start = 0; start < static_cast<int>(cells_.size()); ++start) {
		if (seen[static_cast<std::size_t>(start)] != 0) {
			continue;
		}
		++components;
		std::queue<int> queue;
		queue.push(start);
		seen[static_cast<std::size_t>(start)] = 1;
		while (!queue.empty()) {
			const int current = queue.front();
			queue.pop();
			for (int next : neighbor_cell_indices(current)) {
				if (next >= 0 &&
					next < static_cast<int>(cells_.size()) &&
					seen[static_cast<std::size_t>(next)] == 0) {
					seen[static_cast<std::size_t>(next)] = 1;
					queue.push(next);
				}
			}
			const auto overlay_it = overlay_edges_by_cell_.find(current);
			if (overlay_it == overlay_edges_by_cell_.end()) {
				continue;
			}
			for (const auto& edge : overlay_it->second) {
				const int next = edge.target_cell;
				if (next >= 0 &&
					next < static_cast<int>(cells_.size()) &&
					seen[static_cast<std::size_t>(next)] == 0) {
					seen[static_cast<std::size_t>(next)] = 1;
					queue.push(next);
				}
			}
		}
	}
	return components;
}

std::vector<std::vector<int>> AdaptiveGridPartition::component_box_ids_with_overlay() const {
	const auto cell_components = component_cell_indices_with_overlay();
	std::vector<std::vector<int>> components;
	components.reserve(cell_components.size());
	for (const auto& cell_component : cell_components) {
		std::vector<int> ids;
		ids.reserve(cell_component.size());
		for (int cell_index : cell_component) {
			if (cell_index >= 0 && cell_index < static_cast<int>(cells_.size())) {
				ids.push_back(cells_[static_cast<std::size_t>(cell_index)].box_id);
			}
		}
		components.push_back(std::move(ids));
	}
	std::sort(components.begin(), components.end(), [](const auto& lhs, const auto& rhs) {
		return lhs.size() > rhs.size();
	});
	return components;
}

std::vector<std::vector<int>> AdaptiveGridPartition::component_cell_indices_with_overlay() const {
	std::vector<std::vector<int>> components;
	if (cells_.empty()) {
		return components;
	}
	if (overlay_parent_.size() == cells_.size()) {
		std::unordered_map<int, std::vector<int>> by_root;
		by_root.reserve(cells_.size());
		for (int cell = 0; cell < static_cast<int>(cells_.size()); ++cell) {
			const int root = overlay_component_root(cell);
			if (root >= 0) {
				by_root[root].push_back(cell);
			}
		}
		components.reserve(by_root.size());
		for (auto& [_, ids] : by_root) {
			components.push_back(std::move(ids));
		}
		std::sort(components.begin(), components.end(), [](const auto& lhs, const auto& rhs) {
			return lhs.size() > rhs.size();
		});
		return components;
	}

	std::vector<unsigned char> seen(cells_.size(), 0);
	for (int start = 0; start < static_cast<int>(cells_.size()); ++start) {
		if (seen[static_cast<std::size_t>(start)] != 0) {
			continue;
		}
		std::vector<int> component;
		std::queue<int> queue;
		queue.push(start);
		seen[static_cast<std::size_t>(start)] = 1;
		while (!queue.empty()) {
			const int current = queue.front();
			queue.pop();
			component.push_back(current);
			for (int next : neighbor_cell_indices(current)) {
				if (next >= 0 &&
					next < static_cast<int>(cells_.size()) &&
					seen[static_cast<std::size_t>(next)] == 0) {
					seen[static_cast<std::size_t>(next)] = 1;
					queue.push(next);
				}
			}
			const auto overlay_it = overlay_edges_by_cell_.find(current);
			if (overlay_it == overlay_edges_by_cell_.end()) {
				continue;
			}
			for (const auto& edge : overlay_it->second) {
				const int next = edge.target_cell;
				if (next >= 0 &&
					next < static_cast<int>(cells_.size()) &&
					seen[static_cast<std::size_t>(next)] == 0) {
					seen[static_cast<std::size_t>(next)] = 1;
					queue.push(next);
				}
			}
		}
		components.push_back(std::move(component));
	}
	std::sort(components.begin(), components.end(), [](const auto& lhs, const auto& rhs) {
		return lhs.size() > rhs.size();
	});
	return components;
}

std::vector<int> AdaptiveGridPartition::largest_component_box_ids_with_overlay() const {
	std::vector<int> largest;
	if (cells_.empty()) {
		return largest;
	}
	if (overlay_parent_.size() == cells_.size()) {
		std::unordered_map<int, int> island_to_component;
		std::unordered_map<int, int> overlay_to_component;
		island_to_component.reserve(cells_.size());
		overlay_to_component.reserve(cells_.size());
		std::vector<int> parent(cells_.size());
		for (int i = 0; i < static_cast<int>(parent.size()); ++i) {
			parent[static_cast<std::size_t>(i)] = i;
		}
		auto find_root = [&](int value) {
			int root = value;
			while (root >= 0 &&
				   root < static_cast<int>(parent.size()) &&
				   parent[static_cast<std::size_t>(root)] != root) {
				root = parent[static_cast<std::size_t>(root)];
			}
			while (value >= 0 &&
				   value < static_cast<int>(parent.size()) &&
				   parent[static_cast<std::size_t>(value)] != value) {
				const int next = parent[static_cast<std::size_t>(value)];
				parent[static_cast<std::size_t>(value)] = root;
				value = next;
			}
			return root;
		};
		auto unite = [&](int lhs, int rhs) {
			if (lhs < 0 || rhs < 0 ||
				lhs >= static_cast<int>(parent.size()) ||
				rhs >= static_cast<int>(parent.size())) {
				return;
			}
			const int lhs_root = find_root(lhs);
			const int rhs_root = find_root(rhs);
			if (lhs_root != rhs_root) {
				parent[static_cast<std::size_t>(rhs_root)] = lhs_root;
			}
		};
		for (int cell = 0; cell < static_cast<int>(cells_.size()); ++cell) {
			const int island = cells_[static_cast<std::size_t>(cell)].island_id;
			if (island >= 0) {
				const auto it = island_to_component.find(island);
				if (it == island_to_component.end()) {
					island_to_component.emplace(island, cell);
				} else {
					unite(it->second, cell);
				}
			}
			const int overlay_root = overlay_component_root(cell);
			if (overlay_root >= 0) {
				const auto it = overlay_to_component.find(overlay_root);
				if (it == overlay_to_component.end()) {
					overlay_to_component.emplace(overlay_root, cell);
				} else {
					unite(it->second, cell);
				}
			}
		}
		std::unordered_map<int, int> counts;
		counts.reserve(cells_.size());
		int best_root = -1;
		int best_count = 0;
		for (int cell = 0; cell < static_cast<int>(cells_.size()); ++cell) {
			const int root = find_root(cell);
			const int count = ++counts[root];
			if (count > best_count || (count == best_count && root < best_root)) {
				best_root = root;
				best_count = count;
			}
		}
		if (best_root < 0) {
			return largest;
		}
		largest.reserve(static_cast<std::size_t>(best_count));
		for (int cell = 0; cell < static_cast<int>(cells_.size()); ++cell) {
			if (find_root(cell) == best_root) {
				largest.push_back(cells_[static_cast<std::size_t>(cell)].box_id);
			}
		}
		return largest;
	}

	std::vector<unsigned char> seen(cells_.size(), 0);
	for (int start = 0; start < static_cast<int>(cells_.size()); ++start) {
		if (seen[static_cast<std::size_t>(start)] != 0) {
			continue;
		}
		std::vector<int> component;
		std::queue<int> queue;
		queue.push(start);
		seen[static_cast<std::size_t>(start)] = 1;
		while (!queue.empty()) {
			const int current = queue.front();
			queue.pop();
			component.push_back(cells_[static_cast<std::size_t>(current)].box_id);
			for (int next : neighbor_cell_indices(current)) {
				if (next >= 0 &&
					next < static_cast<int>(cells_.size()) &&
					seen[static_cast<std::size_t>(next)] == 0) {
					seen[static_cast<std::size_t>(next)] = 1;
					queue.push(next);
				}
			}
			const auto overlay_it = overlay_edges_by_cell_.find(current);
			if (overlay_it == overlay_edges_by_cell_.end()) {
				continue;
			}
			for (const auto& edge : overlay_it->second) {
				const int next = edge.target_cell;
				if (next >= 0 &&
					next < static_cast<int>(cells_.size()) &&
					seen[static_cast<std::size_t>(next)] == 0) {
					seen[static_cast<std::size_t>(next)] = 1;
					queue.push(next);
				}
			}
		}
		if (component.size() > largest.size()) {
			largest = std::move(component);
		}
	}
	return largest;
}

}  // namespace rbf
