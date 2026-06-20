#include "planning_forest_adaptive_merge_grid.h"

#include "planning_forest_adaptive_merge_internal.h"

#include <SBF/box_graph.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <sstream>
#include <string>
#include <unordered_map>

namespace rbf {
namespace {

using adaptive_merge_detail::clip_intervals_to_domain_local;
using adaptive_merge_detail::intervals_equal_local;

struct GridCellKey {
    bool valid = false;
    int depth = 0;
    std::vector<int> split_counts;
    std::vector<std::uint64_t> coords;
};

int heap_node_depth(OracleNodeId node) {
    if (node < 0) {
        return -1;
    }
    std::uint64_t value = static_cast<std::uint64_t>(node) + 1u;
    int depth = -1;
    while (value != 0u) {
        value >>= 1u;
        depth += 1;
    }
    return depth;
}

bool heap_path_bit(OracleNodeId node, int depth, int bit_index) {
    const std::uint64_t first_at_depth = (std::uint64_t{1} << static_cast<unsigned>(depth)) - 1u;
    const std::uint64_t local = static_cast<std::uint64_t>(node) - first_at_depth;
    const int shift = depth - 1 - bit_index;
    return ((local >> static_cast<unsigned>(shift)) & std::uint64_t{1}) != 0u;
}

GridCellKey grid_key_for_heap_box(const DatabaseBoxOracle& oracle,
                                  const BoxNode& box,
                                  const std::vector<int>& depth_dimensions,
                                  double tolerance) {
    GridCellKey key;
    const int nd = box.n_dims();
    const int depth = heap_node_depth(box.tree_id);
    if (depth < 0 || nd <= 0 || static_cast<int>(depth_dimensions.size()) < depth) {
        return key;
    }
    const auto& root = oracle.root_intervals();
    if (static_cast<int>(root.size()) != nd) {
        return key;
    }
    key.depth = depth;
    key.split_counts.assign(static_cast<std::size_t>(nd), 0);
    key.coords.assign(static_cast<std::size_t>(nd), 0u);
    for (int level = 0; level < depth; ++level) {
        const int dim = depth_dimensions[static_cast<std::size_t>(level)];
        if (dim < 0 || dim >= nd) {
            key.valid = false;
            return key;
        }
        key.coords[static_cast<std::size_t>(dim)] =
            (key.coords[static_cast<std::size_t>(dim)] << 1u) |
            (heap_path_bit(box.tree_id, depth, level) ? std::uint64_t{1} : std::uint64_t{0});
        key.split_counts[static_cast<std::size_t>(dim)] += 1;
    }
    std::vector<Interval> expected = root;
    for (int dim = 0; dim < nd; ++dim) {
        const int count = key.split_counts[static_cast<std::size_t>(dim)];
        const double cell_width = std::ldexp(root[static_cast<std::size_t>(dim)].width(), -count);
        expected[static_cast<std::size_t>(dim)].lo =
            root[static_cast<std::size_t>(dim)].lo +
            static_cast<double>(key.coords[static_cast<std::size_t>(dim)]) * cell_width;
        expected[static_cast<std::size_t>(dim)].hi =
            expected[static_cast<std::size_t>(dim)].lo + cell_width;
    }
    if (!intervals_equal_local(expected, box.joint_intervals, std::max(tolerance, 1e-10))) {
        return key;
    }
    key.valid = true;
    return key;
}

std::string grid_line_key(const GridCellKey& key, int merge_dim) {
    std::ostringstream oss;
    oss << "d" << key.depth << "|m" << merge_dim << '|';
    for (std::size_t dim = 0; dim < key.coords.size(); ++dim) {
        oss << key.split_counts[dim] << ':';
        if (static_cast<int>(dim) == merge_dim) {
            oss << '*';
        } else {
            oss << key.coords[dim];
        }
        oss << ';';
    }
    return oss.str();
}

bool try_tree_sibling_merge_round(BoxOracle& oracle,
                                  std::vector<BoxNode>& boxes,
                                  double tolerance,
                                  int& merges) {
    std::unordered_map<OracleNodeId, int> index_by_node;
    index_by_node.reserve(boxes.size() * 2);
    for (int index = 0; index < static_cast<int>(boxes.size()); ++index) {
        const OracleNodeId node = boxes[static_cast<std::size_t>(index)].tree_id;
        if (node <= 0) {
            continue;
        }
        if (index_by_node.find(node) == index_by_node.end()) {
            index_by_node[node] = index;
        }
    }
    std::vector<unsigned char> removed(boxes.size(), 0);
    bool changed = false;
    const auto planning_domain = oracle.planning_intervals();
    for (int left_index = 0; left_index < static_cast<int>(boxes.size()); ++left_index) {
        if (removed[static_cast<std::size_t>(left_index)] != 0) {
            continue;
        }
        const OracleNodeId left_node = boxes[static_cast<std::size_t>(left_index)].tree_id;
        if (left_node <= 0 || left_node % 2 == 0) {
            continue;
        }
        const OracleNodeId right_node = left_node + 1;
        auto right_it = index_by_node.find(right_node);
        if (right_it == index_by_node.end()) {
            continue;
        }
        const int right_index = right_it->second;
        if (right_index < 0 || right_index >= static_cast<int>(boxes.size()) ||
            removed[static_cast<std::size_t>(right_index)] != 0) {
            continue;
        }
        const OracleNodeId parent_node = (left_node - 1) / 2;
        BoxNode& left_box = boxes[static_cast<std::size_t>(left_index)];
        BoxNode& right_box = boxes[static_cast<std::size_t>(right_index)];
        if (left_box.n_dims() != right_box.n_dims() ||
            !boxes_connected(left_box, right_box, tolerance)) {
            continue;
        }
        std::vector<Interval> hull = left_box.joint_intervals;
        for (std::size_t dim = 0; dim < hull.size(); ++dim) {
            hull[dim] = hull[dim].hull(right_box.joint_intervals[dim]);
        }
        std::vector<Interval> parent_intervals;
        try {
            parent_intervals = oracle.node_intervals(parent_node);
        } catch (...) {
            continue;
        }
        if (!clip_intervals_to_domain_local(parent_intervals, planning_domain)) {
            continue;
        }
        if (!intervals_equal_local(hull, parent_intervals, std::max(tolerance, 1e-12))) {
            continue;
        }
        left_box.joint_intervals = std::move(hull);
        left_box.tree_id = parent_node;
        left_box.parent_box_id = -1;
        left_box.root_id = left_box.id;
        left_box.seed_config = left_box.center();
        left_box.compute_volume();
        oracle.release_box(right_box.id);
        removed[static_cast<std::size_t>(right_index)] = 1;
        merges += 1;
        changed = true;
    }
    if (!changed) {
        return false;
    }
    std::vector<BoxNode> kept;
    kept.reserve(boxes.size());
    for (int index = 0; index < static_cast<int>(boxes.size()); ++index) {
        if (removed[static_cast<std::size_t>(index)] == 0) {
            kept.push_back(std::move(boxes[static_cast<std::size_t>(index)]));
        }
    }
    boxes = std::move(kept);
    return true;
}

}  // namespace

int grid_line_merge_leaf(DatabaseBoxOracle& oracle,
                         std::vector<BoxNode>& boxes,
                         double tolerance,
                         int max_rounds,
                         int& rounds) {
    rounds = 0;
    if (boxes.empty()) {
        return 0;
    }
    const auto descriptor = oracle.database().split_policy_descriptor();
    if (descriptor.depth_dimensions.empty()) {
        return 0;
    }
    int total_merges = 0;
    for (int round = 0; round < std::max(1, max_rounds); ++round) {
        bool changed = false;
        const int nd = boxes.front().n_dims();
        for (int merge_dim = 0; merge_dim < nd; ++merge_dim) {
            std::vector<GridCellKey> keys;
            keys.reserve(boxes.size());
            std::unordered_map<std::string, std::vector<int>> groups;
            groups.reserve(boxes.size() * 2);
            for (int index = 0; index < static_cast<int>(boxes.size()); ++index) {
                keys.push_back(grid_key_for_heap_box(oracle,
                                                     boxes[static_cast<std::size_t>(index)],
                                                     descriptor.depth_dimensions,
                                                     tolerance));
                const auto& key = keys.back();
                if (!key.valid) {
                    continue;
                }
                groups[grid_line_key(key, merge_dim)].push_back(index);
            }
            std::vector<unsigned char> removed(boxes.size(), 0);
            std::vector<BoxNode> additions;
            for (auto& [_, members] : groups) {
                if (members.size() <= 1) {
                    continue;
                }
                std::sort(members.begin(), members.end(), [&](int lhs, int rhs) {
                    return keys[static_cast<std::size_t>(lhs)].coords[static_cast<std::size_t>(merge_dim)] <
                           keys[static_cast<std::size_t>(rhs)].coords[static_cast<std::size_t>(merge_dim)];
                });
                std::size_t run_begin = 0;
                while (run_begin < members.size()) {
                    std::size_t run_end = run_begin + 1;
                    while (run_end < members.size()) {
                        const auto prev_coord =
                            keys[static_cast<std::size_t>(members[run_end - 1])].coords[static_cast<std::size_t>(merge_dim)];
                        const auto next_coord =
                            keys[static_cast<std::size_t>(members[run_end])].coords[static_cast<std::size_t>(merge_dim)];
                        if (next_coord != prev_coord + 1u) {
                            break;
                        }
                        ++run_end;
                    }
                    if (run_end - run_begin > 1) {
                        BoxNode merged = boxes[static_cast<std::size_t>(members[run_begin])];
                        for (std::size_t pos = run_begin + 1; pos < run_end; ++pos) {
                            const BoxNode& next = boxes[static_cast<std::size_t>(members[pos])];
                            for (std::size_t dim = 0; dim < merged.joint_intervals.size(); ++dim) {
                                merged.joint_intervals[dim] = merged.joint_intervals[dim].hull(next.joint_intervals[dim]);
                            }
                            removed[static_cast<std::size_t>(members[pos])] = 1;
                            oracle.release_box(next.id);
                            total_merges += 1;
                        }
                        removed[static_cast<std::size_t>(members[run_begin])] = 1;
                        merged.tree_id = -1;
                        merged.parent_box_id = -1;
                        merged.root_id = merged.id;
                        merged.seed_config = merged.center();
                        merged.compute_volume();
                        additions.push_back(std::move(merged));
                        changed = true;
                    }
                    run_begin = run_end;
                }
            }
            if (changed) {
                std::vector<BoxNode> kept;
                kept.reserve(boxes.size() + additions.size());
                for (int index = 0; index < static_cast<int>(boxes.size()); ++index) {
                    if (removed[static_cast<std::size_t>(index)] == 0) {
                        kept.push_back(std::move(boxes[static_cast<std::size_t>(index)]));
                    }
                }
                kept.insert(kept.end(),
                            std::make_move_iterator(additions.begin()),
                            std::make_move_iterator(additions.end()));
                boxes = std::move(kept);
            }
        }
        rounds += 1;
        if (!changed) {
            break;
        }
    }
    return total_merges;
}

int tree_sibling_merge_leaf(BoxOracle& oracle,
                            std::vector<BoxNode>& boxes,
                            double tolerance,
                            int max_rounds,
                            const std::chrono::steady_clock::time_point* deadline,
                            bool& timed_out,
                            int& rounds) {
    int merges = 0;
    rounds = 0;
    for (int round = 0; round < std::max(1, max_rounds); ++round) {
        if (deadline != nullptr && std::chrono::steady_clock::now() >= *deadline) {
            timed_out = true;
            break;
        }
        int round_merges = 0;
        const bool changed = try_tree_sibling_merge_round(oracle, boxes, tolerance, round_merges);
        merges += round_merges;
        rounds += 1;
        if (!changed) {
            break;
        }
    }
    return merges;
}

}  // namespace rbf
