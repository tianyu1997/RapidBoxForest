#include "planning_forest_adaptive_merge.h"

#include "planning_forest_qroot_helpers.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <string>
#include <unordered_map>

namespace rbf {
namespace {

bool intervals_equal_local(const std::vector<Interval>& lhs,
                           const std::vector<Interval>& rhs,
                           double tolerance = 0.0) {
    if (lhs.size() != rhs.size()) {
        return false;
    }
    for (std::size_t dim = 0; dim < lhs.size(); ++dim) {
        if (std::abs(lhs[dim].lo - rhs[dim].lo) > tolerance ||
            std::abs(lhs[dim].hi - rhs[dim].hi) > tolerance) {
            return false;
        }
    }
    return true;
}

bool clip_intervals_to_domain_local(std::vector<Interval>& intervals,
                                    const std::vector<Interval>& domain) {
    if (domain.empty()) {
        return !intervals.empty();
    }
    if (intervals.size() != domain.size()) {
        return false;
    }
    for (std::size_t dim = 0; dim < intervals.size(); ++dim) {
        intervals[dim].lo = std::max(intervals[dim].lo, domain[dim].lo);
        intervals[dim].hi = std::min(intervals[dim].hi, domain[dim].hi);
        if (intervals[dim].lo > intervals[dim].hi) {
            return false;
        }
    }
    return true;
}

bool intervals_touch_or_overlap_local(const Interval& lhs, const Interval& rhs, double tolerance) {
    return lhs.lo <= rhs.hi + tolerance && rhs.lo <= lhs.hi + tolerance;
}

std::string exact_face_merge_signature(const BoxNode& box, int merge_dim) {
    std::ostringstream oss;
    oss << std::setprecision(17);
    oss << static_cast<int>(box.safety_status) << '|'
        << (box.strict_audit_required ? 1 : 0) << '|';
    for (int dim = 0; dim < box.n_dims(); ++dim) {
        if (dim == merge_dim) {
            continue;
        }
        const auto& interval = box.joint_intervals[static_cast<std::size_t>(dim)];
        oss << dim << ':' << interval.lo << ',' << interval.hi << ';';
    }
    return oss.str();
}

bool fast_exact_face_merge_one_dim(BoxOracle& oracle,
                                   std::vector<BoxNode>& boxes,
                                   int merge_dim,
                                   double tolerance,
                                   int& exact_merges,
                                   const std::chrono::steady_clock::time_point* deadline = nullptr,
                                   bool* timed_out = nullptr) {
    if (boxes.empty() || merge_dim < 0 || merge_dim >= boxes.front().n_dims()) {
        return false;
    }
    std::unordered_map<std::string, std::vector<BoxNode>> groups;
    groups.reserve(boxes.size() * 2);
    for (const auto& box : boxes) {
        if (box.n_dims() != boxes.front().n_dims()) {
            continue;
        }
        groups[exact_face_merge_signature(box, merge_dim)].push_back(box);
    }

    bool changed = false;
    std::vector<BoxNode> merged_boxes;
    merged_boxes.reserve(boxes.size());
    std::vector<std::string> keys;
    keys.reserve(groups.size());
    for (const auto& item : groups) {
        keys.push_back(item.first);
    }
    std::sort(keys.begin(), keys.end());
    for (std::size_t key_index = 0; key_index < keys.size(); ++key_index) {
        const auto& key = keys[key_index];
        if (deadline != nullptr && std::chrono::steady_clock::now() >= *deadline) {
            if (timed_out != nullptr) {
                *timed_out = true;
            }
            for (std::size_t remaining = key_index; remaining < keys.size(); ++remaining) {
                auto& remaining_group = groups[keys[remaining]];
                merged_boxes.insert(merged_boxes.end(),
                                    std::make_move_iterator(remaining_group.begin()),
                                    std::make_move_iterator(remaining_group.end()));
            }
            break;
        }
        auto& group = groups[key];
        std::sort(group.begin(), group.end(), [merge_dim](const BoxNode& lhs, const BoxNode& rhs) {
            const auto& li = lhs.joint_intervals[static_cast<std::size_t>(merge_dim)];
            const auto& ri = rhs.joint_intervals[static_cast<std::size_t>(merge_dim)];
            if (std::abs(li.lo - ri.lo) > 1e-18) {
                return li.lo < ri.lo;
            }
            if (std::abs(li.hi - ri.hi) > 1e-18) {
                return li.hi < ri.hi;
            }
            return lhs.id < rhs.id;
        });
        BoxNode current = group.front();
        for (std::size_t index = 1; index < group.size(); ++index) {
            const BoxNode& next = group[index];
            auto& current_interval = current.joint_intervals[static_cast<std::size_t>(merge_dim)];
            const auto& next_interval = next.joint_intervals[static_cast<std::size_t>(merge_dim)];
            if (intervals_touch_or_overlap_local(current_interval, next_interval, tolerance)) {
                current_interval = current_interval.hull(next_interval);
                current.compute_volume();
                current.tree_id = -1;
                current.parent_box_id = -1;
                current.seed_config = current.center();
                oracle.release_box(next.id);
                exact_merges += 1;
                changed = true;
            } else {
                merged_boxes.push_back(current);
                current = next;
            }
        }
        merged_boxes.push_back(current);
    }
    boxes = std::move(merged_boxes);
    return changed;
}

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

int indexed_containment_prune_leaf(BoxOracle& oracle,
                                   std::vector<BoxNode>& boxes,
                                   double tolerance,
                                   const std::chrono::steady_clock::time_point* deadline,
                                   bool& timed_out) {
    if (boxes.size() <= 1) {
        return 0;
    }
    BoxSpatialIndex index;
    index.rebuild(boxes, tolerance);
    std::vector<unsigned char> remove(boxes.size(), 0);
    int pruned = 0;
    for (int i = 0; i < static_cast<int>(boxes.size()); ++i) {
        if (deadline != nullptr && std::chrono::steady_clock::now() >= *deadline) {
            timed_out = true;
            break;
        }
        if (remove[static_cast<std::size_t>(i)] != 0) {
            continue;
        }
        const BoxNode& inner = boxes[static_cast<std::size_t>(i)];
        const auto candidates = index.point_candidates(inner.center());
        for (int j : candidates) {
            if (j == i || j < 0 || j >= static_cast<int>(boxes.size()) ||
                remove[static_cast<std::size_t>(j)] != 0) {
                continue;
            }
            const BoxNode& outer = boxes[static_cast<std::size_t>(j)];
            if (outer.volume + 1e-18 < inner.volume) {
                continue;
            }
            if (std::abs(outer.volume - inner.volume) <= 1e-18 && outer.id > inner.id) {
                continue;
            }
            if (intervals_subset_local(inner.joint_intervals, outer.joint_intervals, tolerance)) {
                remove[static_cast<std::size_t>(i)] = 1;
                pruned += 1;
                oracle.release_box(inner.id);
                break;
            }
        }
    }
    if (pruned <= 0) {
        return 0;
    }
    std::vector<BoxNode> kept;
    kept.reserve(boxes.size() - static_cast<std::size_t>(pruned));
    for (int i = 0; i < static_cast<int>(boxes.size()); ++i) {
        if (remove[static_cast<std::size_t>(i)] == 0) {
            kept.push_back(std::move(boxes[static_cast<std::size_t>(i)]));
        }
    }
    boxes = std::move(kept);
    return pruned;
}

MergerResult fast_exact_face_merge_leaf_impl(BoxOracle& oracle,
                                             std::vector<BoxNode>& boxes,
                                             const MergerConfig& config,
                                             const std::chrono::steady_clock::time_point* deadline = nullptr,
                                             bool* timed_out = nullptr) {
    MergerResult result;
    result.boxes_before = static_cast<int>(boxes.size());
    if (boxes.empty()) {
        result.boxes_after = 0;
        return result;
    }
    const int nd = boxes.front().n_dims();
    const int max_rounds = std::max(1, config.max_rounds);
    for (int round = 0; round < max_rounds; ++round) {
        bool changed = false;
        for (int dim = 0; dim < nd; ++dim) {
            if (deadline != nullptr && std::chrono::steady_clock::now() >= *deadline) {
                if (timed_out != nullptr) {
                    *timed_out = true;
                }
                result.boxes_after = static_cast<int>(boxes.size());
                return result;
            }
            changed = fast_exact_face_merge_one_dim(oracle,
                                                    boxes,
                                                    dim,
                                                    config.adjacency_tolerance,
                                                    result.exact_merges,
                                                    deadline,
                                                    timed_out) || changed;
            if (timed_out != nullptr && *timed_out) {
                result.boxes_after = static_cast<int>(boxes.size());
                return result;
            }
        }
        result.rounds += 1;
        if (!changed) {
            break;
        }
    }
    result.boxes_after = static_cast<int>(boxes.size());
    return result;
}

}  // namespace

MergerResult fast_exact_face_merge_leaf(BoxOracle& oracle,
                                        std::vector<BoxNode>& boxes,
                                        const MergerConfig& config) {
    return fast_exact_face_merge_leaf_impl(oracle, boxes, config);
}

BudgetedMergeStats budgeted_leaf_merge(DatabaseBoxOracle& oracle,
                                       std::vector<BoxNode>& boxes,
                                       MergerConfig config,
                                       double max_merge_ms,
                                       int max_merge_rounds,
                                       int max_merge_input_boxes,
                                       double adjacency_tolerance) {
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    BudgetedMergeStats stats;
    stats.input_boxes = static_cast<int>(boxes.size());
    stats.output_boxes = stats.input_boxes;
    if (boxes.empty()) {
        stats.stop_reason = 0;
        return stats;
    }
    const bool has_budget = max_merge_ms > 0.0;
    const auto deadline = has_budget
        ? start + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double, std::milli>(max_merge_ms))
        : Clock::time_point::max();
    bool timed_out = false;
    const auto grid_start = Clock::now();
    stats.grid_merges = grid_line_merge_leaf(oracle,
                                             boxes,
                                             adjacency_tolerance,
                                             std::max(1, max_merge_rounds),
                                             stats.grid_rounds);
    stats.grid_ms = std::chrono::duration<double, std::milli>(Clock::now() - grid_start).count();
    if (has_budget && Clock::now() >= deadline) {
        stats.stop_reason = 2;
        stats.output_boxes = static_cast<int>(boxes.size());
        stats.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        return stats;
    }
    const auto tree_start = Clock::now();
    stats.tree_merges = tree_sibling_merge_leaf(oracle,
                                                boxes,
                                                adjacency_tolerance,
                                                std::max(1, max_merge_rounds),
                                                has_budget ? &deadline : nullptr,
                                                timed_out,
                                                stats.tree_rounds);
    stats.tree_ms = std::chrono::duration<double, std::milli>(Clock::now() - tree_start).count();
    if (timed_out) {
        stats.stop_reason = 2;
        stats.output_boxes = static_cast<int>(boxes.size());
        stats.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        return stats;
    }
    const auto containment_start = Clock::now();
    if (config.containment_prune) {
        const double containment_budget_ms = has_budget
            ? std::min(max_merge_ms * 0.15, 100.0)
            : 0.0;
        const auto containment_deadline = has_budget
            ? start + std::chrono::duration_cast<Clock::duration>(
                          std::chrono::duration<double, std::milli>(containment_budget_ms))
            : Clock::time_point::max();
        stats.containment_pruned = indexed_containment_prune_leaf(oracle,
                                                                  boxes,
                                                                  adjacency_tolerance,
                                                                  has_budget ? &containment_deadline : nullptr,
                                                                  timed_out);
    }
    stats.containment_ms = std::chrono::duration<double, std::milli>(Clock::now() - containment_start).count();
    if (timed_out) {
        timed_out = false;
    }
    const int input_cap = std::max(0, max_merge_input_boxes);
    if (input_cap > 0 && static_cast<int>(boxes.size()) > input_cap) {
        stats.stop_reason = 3;
        stats.output_boxes = static_cast<int>(boxes.size());
        stats.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        return stats;
    }
    config.exact_face_merge = true;
    config.greedy_hull_merge = false;
    config.containment_prune = false;
    config.adjacency_tolerance = adjacency_tolerance;
    config.max_rounds = std::max(1, max_merge_rounds);
    const auto exact_start = Clock::now();
    bool exact_timed_out = false;
    MergerResult exact_result = fast_exact_face_merge_leaf_impl(oracle,
                                                                boxes,
                                                                config,
                                                                has_budget ? &deadline : nullptr,
                                                                &exact_timed_out);
    stats.exact_ms = std::chrono::duration<double, std::milli>(Clock::now() - exact_start).count();
    stats.exact_merges = exact_result.exact_merges;
    stats.rounds = exact_result.rounds;
    if (exact_timed_out) {
        stats.stop_reason = 2;
    }
    stats.output_boxes = static_cast<int>(boxes.size());
    stats.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    return stats;
}

}  // namespace rbf
