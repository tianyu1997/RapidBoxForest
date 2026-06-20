#include "planning_forest_adaptive_merge.h"

#include "planning_forest_adaptive_merge_grid.h"
#include "planning_forest_adaptive_merge_internal.h"
#include "planning_forest_qroot_helpers.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <string>
#include <unordered_map>

namespace rbf {
namespace {

using adaptive_merge_detail::intervals_touch_or_overlap_local;

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
