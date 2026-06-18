#include <SBF/safe_box_forest.h>

#include <sbf/core/joint_symmetry.h>
#include <sbf/envelope/envelope_collision.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iterator>
#include <limits>
#include <optional>
#include <queue>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "env_config.h"
#include "planning_forest_audit.h"
#include "planning_forest_diagnostics.h"
#include "planning_forest_obb.h"
#include "planning_forest_qroot_helpers.h"
#include "planning_forest_query_utils.h"
#include "virtual_sparse_ffb.h"

namespace rbf {

namespace {

using detail::env_double_list_or_empty;
using detail::env_double_or_default;
using detail::env_index_list_contains;
using detail::env_indexed_double_or_default;
using detail::env_indexed_int_or_default;
using detail::env_int_list_or_empty;
using detail::env_int_or_default;

bool collision_bracket(const Eigen::VectorXd& lhs,
                       const Eigen::VectorXd& rhs,
                       const CollisionChecker& checker,
                       int resolution,
                       Eigen::VectorXd& repair_start,
                       Eigen::VectorXd& repair_goal) {
    if (!checker.check_config(lhs) && !checker.check_config(rhs)) {
        repair_start = lhs;
        repair_goal = rhs;
        return true;
    }
    const int samples = std::max(4, resolution);
    const Eigen::VectorXd diff = rhs - lhs;
    int first_collision = -1;
    int last_collision = -1;
    for (int sample = 0; sample <= samples; ++sample) {
        const double t = static_cast<double>(sample) / static_cast<double>(samples);
        const Eigen::VectorXd q = lhs + t * diff;
        if (checker.check_config(q)) {
            if (first_collision < 0) {
                first_collision = sample;
            }
            last_collision = sample;
        }
    }
    if (first_collision <= 0 || last_collision < 0 || last_collision >= samples) {
        return false;
    }
    const double t0 = static_cast<double>(first_collision - 1) / static_cast<double>(samples);
    const double t1 = static_cast<double>(last_collision + 1) / static_cast<double>(samples);
    repair_start = lhs + t0 * diff;
    repair_goal = lhs + t1 * diff;
    return !checker.check_config(repair_start) && !checker.check_config(repair_goal);
}

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

bool intervals_overlap_local(const std::vector<Interval>& lhs,
                             const std::vector<Interval>& rhs,
                             double tolerance = 0.0) {
    if (lhs.size() != rhs.size()) {
        return false;
    }
    for (std::size_t dim = 0; dim < lhs.size(); ++dim) {
        if (!lhs[dim].overlaps(rhs[dim], tolerance)) {
            return false;
        }
    }
    return true;
}

bool intervals_contain_point_strict_local(const std::vector<Interval>& intervals,
                                          const Eigen::Ref<const Eigen::VectorXd>& point,
                                          double tolerance = 0.0) {
    if (point.size() != static_cast<int>(intervals.size())) {
        return false;
    }
    for (int dim = 0; dim < point.size(); ++dim) {
        if (point[dim] < intervals[static_cast<std::size_t>(dim)].lo - tolerance ||
            point[dim] > intervals[static_cast<std::size_t>(dim)].hi + tolerance) {
            return false;
        }
    }
    return true;
}

double box_priority_point_distance(const BoxNode& box,
                                   const std::vector<Eigen::VectorXd>& priority_points) {
    if (priority_points.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    double best = std::numeric_limits<double>::infinity();
    for (const auto& point : priority_points) {
        if (point.size() != box.n_dims()) {
            continue;
        }
        best = std::min(best, intervals_point_gap_local(box.joint_intervals, point));
    }
    return best;
}

struct PriorityPruneStats {
    int free_before = 0;
    int free_after = 0;
    int collision_before = 0;
    int collision_after = 0;
};

PriorityPruneStats prune_leaf_sweep_to_priority(LeafSweepResult& result,
                                                std::vector<BoxNode>& live_boxes,
                                                std::vector<BoxNode>& raw_boxes,
                                                const std::vector<Eigen::VectorXd>& priority_points,
                                                double radius) {
    PriorityPruneStats stats;
    stats.free_before = static_cast<int>(result.free_boxes.size());
    stats.collision_before = static_cast<int>(result.collision_boxes.size());
    if (priority_points.empty() || !(radius > 0.0)) {
        stats.free_after = stats.free_before;
        stats.collision_after = stats.collision_before;
        return stats;
    }
    auto keep_box = [&](const BoxNode& box) {
        return box_priority_point_distance(box, priority_points) <= radius;
    };
    std::vector<BoxNode> kept_free;
    kept_free.reserve(result.free_boxes.size());
    for (const auto& box : result.free_boxes) {
        if (keep_box(box)) {
            kept_free.push_back(box);
        }
    }
    std::vector<BoxNode> kept_collision;
    std::vector<std::vector<int>> kept_collision_indices;
    kept_collision.reserve(result.collision_boxes.size());
    kept_collision_indices.reserve(result.collision_box_obstacle_indices.size());
    for (std::size_t index = 0; index < result.collision_boxes.size(); ++index) {
        const auto& box = result.collision_boxes[index];
        if (!keep_box(box)) {
            continue;
        }
        kept_collision.push_back(box);
        if (index < result.collision_box_obstacle_indices.size()) {
            kept_collision_indices.push_back(result.collision_box_obstacle_indices[index]);
        } else {
            kept_collision_indices.emplace_back();
        }
    }
    result.free_boxes = std::move(kept_free);
    result.collision_boxes = std::move(kept_collision);
    result.collision_box_obstacle_indices = std::move(kept_collision_indices);
    live_boxes = result.free_boxes;
    raw_boxes = live_boxes;
    stats.free_after = static_cast<int>(result.free_boxes.size());
    stats.collision_after = static_cast<int>(result.collision_boxes.size());
    return stats;
}

Eigen::VectorXd adaptive_center_of_intervals(const std::vector<Interval>& intervals) {
    Eigen::VectorXd center(static_cast<int>(intervals.size()));
    for (int dim = 0; dim < center.size(); ++dim) {
        center[dim] = intervals[static_cast<std::size_t>(dim)].center();
    }
    return center;
}

double adaptive_interval_volume(const std::vector<Interval>& intervals) {
    double volume = 1.0;
    for (const auto& interval : intervals) {
        volume *= std::max(0.0, interval.width());
    }
    return volume;
}

int adaptive_virtual_depth(OracleNodeId node) {
    if (node <= 0) {
        return 0;
    }
    std::uint64_t value = static_cast<std::uint64_t>(node) + 1u;
    int depth = -1;
    while (value != 0u) {
        value >>= 1u;
        ++depth;
    }
    return std::max(0, depth);
}

struct AdaptiveFrontierItem {
    OracleNodeId node = -1;
    std::vector<Interval> intervals;
    int changed_dim = -1;
    int free_seed_hits = 0;
    double overlap_depth = 0.0;
    double overlap_ratio = 0.0;
    double score = 0.0;
};

struct AdaptiveDepthSnapshot {
    int depth = 0;
    int free_probe_count = 0;
    int covered_count = 0;
    int main_accessible_count = 0;
    int anchor_success_count = 0;
    int anchor_to_main_count = 0;
    int anchor_probe_attempts = 0;
    int cell_count = 0;
    int collision_count = 0;
    int island_count = 0;
    int main_island_cell_count = 0;
    double p_box_covered = 0.0;
    double p_main_accessible = 0.0;
    double main_connected_ratio = 0.0;
    double p_anchor_to_main_uncovered = 0.0;
    double probe_ms = 0.0;
    bool readiness_met = false;
    std::string stop_reason;
};

std::string adaptive_depth_snapshots_to_json(const std::vector<AdaptiveDepthSnapshot>& snapshots) {
    std::ostringstream out;
    out << '[';
    for (std::size_t i = 0; i < snapshots.size(); ++i) {
        const auto& snap = snapshots[i];
        if (i > 0) {
            out << ',';
        }
        out << '{'
            << "\"depth\":" << snap.depth
            << ",\"free_probe_count\":" << snap.free_probe_count
            << ",\"covered_count\":" << snap.covered_count
            << ",\"main_accessible_count\":" << snap.main_accessible_count
            << ",\"anchor_success_count\":" << snap.anchor_success_count
            << ",\"anchor_to_main_count\":" << snap.anchor_to_main_count
            << ",\"anchor_probe_attempts\":" << snap.anchor_probe_attempts
            << ",\"cell_count\":" << snap.cell_count
            << ",\"collision_count\":" << snap.collision_count
            << ",\"island_count\":" << snap.island_count
            << ",\"main_island_cell_count\":" << snap.main_island_cell_count
            << ",\"p_box_covered\":" << snap.p_box_covered
            << ",\"p_main_accessible\":" << snap.p_main_accessible
            << ",\"main_connected_ratio\":" << snap.main_connected_ratio
            << ",\"p_anchor_to_main_uncovered\":" << snap.p_anchor_to_main_uncovered
            << ",\"probe_ms\":" << snap.probe_ms
            << ",\"readiness_met\":" << (snap.readiness_met ? "true" : "false")
            << ",\"stop_reason\":\"" << snap.stop_reason << "\""
            << '}';
    }
    out << ']';
    return out.str();
}

bool adaptive_virtual_split_node(const lect_database::SplitPolicyDescriptor& descriptor,
                                 const AdaptiveFrontierItem& item,
                                 AdaptiveFrontierItem& left,
                                 AdaptiveFrontierItem& right) {
    if (item.node < 0 || item.intervals.empty()) {
        return false;
    }
    const int depth = adaptive_virtual_depth(item.node);
    int split_dim = -1;
    if (!descriptor.depth_dimensions.empty() &&
        depth >= 0 &&
        depth < static_cast<int>(descriptor.depth_dimensions.size())) {
        split_dim = descriptor.depth_dimensions[static_cast<std::size_t>(depth)];
    } else {
        split_dim = depth % static_cast<int>(item.intervals.size());
    }
    if (split_dim < 0 || split_dim >= static_cast<int>(item.intervals.size())) {
        return false;
    }
    const auto dim = static_cast<std::size_t>(split_dim);
    const double split_value = item.intervals[dim].center();
    if (!(split_value > item.intervals[dim].lo && split_value < item.intervals[dim].hi)) {
        return false;
    }
    left = item;
    right = item;
    left.node = static_cast<OracleNodeId>(2 * static_cast<std::uint64_t>(item.node) + 1u);
    right.node = static_cast<OracleNodeId>(2 * static_cast<std::uint64_t>(item.node) + 2u);
    left.changed_dim = split_dim;
    right.changed_dim = split_dim;
    left.intervals[dim].hi = split_value;
    right.intervals[dim].lo = split_value;
    left.free_seed_hits = 0;
    right.free_seed_hits = 0;
    left.overlap_depth = 0.0;
    right.overlap_depth = 0.0;
    left.overlap_ratio = 0.0;
    right.overlap_ratio = 0.0;
    left.score = 0.0;
    right.score = 0.0;
    return true;
}

BoxNode adaptive_make_box_from_intervals(const std::vector<Interval>& intervals,
                                         OracleNodeId node,
                                         int id,
                                         BoxSafetyStatus status,
                                         bool strict_audit_required) {
    BoxNode box;
    box.id = id;
    box.joint_intervals = intervals;
    box.seed_config = adaptive_center_of_intervals(intervals);
    box.tree_id = node;
    box.parent_box_id = -1;
    box.root_id = id;
    box.safety_status = status;
    box.strict_audit_required = strict_audit_required;
    box.compute_volume();
    return box;
}

std::unordered_set<int> adaptive_largest_island_ids(const AdjacencyGraph& graph) {
    std::unordered_set<int> ids;
    const auto islands = find_islands(graph);
    if (islands.empty()) {
        return ids;
    }
    const auto* largest = &islands.front();
    for (const auto& island : islands) {
        if (island.size() > largest->size()) {
            largest = &island;
        }
    }
    ids.insert(largest->begin(), largest->end());
    return ids;
}

bool adaptive_has_adjacency_to_any(const std::vector<BoxNode>& boxes,
                                   const BoxNode& candidate,
                                   const std::unordered_set<int>* allowed_ids,
                                   double tolerance) {
    for (const auto& existing : boxes) {
        if (allowed_ids != nullptr && allowed_ids->find(existing.id) == allowed_ids->end()) {
            continue;
        }
        if (boxes_connected(existing, candidate, tolerance)) {
            return true;
        }
    }
    return false;
}

struct AdaptiveConnectivityDominance {
    int adjacent_free = 0;
    int adjacent_main = 0;
    int adjacent_other = 0;
    bool has_free_context = false;
    bool connector_candidate = false;
    bool single_component = false;
    bool isolated = true;
    double priority_delta = 0.0;
};

AdaptiveConnectivityDominance adaptive_connectivity_dominance(
    const std::vector<BoxNode>& boxes,
    const AdaptiveFrontierItem& item,
    const std::unordered_set<int>& main_ids,
    double tolerance) {
    AdaptiveConnectivityDominance out;
    out.has_free_context = !boxes.empty();
    if (!out.has_free_context) {
        return out;
    }
    const BoxNode candidate = adaptive_make_box_from_intervals(item.intervals,
                                                              item.node,
                                                              -1,
                                                              BoxSafetyStatus::Unknown,
                                                              false);
    for (const auto& box : boxes) {
        if (!boxes_connected(box, candidate, tolerance)) {
            continue;
        }
        out.adjacent_free += 1;
        if (main_ids.find(box.id) != main_ids.end()) {
            out.adjacent_main += 1;
        } else {
            out.adjacent_other += 1;
        }
    }
    out.connector_candidate = out.adjacent_main > 0 && out.adjacent_other > 0;
    out.single_component = out.adjacent_free > 0 && !out.connector_candidate;
    out.isolated = out.adjacent_free == 0;
    if (out.connector_candidate) {
        out.priority_delta = 60.0;
    } else if (out.adjacent_main > 0) {
        out.priority_delta = 18.0;
    } else if (out.single_component) {
        out.priority_delta = -4.0;
    } else {
        out.priority_delta = -12.0;
    }
    return out;
}

double adaptive_frontier_score(const std::vector<BoxNode>& boxes,
                               const AdaptiveFrontierItem& item,
                               const std::unordered_set<int>& main_ids,
                               double overlap_depth_threshold,
                               double tolerance) {
    const double volume = std::max(adaptive_interval_volume(item.intervals), 1e-300);
    const AdaptiveConnectivityDominance dominance =
        adaptive_connectivity_dominance(boxes, item, main_ids, tolerance);
    const double normalized_overlap_depth =
        overlap_depth_threshold > 1e-12
            ? std::max(0.0, item.overlap_depth / overlap_depth_threshold)
            : 0.0;
    return std::log(volume) +
           75.0 * static_cast<double>(item.free_seed_hits) -
           (item.free_seed_hits > 0 ? 0.0 : (dominance.has_free_context ? 0.0 : 6.0)) +
           dominance.priority_delta -
           3.0 * std::max(0.0, item.overlap_ratio) -
           1.5 * normalized_overlap_depth -
           0.10 * static_cast<double>(adaptive_virtual_depth(item.node));
}

std::vector<Eigen::VectorXd> adaptive_generate_free_probes(DatabaseBoxOracle& oracle,
                                                           const std::vector<Interval>& domain,
                                                           int probe_count,
                                                           int rng_seed,
                                                           int& attempted) {
    attempted = std::max(0, probe_count);
    std::vector<Eigen::VectorXd> free_points;
    if (probe_count <= 0 || domain.empty()) {
        return free_points;
    }
    std::mt19937 rng(static_cast<std::uint32_t>(rng_seed));
    std::vector<std::uniform_real_distribution<double>> distributions;
    distributions.reserve(domain.size());
    for (const auto& interval : domain) {
        distributions.emplace_back(interval.lo, interval.hi);
    }
    for (int index = 0; index < probe_count; ++index) {
        Eigen::VectorXd point(static_cast<int>(domain.size()));
        for (int dim = 0; dim < point.size(); ++dim) {
            point[dim] = distributions[static_cast<std::size_t>(dim)](rng);
        }
        if (!oracle.point_in_collision(point)) {
            free_points.push_back(std::move(point));
        }
    }
    return free_points;
}

int adaptive_count_seed_hits(const AdaptiveFrontierItem& item,
                             const std::vector<Eigen::VectorXd>& free_probes) {
    int hits = 0;
    for (const auto& point : free_probes) {
        if (intervals_contain_point_strict_local(item.intervals, point, 0.0)) {
            ++hits;
        }
    }
    return hits;
}

void adaptive_add_depth_counter(std::unordered_map<std::string, double>& diagnostics,
                                const std::string& prefix,
                                int depth) {
    diagnostics[prefix + std::to_string(depth)] += 1.0;
}

class ScopedAdaptiveFullOverlapStats {
public:
    ScopedAdaptiveFullOverlapStats(DatabaseBoxOracle& oracle, bool enabled)
        : oracle_(oracle), previous_(oracle.validation_config().collect_full_overlap_stats) {
        if (enabled) {
            oracle_.set_collect_full_overlap_stats(true);
        }
    }

    ~ScopedAdaptiveFullOverlapStats() {
        oracle_.set_collect_full_overlap_stats(previous_);
    }

    ScopedAdaptiveFullOverlapStats(const ScopedAdaptiveFullOverlapStats&) = delete;
    ScopedAdaptiveFullOverlapStats& operator=(const ScopedAdaptiveFullOverlapStats&) = delete;

private:
    DatabaseBoxOracle& oracle_;
    bool previous_ = false;
};

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

MergerResult fast_exact_face_merge_leaf(BoxOracle& oracle,
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

struct BudgetedMergeStats {
    int input_boxes = 0;
    int output_boxes = 0;
    int grid_merges = 0;
    int grid_rounds = 0;
    int tree_merges = 0;
    int tree_rounds = 0;
    int containment_pruned = 0;
    int exact_merges = 0;
    int rounds = 0;
    int stop_reason = 1;  // 0 skipped, 1 complete, 2 time_budget, 3 input_cap
    double containment_ms = 0.0;
    double grid_ms = 0.0;
    double tree_ms = 0.0;
    double exact_ms = 0.0;
    double total_ms = 0.0;
};

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
                keys.push_back(grid_key_for_heap_box(oracle, boxes[static_cast<std::size_t>(index)], descriptor.depth_dimensions, tolerance));
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
                        const auto prev_coord = keys[static_cast<std::size_t>(members[run_end - 1])].coords[static_cast<std::size_t>(merge_dim)];
                        const auto next_coord = keys[static_cast<std::size_t>(members[run_end])].coords[static_cast<std::size_t>(merge_dim)];
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
    MergerResult exact_result = fast_exact_face_merge_leaf(oracle,
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

struct QueryRootPair {
    Eigen::VectorXd start;
    Eigen::VectorXd goal;
    int start_box_id = -1;
    int goal_box_id = -1;
    int start_frontier_box_id = -1;
    int goal_frontier_box_id = -1;
    int attempts = 0;
    bool grow_from_start = true;
};

std::vector<QueryRootPair> make_query_root_pairs(const std::vector<Eigen::VectorXd>& priority_points) {
    std::vector<QueryRootPair> pairs;
    if (priority_points.size() >= 5) {
        for (std::size_t index = 0; index + 4 < priority_points.size(); index += 5) {
            QueryRootPair pair;
            pair.start = priority_points[index];
            pair.goal = priority_points[index + 4];
            pairs.push_back(std::move(pair));
        }
        return pairs;
    }
    for (std::size_t index = 0; index + 1 < priority_points.size(); index += 2) {
        QueryRootPair pair;
        pair.start = priority_points[index];
        pair.goal = priority_points[index + 1];
        pairs.push_back(std::move(pair));
    }
    return pairs;
}

int find_containing_domain_index(const std::vector<BoxNode>& domains,
                                 const BoxSpatialIndex& domain_index,
                                 const Eigen::Ref<const Eigen::VectorXd>& point,
                                 double tolerance) {
    auto candidates = domain_index.point_candidates(point);
    if (candidates.empty()) {
        candidates.reserve(domains.size());
        for (int index = 0; index < static_cast<int>(domains.size()); ++index) {
            candidates.push_back(index);
        }
    }
    int best = -1;
    double best_volume = std::numeric_limits<double>::infinity();
    for (int index : candidates) {
        if (index < 0 || index >= static_cast<int>(domains.size())) {
            continue;
        }
        const auto& domain = domains[static_cast<std::size_t>(index)];
        if (intervals_contain_point_strict_local(domain.joint_intervals, point, tolerance) &&
            domain.volume < best_volume) {
            best = index;
            best_volume = domain.volume;
        }
    }
    return best;
}

bool make_directed_face_seed(const BoxNode& source,
                             const Eigen::Ref<const Eigen::VectorXd>& target,
                             const std::vector<Interval>& root,
                             double epsilon,
                             int rank,
                             Eigen::VectorXd& seed) {
    if (source.n_dims() != target.size() || target.size() != static_cast<int>(root.size())) {
        return false;
    }
    struct Candidate {
        int dim = -1;
        int side = 0;
        double score = 0.0;
    };
    std::vector<Candidate> candidates;
    const Eigen::VectorXd center = source.center();
    for (int dim = 0; dim < source.n_dims(); ++dim) {
        const double delta = target[dim] - center[dim];
        if (std::abs(delta) <= 1e-12) {
            continue;
        }
        const int side = delta > 0.0 ? 1 : 0;
        const double value = side > 0
            ? source.joint_intervals[dim].hi + epsilon
            : source.joint_intervals[dim].lo - epsilon;
        if (!root[static_cast<std::size_t>(dim)].contains(value, 0.0)) {
            continue;
        }
        candidates.push_back({dim, side, std::abs(delta)});
    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
        if (std::abs(lhs.score - rhs.score) > 1e-18) {
            return lhs.score > rhs.score;
        }
        return lhs.dim < rhs.dim;
    });
    // For each outward face, try several points on the same face.  A pure
    // target-clamped corner seed often falls into the narrow obstacle side of a
    // collision domain; face-center and half-target variants give the local
    // FFB a chance to grow around that obstruction while still remaining
    // adjacent to the source box.
    constexpr int variants_per_face = 3;
    const int candidate_rank = rank / variants_per_face;
    const int variant = rank % variants_per_face;
    if (rank < 0 || candidate_rank < 0 || candidate_rank >= static_cast<int>(candidates.size())) {
        return false;
    }
    const Candidate& candidate = candidates[static_cast<std::size_t>(candidate_rank)];
    seed = source.center();
    for (int dim = 0; dim < source.n_dims(); ++dim) {
        if (dim == candidate.dim) {
            seed[dim] = candidate.side > 0
                ? source.joint_intervals[dim].hi + epsilon
                : source.joint_intervals[dim].lo - epsilon;
        } else {
            const double clamped_target = std::clamp(target[dim],
                                                     source.joint_intervals[dim].lo,
                                                     source.joint_intervals[dim].hi);
            if (variant == 0) {
                seed[dim] = center[dim];
            } else if (variant == 1) {
                seed[dim] = 0.5 * (center[dim] + clamped_target);
            } else {
                seed[dim] = clamped_target;
            }
        }
        seed[dim] = std::clamp(seed[dim],
                               root[static_cast<std::size_t>(dim)].lo,
                               root[static_cast<std::size_t>(dim)].hi);
    }
    return true;
}

int nearest_box_in_component(const std::vector<BoxNode>& boxes,
                             BuildDisjointSet& dsu,
                             int component_box_id,
                             const Eigen::Ref<const Eigen::VectorXd>& point) {
    if (component_box_id < 0) {
        return -1;
    }
    const int component = dsu.find(component_box_id);
    int best_id = -1;
    double best_gap = std::numeric_limits<double>::infinity();
    double best_center = std::numeric_limits<double>::infinity();
    for (const auto& box : boxes) {
        if (dsu.find(box.id) != component || box.n_dims() != point.size()) {
            continue;
        }
        const double gap = intervals_point_gap_local(box.joint_intervals, point);
        const double center_dist = (box.center() - point).squaredNorm();
        if (gap < best_gap - 1e-18 ||
            (std::abs(gap - best_gap) <= 1e-18 && center_dist < best_center)) {
            best_id = box.id;
            best_gap = gap;
            best_center = center_dist;
        }
    }
    return best_id;
}

int nearest_box_outside_component(const std::vector<BoxNode>& boxes,
                                  BuildDisjointSet& dsu,
                                  int component_box_id,
                                  const Eigen::Ref<const Eigen::VectorXd>& point) {
    if (component_box_id < 0) {
        return -1;
    }
    const int component = dsu.find(component_box_id);
    int best_id = -1;
    double best_gap = std::numeric_limits<double>::infinity();
    double best_center = std::numeric_limits<double>::infinity();
    for (const auto& box : boxes) {
        if (dsu.find(box.id) == component || box.n_dims() != point.size()) {
            continue;
        }
        const double gap = intervals_point_gap_local(box.joint_intervals, point);
        const double center_dist = (box.center() - point).squaredNorm();
        if (gap < best_gap - 1e-18 ||
            (std::abs(gap - best_gap) <= 1e-18 && center_dist < best_center)) {
            best_id = box.id;
            best_gap = gap;
            best_center = center_dist;
        }
    }
    return best_id;
}

struct OfflineAnchorGrowResult {
    int candidates_total = 0;
    int candidates_covered = 0;
    int boxes_added = 0;
    int ffb_success = 0;
    int ffb_fail = 0;
    int contained_rejects = 0;
    int domain_rejects = 0;
    int adjacency_rejects = 0;
    int commit_rejects = 0;
    int adjacency_candidates_tested = 0;
    int adjacency_edges_added = 0;
    int islands_before = 0;
    int islands_after = 0;
    double box_volume_sum = 0.0;
    double box_volume_max = 0.0;
    double index_rebuild_ms = 0.0;
    double index_query_ms = 0.0;
    double total_ms = 0.0;
};

OfflineAnchorGrowResult run_offline_anchor_grower(
    BoxOracle& oracle,
    const LeafSweepRefineConfig& refine_config,
    const std::vector<BoxNode>& collision_domains,
    const std::vector<Eigen::VectorXd>& offline_anchor_points,
    const std::function<FindFreeBoxResult(const Eigen::VectorXd&,
                                         const std::vector<Interval>&,
                                         StageContext&,
                                         const FindFreeBoxOptions&)>& find_in_domain,
    BoxCommitPolicy commit_policy,
    std::vector<BoxNode>& boxes,
    std::vector<BoxNode>& raw_boxes,
    AdjacencyGraph& graph,
    int& next_id,
    StageContext& context,
    const FindFreeBoxOptions& base_options,
    double adjacency_tolerance) {
    using Clock = std::chrono::steady_clock;
    const auto total_start = Clock::now();
    OfflineAnchorGrowResult stats;
    stats.candidates_total = static_cast<int>(offline_anchor_points.size());
    if (offline_anchor_points.empty()) {
        return stats;
    }

    BuildDisjointSet dsu = make_dsu_from_graph(boxes, graph);
    stats.islands_before = dsu.island_count();

    const auto index_start = Clock::now();
    BoxSpatialIndex box_index;
    box_index.rebuild(boxes, adjacency_tolerance);
    BoxSpatialIndex domain_index;
    domain_index.rebuild(collision_domains, adjacency_tolerance);
    stats.index_rebuild_ms += std::chrono::duration<double, std::milli>(Clock::now() - index_start).count();

    QueryRootGrowResult commit_stats;
    FindFreeBoxOptions options = base_options;
    options.max_depth = refine_config.deep_ffb_depth;
    options.reject_seed_collision = false;
    const int max_boxes = std::max(0, refine_config.deep_max_boxes);

    for (const auto& point : offline_anchor_points) {
        if (context.should_stop() || commit_stats.boxes_added >= max_boxes) {
            break;
        }
        const auto cover_start = Clock::now();
        const int owner_index = box_index.covering_box(boxes, point, adjacency_tolerance);
        commit_stats.index_query_ms +=
            std::chrono::duration<double, std::milli>(Clock::now() - cover_start).count();
        if (owner_index >= 0) {
            stats.candidates_covered += 1;
            continue;
        }
        const int domain_idx = find_containing_domain_index(collision_domains,
                                                            domain_index,
                                                            point,
                                                            adjacency_tolerance);
        if (domain_idx < 0) {
            commit_stats.domain_rejects += 1;
            continue;
        }
        const int new_id = commit_query_root_box(oracle,
                                                 options,
                                                 commit_policy,
                                                 find_in_domain,
                                                 point,
                                                 collision_domains[static_cast<std::size_t>(domain_idx)],
                                                 -1,
                                                 -1,
                                                 boxes,
                                                 raw_boxes,
                                                 graph,
                                                 box_index,
                                                 dsu,
                                                 next_id,
                                                 context,
                                                 commit_stats,
                                                 adjacency_tolerance);
        if (new_id >= 0) {
            if (const BoxNode* box = find_box_by_id(boxes, new_id)) {
                stats.box_volume_sum += box->volume;
                stats.box_volume_max = std::max(stats.box_volume_max, box->volume);
            }
        }
    }

    stats.boxes_added = commit_stats.boxes_added;
    stats.ffb_success = commit_stats.ffb_success;
    stats.ffb_fail = commit_stats.ffb_fail;
    stats.contained_rejects = commit_stats.contained_rejects;
    stats.domain_rejects = commit_stats.domain_rejects;
    stats.adjacency_rejects = commit_stats.adjacency_rejects;
    stats.commit_rejects = commit_stats.commit_rejects;
    stats.adjacency_candidates_tested = commit_stats.adjacency_candidates_tested;
    stats.adjacency_edges_added = commit_stats.adjacency_edges_added;
    stats.index_query_ms = commit_stats.index_query_ms;
    stats.islands_after = dsu.island_count();
    stats.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
    return stats;
}

QueryRootGrowResult run_query_root_box_grower(BoxOracle& oracle,
                                              const LeafSweepRefineConfig& refine_config,
                                              const std::vector<BoxNode>& collision_domains,
                                              const std::vector<Eigen::VectorXd>& priority_points,
                                              const std::function<FindFreeBoxResult(const Eigen::VectorXd&,
                                                                                   const std::vector<Interval>&,
                                                                                   StageContext&,
                                                                                   const FindFreeBoxOptions&)>& find_in_domain,
                                              BoxCommitPolicy commit_policy,
                                              std::vector<BoxNode>& boxes,
                                              std::vector<BoxNode>& raw_boxes,
                                              AdjacencyGraph& graph,
                                              int& next_id,
                                              StageContext& context,
                                              const FindFreeBoxOptions& base_options,
                                              double adjacency_tolerance) {
    using Clock = std::chrono::steady_clock;
    const auto total_start = Clock::now();
    QueryRootGrowResult stats;
    auto pairs = make_query_root_pairs(priority_points);
    stats.pairs_total = static_cast<int>(pairs.size());
    BuildDisjointSet dsu = make_dsu_from_graph(boxes, graph);
    stats.islands_before = dsu.island_count();

    const auto index_start = Clock::now();
    BoxSpatialIndex box_index;
    box_index.rebuild(boxes, adjacency_tolerance);
    BoxSpatialIndex domain_index;
    domain_index.rebuild(collision_domains, adjacency_tolerance);
    stats.index_rebuild_ms += std::chrono::duration<double, std::milli>(Clock::now() - index_start).count();

	FindFreeBoxOptions options = base_options;
	options.max_depth = refine_config.deep_ffb_depth;
	options.reject_seed_collision = false;
	const auto root = oracle.planning_intervals();
	BoxNode root_domain;
	root_domain.id = -1;
	root_domain.joint_intervals = root;
	root_domain.compute_volume();
	const double epsilon = std::max(1e-10, 0.25 * adjacency_tolerance);

    auto owner_for_point = [&](const Eigen::VectorXd& point, double tolerance) -> int {
        const auto t0 = Clock::now();
        const int owner_index = box_index.covering_box(boxes, point, tolerance);
        stats.index_query_ms += std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        return owner_index >= 0 ? boxes[static_cast<std::size_t>(owner_index)].id : -1;
    };

    auto ensure_endpoint_box = [&](const Eigen::VectorXd& point) -> int {
        int owner = owner_for_point(point, adjacency_tolerance);
        if (owner >= 0) {
            return owner;
        }
        stats.uncovered_endpoints += 1;
		const int domain_idx = find_containing_domain_index(collision_domains, domain_index, point, adjacency_tolerance);
		if (stats.boxes_added >= std::max(0, refine_config.deep_max_boxes)) {
			stats.domain_rejects += 1;
			return -1;
		}
		int box_id = -1;
		if (domain_idx >= 0) {
			const BoxNode& domain = collision_domains[static_cast<std::size_t>(domain_idx)];
			box_id = commit_query_root_box(oracle,
									 options,
									 commit_policy,
									 find_in_domain,
									 point,
									 domain,
									 -1,
									 -1,
									 boxes,
									 raw_boxes,
									 graph,
									 box_index,
									 dsu,
									 next_id,
									 context,
									 stats,
									 adjacency_tolerance);
		} else {
			stats.domain_rejects += 1;
		}
		if (box_id < 0 && stats.boxes_added < std::max(0, refine_config.deep_max_boxes)) {
			stats.endpoint_root_fallbacks += 1;
			box_id = commit_query_root_box(oracle,
										 options,
										 commit_policy,
										 find_in_domain,
										 point,
										 root_domain,
										 -1,
										 -1,
										 boxes,
										 raw_boxes,
										 graph,
										 box_index,
										 dsu,
										 next_id,
										 context,
										 stats,
										 adjacency_tolerance);
		}
		if (box_id >= 0) {
			stats.endpoint_anchors_added += 1;
		}
        return box_id;
    };

    for (auto& pair : pairs) {
        pair.start_box_id = ensure_endpoint_box(pair.start);
        pair.goal_box_id = ensure_endpoint_box(pair.goal);
        pair.start_frontier_box_id = pair.start_box_id;
        pair.goal_frontier_box_id = pair.goal_box_id;
        if (pair.start_box_id >= 0 && pair.goal_box_id >= 0 && dsu.connected(pair.start_box_id, pair.goal_box_id)) {
            stats.pairs_connected_before += 1;
        }
    }

    const int max_boxes = std::max(0, refine_config.deep_max_boxes);
    const int per_pair_attempt_cap = std::max(1, refine_config.domain_attempt_cap);
    const int max_attempts = static_cast<int>(pairs.size()) * per_pair_attempt_cap;
    bool progressed = true;
    while (!context.should_stop() &&
           progressed &&
           stats.boxes_added < max_boxes &&
           stats.pair_attempts < max_attempts) {
        progressed = false;
        for (auto& pair : pairs) {
            if (context.should_stop() ||
                stats.boxes_added >= max_boxes ||
                stats.pair_attempts >= max_attempts) {
                break;
            }
            if (pair.start_box_id < 0 || pair.goal_box_id < 0 ||
                dsu.connected(pair.start_box_id, pair.goal_box_id)) {
                continue;
            }
            if (pair.attempts >= per_pair_attempt_cap) {
                continue;
            }
            const int source_id = pair.grow_from_start ? pair.start_frontier_box_id : pair.goal_frontier_box_id;
            const int target_anchor_id = pair.grow_from_start ? pair.goal_box_id : pair.start_box_id;
            const Eigen::VectorXd& target_point = pair.grow_from_start ? pair.goal : pair.start;
            const BoxNode* source = find_box_by_id(boxes, source_id);
            if (source == nullptr) {
                continue;
            }
            int target_box_id = nearest_box_outside_component(boxes, dsu, source->id, source->center());
            if (target_box_id < 0) {
                target_box_id = nearest_box_in_component(boxes, dsu, target_anchor_id, source->center());
            }
            const BoxNode* target_box = target_box_id >= 0 ? find_box_by_id(boxes, target_box_id) : nullptr;
            const Eigen::VectorXd target = target_box != nullptr ? target_box->center() : target_point;
            bool added = false;
            const int remaining_pair_attempts = per_pair_attempt_cap - pair.attempts;
            const int local_seed_cap = std::min(std::max(1, refine_config.domain_seed_cap),
                                                std::max(1, remaining_pair_attempts));
            for (int face_rank = 0;
                 face_rank < local_seed_cap && pair.attempts < per_pair_attempt_cap;
                 ++face_rank) {
                Eigen::VectorXd seed;
                if (!make_directed_face_seed(*source, target, root, epsilon, face_rank, seed)) {
                    break;
                }
                stats.pair_attempts += 1;
                pair.attempts += 1;
                const int covered_owner = owner_for_point(seed, 0.0);
                if (covered_owner >= 0) {
                    const BoxNode* owner_box = find_box_by_id(boxes, covered_owner);
                    if (owner_box != nullptr &&
                        owner_box->id != source->id &&
                        boxes_connected(*source, *owner_box, adjacency_tolerance)) {
                        append_local_edge(graph, source->id, owner_box->id);
                        dsu.unite(source->id, owner_box->id);
                        stats.adjacency_edges_added += 1;
                        if (pair.grow_from_start) {
                            pair.start_frontier_box_id = owner_box->id;
                        } else {
                            pair.goal_frontier_box_id = owner_box->id;
                        }
                        progressed = true;
                        added = true;
                        break;
                    }
                    stats.contained_rejects += 1;
                    continue;
                }
                const int domain_idx = find_containing_domain_index(collision_domains, domain_index, seed, adjacency_tolerance);
                if (domain_idx < 0) {
                    stats.domain_rejects += 1;
                    continue;
                }
                const int new_id = commit_query_root_box(oracle,
                                                         options,
                                                         commit_policy,
                                                         find_in_domain,
                                                         seed,
                                                         collision_domains[static_cast<std::size_t>(domain_idx)],
                                                         source->id,
                                                         source->root_id >= 0 ? source->root_id : source->id,
                                                         boxes,
                                                         raw_boxes,
                                                         graph,
                                                         box_index,
                                                         dsu,
                                                         next_id,
                                                         context,
                                                         stats,
                                                         adjacency_tolerance);
                if (new_id >= 0) {
                    if (pair.grow_from_start) {
                        pair.start_frontier_box_id = new_id;
                    } else {
                        pair.goal_frontier_box_id = new_id;
                    }
                    added = true;
                    progressed = true;
                    break;
                }
                if (stats.pair_attempts >= max_attempts ||
                    stats.boxes_added >= max_boxes ||
                    context.should_stop()) {
                    break;
                }
            }
            pair.grow_from_start = !pair.grow_from_start;
            if (!added && pair.attempts >= refine_config.domain_attempt_cap * 2) {
                pair.grow_from_start = !pair.grow_from_start;
            }
        }
    }

    for (const auto& pair : pairs) {
        if (pair.start_box_id >= 0 && pair.goal_box_id >= 0 && dsu.connected(pair.start_box_id, pair.goal_box_id)) {
            stats.pairs_connected_after += 1;
        }
    }
    stats.islands_after = dsu.island_count();
    stats.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
    return stats;
}

void summarize_query_path(QueryResult& result,
                          const std::vector<BoxNode>& boxes,
                          const SegmentEdgeList& segment_edges) {
    if (result.raw_path_length <= 0.0 && result.path_length > 0.0) {
        result.raw_path_length = result.path_length;
    }
    result.segment_edge_length = 0.0;
    result.segment_edges_used = 0;
    result.obb_edges_used = 0;
    result.obb_regions_used = 0;
    result.obb_edge_length = 0.0;
    for (int edge_id : result.segment_edge_sequence) {
        if (edge_id < 0) {
            continue;
        }
        if (const SegmentEdge* edge = find_segment_edge_by_id(segment_edges, edge_id)) {
            if (counts_as_segment_edge(edge->type)) {
                result.segment_edges_used += 1;
                result.segment_edge_length +=
                    uncovered_segment_edge_length(*edge, boxes);
            }
            if (!edge->obb_centers.empty()) {
                result.obb_edges_used += 1;
                result.obb_regions_used += static_cast<int>(edge->obb_centers.size());
                result.obb_edge_length += edge->obb_covered_length > 0.0
                    ? edge->obb_covered_length
                    : edge->length;
            }
        }
    }
    int provisional_or_unknown_boxes = 0;
    for (int box_id : result.box_sequence) {
        const BoxNode* box = find_box_by_id(boxes, box_id);
        if (box == nullptr || box->safety_status != BoxSafetyStatus::CertifiedFree || box->strict_audit_required) {
            provisional_or_unknown_boxes += 1;
        }
    }
    const double box_path_length = std::max(0.0, result.path_length - result.segment_edge_length);
    const double residual_denominator =
        result.raw_path_length > 1e-12 ? result.raw_path_length : result.path_length;
    result.residual_segment_fraction =
        residual_denominator > 1e-12
            ? result.segment_edge_length / residual_denominator
            : 0.0;
    if (provisional_or_unknown_boxes == 0) {
        result.certified_box_length = box_path_length;
        result.provisional_audited_length = 0.0;
    } else {
        result.certified_box_length = 0.0;
        result.provisional_audited_length = box_path_length;
    }
    result.remaining_unsafe_assumptions = provisional_or_unknown_boxes;
}

bool try_local_birrt_repair(QueryResult& result,
                            const PathAuditCheck& audit,
                            const CollisionChecker& checker,
                            const Robot& robot,
                            const QueryConfig& query_config,
                            const RRTConnectConfig& base_repair_config,
                            int planner_seed_base) {
    if (audit.failed_segment_index < 0 || audit.failed_segment_index + 1 >= static_cast<int>(result.path.size())) {
        return false;
    }
    Eigen::VectorXd repair_start;
    Eigen::VectorXd repair_goal;
    if (!collision_bracket(result.path[static_cast<std::size_t>(audit.failed_segment_index)],
                           result.path[static_cast<std::size_t>(audit.failed_segment_index + 1)],
                           checker,
                           query_config.audit_resolution,
                           repair_start,
                           repair_goal)) {
        return false;
    }

    RRTConnectConfig repair_config = base_repair_config;
    repair_config.max_iters = std::max(repair_config.max_iters, query_config.repair_rrt_max_iters);
    if (query_config.repair_timeout_ms > 0.0) {
        repair_config.timeout_ms = query_config.repair_timeout_ms;
    }
    repair_config.segment_resolution = std::max(repair_config.segment_resolution, query_config.audit_resolution);

    const int attempts = std::max(1, query_config.repair_max_attempts);
    std::vector<Eigen::VectorXd> best_repaired;
    double best_length = std::numeric_limits<double>::infinity();
    for (int attempt = 0; attempt < attempts; ++attempt) {
        RRTConnectConfig attempt_config = repair_config;
        if (query_config.repair_local_sampling_radius > 0.0 && attempt + 1 < attempts) {
            const double growth = std::max(1.0, query_config.repair_local_sampling_growth);
            attempt_config.local_sampling_radius = query_config.repair_local_sampling_radius * std::pow(growth, attempt);
        }
        auto repair_path = rrt_connect(repair_start,
                                       repair_goal,
                                       checker,
                                       robot,
                                       attempt_config,
                                       derived_planner_seed(planner_seed_base,
                                                            kSeedRepairLocalOffset,
                                                            attempt,
                                                            0,
                                                            audit.failed_segment_index));
        if (repair_path.empty()) {
            continue;
        }
        std::vector<Eigen::VectorXd> repaired;
        repaired.reserve(result.path.size() + repair_path.size() + 2);
        for (int index = 0; index <= audit.failed_segment_index; ++index) {
            append_waypoint_unique(repaired, result.path[static_cast<std::size_t>(index)]);
        }
        append_waypoint_unique(repaired, repair_start);
        for (const auto& waypoint : repair_path) {
            append_waypoint_unique(repaired, waypoint);
        }
        append_waypoint_unique(repaired, repair_goal);
        for (std::size_t index = static_cast<std::size_t>(audit.failed_segment_index + 1); index < result.path.size(); ++index) {
            append_waypoint_unique(repaired, result.path[index]);
        }
        if (audit_waypoint_path(repaired,
                                checker,
                                query_config.audit_resolution,
                                query_config.audit_segment_step)
                .passed) {
            if (query_config.collision_shortcut && repaired.size() > 2) {
                std::vector<Eigen::VectorXd> shortened = collision_shortcut_path(
                    repaired,
                    checker,
                    collision_shortcut_resolution(query_config));
                if (audit_waypoint_path(shortened,
                                        checker,
                                        query_config.audit_resolution,
                                        query_config.audit_segment_step)
                        .passed &&
                    path_length(shortened) <= path_length(repaired) + 1e-12) {
                    repaired = std::move(shortened);
                }
            }
            const double repaired_length = path_length(repaired);
            if (repaired_length < best_length) {
                best_length = repaired_length;
                best_repaired = std::move(repaired);
            }
        }
    }
    if (!best_repaired.empty()) {
        result.path = std::move(best_repaired);
        result.path_length = best_length;
        result.repair_count += 1;
        return true;
    }
    return false;
}

}  // namespace

AdaptiveLeafSweepResult RBFPlanningForest::build_adaptive_deep_leaf_sweep_cover(
    const std::vector<Obstacle>& obstacles,
    const AdaptiveLeafSweepConfig& adaptive_config) {
    using Clock = std::chrono::steady_clock;
    const auto total_start = Clock::now();
    AdaptiveLeafSweepResult out;
    out.diagnostics["adaptive.offline_query_agnostic_build"] = 1.0;
    out.diagnostics["adaptive.qroot_pairs_total"] = 0.0;
    out.diagnostics["adaptive.qroot_uncovered_endpoints"] = 0.0;
    out.diagnostics["adaptive.fast_virtual_checkpoint_mode"] =
        adaptive_config.fast_virtual_checkpoint_mode ? 1.0 : 0.0;
    out.diagnostics["adaptive.terminal_controller_enabled"] =
        adaptive_config.fast_virtual_checkpoint_mode ? 0.0 : 1.0;

    const bool adaptive_depth_enabled = adaptive_config.adaptive_depth_enabled;
    const int adaptive_depth_min = std::max(
        adaptive_config.shallow_start_depth,
        adaptive_config.adaptive_depth_min > 0
            ? adaptive_config.adaptive_depth_min
            : adaptive_config.shallow_max_depth);
    const int adaptive_depth_max = std::max(
        adaptive_depth_min,
        adaptive_config.adaptive_depth_max > 0
            ? adaptive_config.adaptive_depth_max
            : adaptive_config.target_max_depth);
    const int initial_leaf_depth = adaptive_depth_enabled
        ? adaptive_depth_min
        : adaptive_config.shallow_max_depth;
    const int target_leaf_depth = adaptive_depth_enabled
        ? adaptive_depth_max
        : adaptive_config.target_max_depth;

    LeafSweepConfig leaf_config;
    leaf_config.obstacle_cluster_gap = adaptive_config.obstacle_cluster_gap;
    leaf_config.n_threads = std::max(1, adaptive_config.threads);
    leaf_config.validation_batch_size = std::max(1, adaptive_config.validation_batch_size);
    leaf_config.timeout_ms = adaptive_config.time_budget_ms > 0.0
        ? adaptive_config.time_budget_ms
        : 0.0;
    leaf_config.store_group_results = adaptive_config.store_group_results;
    leaf_config.use_virtual_topology = adaptive_config.use_virtual_topology;
    leaf_config.parallel_virtual_validation = adaptive_config.parallel_virtual_validation;
    leaf_config.max_free_boxes = std::max(0, adaptive_config.max_free_boxes);
    leaf_config.max_collision_boxes = std::max(0, adaptive_config.max_unresolved_domains);
    leaf_config.collision_overlap_prune_min_depth = -1;
    leaf_config.collision_overlap_prune_threshold = 0.0;
    leaf_config.collision_overlap_prune_min_threshold = 0.0;
    leaf_config.collision_overlap_prune_decay_per_depth = 0.0;
    leaf_config.collision_overlap_prune_ratio_threshold = 0.0;

    AdaptiveLeafSweepConfig partition_config = adaptive_config;
    partition_config.shallow_max_depth = initial_leaf_depth;
    partition_config.target_max_depth = target_leaf_depth;
    if (adaptive_depth_enabled || partition_config.grid_target_depth <= 0) {
        partition_config.grid_target_depth = target_leaf_depth;
    }

    auto next_fast_depth_checkpoint = [&](int depth) {
        const int step = depth < 16 ? 1 : 2;
        return std::min(target_leaf_depth, depth + step);
    };
    auto snapshot_readiness_met = [&](const AdaptiveDepthSnapshot& snapshot) {
        const int min_covered_probes = std::max(0, adaptive_config.adaptive_depth_min_covered_probes);
        const int min_main_probes = std::max(0, adaptive_config.adaptive_depth_min_main_probes);
        const int min_cells = std::max(0, adaptive_config.adaptive_depth_min_cells);
        const int min_main_cells = std::max(0, adaptive_config.adaptive_depth_min_main_cells);
        if (snapshot.cell_count <= 0 || snapshot.main_island_cell_count <= 0) {
            return false;
        }
        const bool probe_gate =
            snapshot.covered_count >= min_covered_probes &&
            snapshot.main_accessible_count >= min_main_probes &&
            (min_covered_probes <= 0 ||
             snapshot.main_connected_ratio >= adaptive_config.adaptive_depth_min_main_ratio);
        const bool cell_gate =
            snapshot.cell_count >= min_cells &&
            snapshot.main_island_cell_count >= min_main_cells;
        return probe_gate &&
               cell_gate &&
               (adaptive_config.adaptive_depth_max_online_cells <= 0 ||
                snapshot.cell_count <= adaptive_config.adaptive_depth_max_online_cells);
    };
    auto snapshot_from_fast_candidate = [&](const AdaptiveLeafSweepResult& candidate,
                                            int depth) {
        AdaptiveDepthSnapshot snapshot;
        snapshot.depth = depth;
        snapshot.free_probe_count = candidate.seed_probe_free_count;
        snapshot.covered_count = candidate.seed_probe_box_covered;
        snapshot.main_accessible_count =
            std::min(candidate.seed_probe_main_accessible, candidate.seed_probe_box_covered);
        snapshot.anchor_success_count = candidate.seed_probe_anchor_success;
        snapshot.anchor_to_main_count =
            std::max(0, candidate.seed_probe_main_accessible - snapshot.main_accessible_count);
        const auto attempts_it = candidate.profile.diagnostics.find("adaptive.seed_anchor_probe_attempts");
        if (attempts_it != candidate.profile.diagnostics.end()) {
            snapshot.anchor_probe_attempts = static_cast<int>(std::llround(attempts_it->second));
        }
        snapshot.cell_count = candidate.partition_cell_count > 0
            ? candidate.partition_cell_count
            : candidate.profile.final_boxes;
        snapshot.collision_count = candidate.shallow_collision_count;
        snapshot.island_count = candidate.partition_islands > 0
            ? candidate.partition_islands
            : candidate.profile.adjacency_islands;
        snapshot.main_island_cell_count = candidate.partition_largest_island > 0
            ? candidate.partition_largest_island
            : candidate.profile.grow_largest_island;
        const double free_den = static_cast<double>(std::max(1, snapshot.free_probe_count));
        snapshot.p_box_covered = static_cast<double>(snapshot.covered_count) / free_den;
        snapshot.p_main_accessible = static_cast<double>(snapshot.main_accessible_count) / free_den;
        snapshot.main_connected_ratio =
            static_cast<double>(snapshot.main_accessible_count) /
            static_cast<double>(std::max(1, snapshot.covered_count));
        snapshot.p_anchor_to_main_uncovered =
            static_cast<double>(snapshot.anchor_to_main_count) /
            static_cast<double>(std::max(1, snapshot.anchor_probe_attempts));
        snapshot.probe_ms = candidate.coverage_probe_ms;
        snapshot.readiness_met = snapshot_readiness_met(snapshot);
        return snapshot;
    };
    if (adaptive_depth_enabled && adaptive_config.fast_virtual_checkpoint_mode) {
        std::vector<AdaptiveDepthSnapshot> depth_snapshots;
        AdaptiveLeafSweepResult selected;
        bool have_selected = false;
        double accumulated_leaf_sweep_ms = 0.0;
        auto materialize_fast_checkpoint_candidate = [&](const LeafSweepResult& leaf_result,
                                                         int depth,
                                                         int sweep_count) {
            AdaptiveLeafSweepResult candidate;
            candidate.leaf_sweep = leaf_result;
            candidate.leaf_sweep_ms = accumulated_leaf_sweep_ms;
            candidate.selected_leaf_depth = depth;
            candidate.shallow_free_count = static_cast<int>(leaf_result.free_boxes.size());
            candidate.shallow_collision_count = static_cast<int>(leaf_result.collision_boxes.size());
            candidate.adaptive_deferred = static_cast<int>(leaf_result.collision_boxes.size());
            candidate.unresolved_domains = static_cast<int>(leaf_result.collision_boxes.size());

            scene_.set_obstacles(obstacles);
            if (oracle_) {
                oracle_->set_scene(scene_);
            }
            boxes_ = leaf_result.free_boxes;
            raw_boxes_ = boxes_;
            adjacency_.clear();
            segment_edges_.clear();
            clear_dynamic_collision_cache();
            invalidate_query_cache();
            populate_dynamic_collision_cache(leaf_result, static_cast<int>(obstacles.size()));
            reserve_existing_boxes();

            const double adjacency_tolerance = config_.query.adjacency_tolerance;
            const auto merge_start = Clock::now();
            BudgetedMergeStats merge_stats;
            if (config_.enable_merger && !boxes_.empty()) {
                bool merged_by_partition = false;
                if (adaptive_config.planning_backend == "partition_native") {
                    rebuild_adaptive_partition(partition_config, nullptr);
                    if (adaptive_partition_query_enabled_ && adaptive_partition_) {
                        AdaptiveGridPartitionMergeOptions options;
                        options.max_ms = adaptive_config.max_merge_ms;
                        options.max_rounds = adaptive_config.max_merge_rounds;
                        options.grid_line_merge = true;
                        options.containment_prune = false;
                        const auto partition_merge =
                            adaptive_partition_->merge_boxes(boxes_, options, adjacency_tolerance);
                        for (int released_id : partition_merge.released_box_ids) {
                            oracle_->release_box(released_id);
                        }
                        merge_stats.input_boxes = partition_merge.input_boxes;
                        merge_stats.output_boxes = partition_merge.output_boxes;
                        merge_stats.grid_merges = partition_merge.grid_merges;
                        merge_stats.grid_rounds = partition_merge.rounds;
                        merge_stats.containment_pruned = partition_merge.containment_pruned;
                        merge_stats.stop_reason =
                            partition_merge.stop_reason == 0 ? 1 : partition_merge.stop_reason;
                        merge_stats.total_ms = partition_merge.total_ms;
                        merge_stats.grid_ms = partition_merge.total_ms;
                        candidate.diagnostics["adaptive.partition_merge_enabled"] = 1.0;
                        candidate.diagnostics["adaptive.partition_merge_released_boxes"] =
                            static_cast<double>(partition_merge.released_box_ids.size());
                        merged_by_partition = true;
                    }
                }
                if (!merged_by_partition) {
                    MergerConfig leaf_merge_config = config_.merger;
                    leaf_merge_config.containment_prune = true;
                    merge_stats = budgeted_leaf_merge(*oracle_,
                                                      boxes_,
                                                      leaf_merge_config,
                                                      adaptive_config.max_merge_ms,
                                                      adaptive_config.max_merge_rounds,
                                                      adaptive_config.max_merge_input_boxes,
                                                      adjacency_tolerance);
                }
                raw_boxes_ = boxes_;
            } else {
                merge_stats.input_boxes = static_cast<int>(boxes_.size());
                merge_stats.output_boxes = static_cast<int>(boxes_.size());
                merge_stats.stop_reason = 0;
            }
            const double merge_ms =
                std::chrono::duration<double, std::milli>(Clock::now() - merge_start).count();

            const bool use_partition_backend = adaptive_config.planning_backend == "partition_native";
            AdjacencyBuildStats adjacency_stats;
            std::unordered_set<int> main_ids;
            int partition_island_count_for_profile = 0;
            int partition_largest_island_for_profile = 0;
            if (use_partition_backend) {
                if (!adaptive_partition_query_enabled_ || !adaptive_partition_) {
                    rebuild_adaptive_partition(partition_config, nullptr);
                }
                if (adaptive_partition_query_enabled_ && adaptive_partition_) {
                    const auto largest = adaptive_partition_->largest_component_box_ids_with_overlay();
                    main_ids.insert(largest.begin(), largest.end());
                    const auto& partition_stats = adaptive_partition_->stats();
                    partition_island_count_for_profile = partition_stats.islands;
                    partition_largest_island_for_profile = partition_stats.largest_island;
                    candidate.diagnostics["adaptive.partition_skipped_graph_adjacency"] = 1.0;
                }
            }
            if (!use_partition_backend) {
                rebuild_adjacency();
                adjacency_stats = last_adjacency_build_stats();
                main_ids = adaptive_largest_island_ids(adjacency_);
            } else if (main_ids.empty()) {
                candidate.diagnostics["adaptive.partition_missing_no_graph_fallback"] = 1.0;
            }

            const auto coverage_start = Clock::now();
            const auto planning_domain = oracle_ ? oracle_->planning_intervals() : std::vector<Interval>{};
            int probe_attempted = 0;
            std::vector<Eigen::VectorXd> free_probes =
                oracle_ ? adaptive_generate_free_probes(*oracle_,
                                                        planning_domain,
                                                        std::max(0, adaptive_config.adaptive_depth_probe_count),
                                                        adaptive_config.adaptive_depth_probe_seed,
                                                        probe_attempted)
                        : std::vector<Eigen::VectorXd>{};
            candidate.seed_probe_count = probe_attempted;
            candidate.seed_probe_free_count = static_cast<int>(free_probes.size());
            int uncovered_anchor_attempts = 0;
            StageContext probe_context = StageContext::from_runtime(config_.runtime);
            FindFreeBoxOptions probe_options = config_.grower.find_free_box;
            probe_options.max_depth = target_leaf_depth;
            probe_options.reject_seed_collision = false;
            probe_options.deadline_ms = std::max(1.0, adaptive_config.adaptive_depth_max_probe_ms);
            const int anchor_cap = std::max(0, adaptive_config.adaptive_depth_anchor_probe_cap);
            BoxSpatialIndex coverage_index;
            const bool use_partition_coverage =
                use_partition_backend && adaptive_partition_query_enabled_ && adaptive_partition_;
            if (!use_partition_coverage) {
                coverage_index.rebuild(boxes_, adjacency_tolerance);
            }
            for (const auto& point : free_probes) {
                const int owner = use_partition_coverage
                    ? adaptive_partition_->locate_containing_box(point, false, adjacency_tolerance)
                    : [&]() {
                          const int owner_index = coverage_index.covering_box(boxes_, point, adjacency_tolerance);
                          return owner_index >= 0 ? boxes_[static_cast<std::size_t>(owner_index)].id : -1;
                      }();
                if (owner >= 0) {
                    candidate.seed_probe_box_covered += 1;
                    if (main_ids.find(owner) != main_ids.end()) {
                        candidate.seed_probe_main_accessible += 1;
                    }
                    continue;
                }
                if (uncovered_anchor_attempts >= anchor_cap || planning_domain.empty()) {
                    continue;
                }
                ++uncovered_anchor_attempts;
                const auto ffb = find_free_box_in_domain(point, planning_domain, probe_context, probe_options);
                if (!ffb.found) {
                    continue;
                }
                candidate.seed_probe_anchor_success += 1;
                const BoxNode anchor = adaptive_make_box_from_intervals(
                    ffb.intervals,
                    ffb.node,
                    -1,
                    ffb.validation_detail.safety_status,
                    ffb.validation_detail.strict_audit_required);
                const bool anchor_main_accessible = use_partition_coverage
                    ? adaptive_partition_->box_adjacent_to_any(anchor, main_ids, adjacency_tolerance)
                    : (!use_partition_backend &&
                       adaptive_has_adjacency_to_any(boxes_, anchor, &main_ids, adjacency_tolerance));
                if (anchor_main_accessible) {
                    candidate.seed_probe_main_accessible += 1;
                }
            }
            candidate.coverage_probe_ms =
                std::chrono::duration<double, std::milli>(Clock::now() - coverage_start).count();
            const double free_den = static_cast<double>(std::max(1, candidate.seed_probe_free_count));
            candidate.p_box_covered = static_cast<double>(candidate.seed_probe_box_covered) / free_den;
            candidate.p_anchor_success = static_cast<double>(candidate.seed_probe_anchor_success) / free_den;
            candidate.p_main_accessible = static_cast<double>(candidate.seed_probe_main_accessible) / free_den;

            candidate.total_ms =
                std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
            candidate.profile = {};
            candidate.profile.raw_boxes = static_cast<int>(raw_boxes_.size());
            candidate.profile.final_boxes = static_cast<int>(boxes_.size());
            candidate.profile.segment_edges = static_cast<int>(segment_edges_.size());
            candidate.profile.grow_ms = candidate.total_ms;
            candidate.profile.total_ms = candidate.total_ms;
            if (use_partition_backend) {
                candidate.profile.grow_adjacency_islands = partition_island_count_for_profile;
                candidate.profile.adjacency_islands = partition_island_count_for_profile;
                candidate.profile.grow_largest_island = partition_largest_island_for_profile;
            } else {
                const auto graph_islands = find_islands(adjacency_);
                candidate.profile.grow_adjacency_islands = static_cast<int>(graph_islands.size());
                candidate.profile.adjacency_islands = candidate.profile.grow_adjacency_islands;
                for (const auto& island : graph_islands) {
                    candidate.profile.grow_largest_island =
                        std::max(candidate.profile.grow_largest_island, static_cast<int>(island.size()));
                }
            }
            candidate.profile.diagnostics = leaf_result.diagnostics;
            for (const auto& [key, value] : candidate.diagnostics) {
                candidate.profile.diagnostics[key] = value;
            }
            candidate.profile.diagnostics["adaptive.fast_checkpoint_mode"] = 1.0;
            candidate.profile.diagnostics["adaptive.fast_virtual_checkpoint_mode"] = 1.0;
            candidate.profile.diagnostics["adaptive.terminal_controller_enabled"] = 0.0;
            candidate.profile.diagnostics["adaptive.in_sweep_checkpoint_mode"] = 1.0;
            candidate.profile.diagnostics["adaptive.in_sweep_checkpoints"] =
                static_cast<double>(sweep_count);
            candidate.profile.diagnostics["adaptive.offline_query_agnostic_build"] = 1.0;
            candidate.profile.diagnostics["adaptive.qroot_pairs_total"] = 0.0;
            candidate.profile.diagnostics["adaptive.qroot_uncovered_endpoints"] = 0.0;
            candidate.profile.diagnostics["adaptive.leaf_sweep_ms"] = candidate.leaf_sweep_ms;
            candidate.profile.diagnostics["adaptive.merge_ms"] = merge_ms;
            candidate.profile.diagnostics["adaptive.merge_input_boxes"] =
                static_cast<double>(merge_stats.input_boxes);
            candidate.profile.diagnostics["adaptive.merge_output_boxes"] =
                static_cast<double>(merge_stats.output_boxes);
            candidate.profile.diagnostics["adaptive.adjacency_ms"] = adjacency_stats.build_ms;
            candidate.profile.diagnostics["adaptive.adjacency_boxes"] =
                static_cast<double>(adjacency_stats.boxes);
            candidate.profile.diagnostics["adaptive.adjacency_selected_dims"] =
                static_cast<double>(adjacency_stats.selected_dims);
            candidate.profile.diagnostics["adaptive.adjacency_primary_dim"] =
                static_cast<double>(adjacency_stats.primary_dim);
            candidate.profile.diagnostics["adaptive.adjacency_candidates"] =
                static_cast<double>(adjacency_stats.candidate_pairs);
            candidate.profile.diagnostics["adaptive.adjacency_exact_tests"] =
                static_cast<double>(adjacency_stats.exact_tests);
            candidate.profile.diagnostics["adaptive.adjacency_edges"] =
                static_cast<double>(adjacency_stats.edges);
            candidate.profile.diagnostics["adaptive.coverage_probe_ms"] = candidate.coverage_probe_ms;
            candidate.profile.diagnostics["adaptive.total_ms"] = candidate.total_ms;
            candidate.profile.diagnostics["adaptive.shallow_free_count"] =
                static_cast<double>(candidate.shallow_free_count);
            candidate.profile.diagnostics["adaptive.shallow_collision_count"] =
                static_cast<double>(candidate.shallow_collision_count);
            candidate.profile.diagnostics["adaptive.seed_probe_count"] =
                static_cast<double>(candidate.seed_probe_count);
            candidate.profile.diagnostics["adaptive.seed_probe_free_count"] =
                static_cast<double>(candidate.seed_probe_free_count);
            candidate.profile.diagnostics["adaptive.seed_probe_box_covered"] =
                static_cast<double>(candidate.seed_probe_box_covered);
            candidate.profile.diagnostics["adaptive.seed_probe_anchor_success"] =
                static_cast<double>(candidate.seed_probe_anchor_success);
            candidate.profile.diagnostics["adaptive.seed_probe_main_accessible"] =
                static_cast<double>(candidate.seed_probe_main_accessible);
            candidate.profile.diagnostics["adaptive.seed_anchor_probe_cap"] =
                static_cast<double>(anchor_cap);
            candidate.profile.diagnostics["adaptive.seed_anchor_probe_attempts"] =
                static_cast<double>(uncovered_anchor_attempts);
            candidate.profile.diagnostics["adaptive.p_box_covered"] = candidate.p_box_covered;
            candidate.profile.diagnostics["adaptive.p_anchor_success"] = candidate.p_anchor_success;
            candidate.profile.diagnostics["adaptive.p_main_accessible"] = candidate.p_main_accessible;
            rebuild_adaptive_partition(partition_config, &candidate.profile);
            if (adaptive_partition_ && !adaptive_partition_->empty()) {
                const auto& partition_stats = adaptive_partition_->stats();
                candidate.partition_cell_count = partition_stats.cells;
                candidate.partition_grid_cell_count = partition_stats.grid_cells;
                candidate.partition_non_grid_cell_count = partition_stats.non_grid_cells;
                candidate.partition_face_index_entries = partition_stats.face_index_entries;
                candidate.partition_islands = partition_stats.islands;
                candidate.partition_largest_island = partition_stats.largest_island;
                candidate.profile.grow_adjacency_islands = partition_stats.islands;
                candidate.profile.adjacency_islands = partition_stats.islands;
                candidate.profile.grow_largest_island = partition_stats.largest_island;
            }
            candidate.diagnostics = candidate.profile.diagnostics;
            return candidate;
        };
        int depth = initial_leaf_depth;
        int sweep_count = 0;
        std::vector<int> checkpoint_depths;
        for (int checkpoint = initial_leaf_depth;
             checkpoint <= target_leaf_depth;
             checkpoint = next_fast_depth_checkpoint(checkpoint)) {
            checkpoint_depths.push_back(checkpoint);
            if (checkpoint >= target_leaf_depth) {
                break;
            }
        }
        const auto adaptive_sweep_start = Clock::now();
        LeafSweepConfig checkpoint_leaf_config = leaf_config;
        checkpoint_leaf_config.checkpoint_depths = checkpoint_depths;
        checkpoint_leaf_config.checkpoint_callback = [&](const LeafSweepResult& checkpoint_leaf,
                                                         int checkpoint_depth) {
            depth = checkpoint_depth;
            ++sweep_count;
            accumulated_leaf_sweep_ms =
                std::chrono::duration<double, std::milli>(Clock::now() - adaptive_sweep_start).count();
            AdaptiveLeafSweepResult candidate =
                materialize_fast_checkpoint_candidate(checkpoint_leaf, depth, sweep_count);
            auto snapshot = snapshot_from_fast_candidate(candidate, depth);
            if (snapshot.readiness_met) {
                snapshot.stop_reason = "coverage_ready";
            } else if (depth >= target_leaf_depth) {
                snapshot.stop_reason = "max_depth";
            } else {
                snapshot.stop_reason = "checkpoint";
            }
            depth_snapshots.push_back(snapshot);

            selected = std::move(candidate);
            selected.selected_leaf_depth = depth;
            have_selected = true;
            return snapshot.readiness_met || depth >= target_leaf_depth;
        };
        out.leaf_sweep = build_leaf_sweep(obstacles,
                                          adaptive_config.shallow_start_depth,
                                          target_leaf_depth,
                                          checkpoint_leaf_config);
        if (!have_selected) {
            ++sweep_count;
            accumulated_leaf_sweep_ms =
                std::chrono::duration<double, std::milli>(Clock::now() - adaptive_sweep_start).count();
            selected = materialize_fast_checkpoint_candidate(out.leaf_sweep,
                                                             target_leaf_depth,
                                                             sweep_count);
            auto snapshot = snapshot_from_fast_candidate(selected, target_leaf_depth);
            snapshot.stop_reason = snapshot.readiness_met ? "coverage_ready" : "max_depth";
            depth_snapshots.push_back(snapshot);
            selected.selected_leaf_depth = target_leaf_depth;
            selected.adaptive_depth_readiness_met = snapshot.readiness_met;
            selected.adaptive_depth_stop_reason = snapshot.stop_reason;
            have_selected = true;
        }
        if (have_selected) {
            for (const auto& [key, value] : out.leaf_sweep.diagnostics) {
                if (key.find("worker_oracle.") != std::string::npos ||
                    key.find("external") != std::string::npos ||
                    key.find("canonical_frame") != std::string::npos) {
                    set_diagnostic_max(selected.profile.diagnostics, key, value);
                }
            }
            const auto& final_snapshot = depth_snapshots.back();
            selected.selected_leaf_depth = final_snapshot.depth;
            selected.adaptive_depth_readiness_met = final_snapshot.readiness_met;
            selected.adaptive_depth_stop_reason = final_snapshot.stop_reason;
            selected.adaptive_depth_snapshots_json =
                adaptive_depth_snapshots_to_json(depth_snapshots);
            selected.seed_probe_box_covered = final_snapshot.covered_count;
            selected.seed_probe_main_accessible =
                final_snapshot.main_accessible_count + final_snapshot.anchor_to_main_count;
            selected.p_box_covered = final_snapshot.p_box_covered;
            selected.p_main_accessible =
                static_cast<double>(selected.seed_probe_main_accessible) /
                static_cast<double>(std::max(1, final_snapshot.free_probe_count));
            selected.p_anchor_to_main_uncovered = final_snapshot.p_anchor_to_main_uncovered;
            selected.total_ms =
                std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
            selected.profile.total_ms = selected.total_ms;
            selected.profile.grow_ms = selected.total_ms;
            selected.profile.diagnostics["adaptive.in_sweep_checkpoints"] =
                static_cast<double>(sweep_count);
            selected.profile.diagnostics["adaptive.fast_checkpoint_mode"] = 1.0;
            selected.profile.diagnostics["adaptive.fast_virtual_checkpoint_mode"] = 1.0;
            selected.profile.diagnostics["adaptive.terminal_controller_enabled"] = 0.0;
            selected.profile.diagnostics["adaptive.in_sweep_checkpoint_mode"] = 1.0;
            selected.profile.diagnostics["adaptive.selected_leaf_depth"] =
                static_cast<double>(selected.selected_leaf_depth);
            selected.profile.diagnostics["adaptive.depth_readiness_met"] =
                selected.adaptive_depth_readiness_met ? 1.0 : 0.0;
            selected.profile.diagnostics["adaptive.depth_enabled"] = 1.0;
            selected.profile.diagnostics["adaptive.depth_min"] = static_cast<double>(adaptive_depth_min);
            selected.profile.diagnostics["adaptive.depth_max"] = static_cast<double>(target_leaf_depth);
            record_depth_semantics_diagnostics(selected.profile.diagnostics,
                                               "adaptive.",
                                               adaptive_config.shallow_start_depth,
                                               initial_leaf_depth,
                                               target_leaf_depth,
                                               config_.grower.find_free_box,
                                               target_leaf_depth);
            if (oracle_) {
                const OracleCounters counters = oracle_->counters();
                normalize_external_evidence_diagnostics(selected.profile.diagnostics, &counters);
            } else {
                normalize_external_evidence_diagnostics(selected.profile.diagnostics);
            }
            record_portal_membership_policy(selected.profile.diagnostics, config_.portal_membership_policy);
            selected.diagnostics = selected.profile.diagnostics;
            last_build_ = selected.profile;
            if (config_.database.checkpoint_after_build && database_) {
                database_->checkpoint();
            }
            return selected;
        }
    }

    if (adaptive_config.node_budget <= 0 && !adaptive_depth_enabled) {
        out.diagnostics["adaptive.fixed_virtual_layer_mode"] = 1.0;
        out.diagnostics["adaptive.terminal_controller_enabled"] = 0.0;
        leaf_config.collision_overlap_prune_min_depth = adaptive_config.defer_min_depth;
        leaf_config.collision_overlap_prune_threshold = adaptive_config.overlap_depth_threshold;
        leaf_config.collision_overlap_prune_min_threshold = adaptive_config.overlap_depth_min_threshold;
        leaf_config.collision_overlap_prune_decay_per_depth = adaptive_config.overlap_depth_decay_per_depth;
        leaf_config.collision_overlap_prune_ratio_threshold = adaptive_config.overlap_ratio_threshold;
        out.leaf_sweep = build_leaf_sweep(obstacles,
                                          adaptive_config.shallow_start_depth,
                                          target_leaf_depth,
                                          leaf_config);
        out.selected_leaf_depth = target_leaf_depth;
        out.adaptive_depth_readiness_met = false;
        out.adaptive_depth_stop_reason = "fixed_depth";
        out.leaf_sweep_ms = out.leaf_sweep.total_ms;
        out.shallow_free_count = static_cast<int>(out.leaf_sweep.free_boxes.size());
        out.shallow_collision_count = static_cast<int>(out.leaf_sweep.collision_boxes.size());
        out.adaptive_deferred = static_cast<int>(out.leaf_sweep.collision_boxes.size());
        out.unresolved_domains = static_cast<int>(out.leaf_sweep.collision_boxes.size());

        const double adjacency_tolerance = config_.query.adjacency_tolerance;
        const auto merge_start = Clock::now();
        BudgetedMergeStats merge_stats;
        if (config_.enable_merger && !boxes_.empty()) {
            bool merged_by_partition = false;
            if (adaptive_config.planning_backend == "partition_native") {
                rebuild_adaptive_partition(partition_config, nullptr);
                if (adaptive_partition_query_enabled_ && adaptive_partition_) {
                    AdaptiveGridPartitionMergeOptions options;
                    options.max_ms = adaptive_config.max_merge_ms;
                    options.max_rounds = adaptive_config.max_merge_rounds;
                    options.grid_line_merge = true;
                    options.containment_prune = false;
                    const auto partition_merge =
                        adaptive_partition_->merge_boxes(boxes_, options, adjacency_tolerance);
                    for (int released_id : partition_merge.released_box_ids) {
                        oracle_->release_box(released_id);
                    }
                    merge_stats.input_boxes = partition_merge.input_boxes;
                    merge_stats.output_boxes = partition_merge.output_boxes;
                    merge_stats.grid_merges = partition_merge.grid_merges;
                    merge_stats.grid_rounds = partition_merge.rounds;
                    merge_stats.containment_pruned = partition_merge.containment_pruned;
                    merge_stats.stop_reason = partition_merge.stop_reason == 0 ? 1 : partition_merge.stop_reason;
                    merge_stats.total_ms = partition_merge.total_ms;
                    merge_stats.grid_ms = partition_merge.total_ms;
                    out.diagnostics["adaptive.partition_merge_enabled"] = 1.0;
                    out.diagnostics["adaptive.partition_merge_released_boxes"] =
                        static_cast<double>(partition_merge.released_box_ids.size());
                    out.diagnostics["adaptive.partition_merge_containment_skipped"] =
                        static_cast<double>(partition_merge.containment_skipped);
                    out.diagnostics["adaptive.partition_merge_containment_bucket_entries"] =
                        static_cast<double>(partition_merge.containment_bucket_entries);
                    out.diagnostics["adaptive.partition_merge_containment_candidates"] =
                        static_cast<double>(partition_merge.containment_candidates);
                    out.diagnostics["adaptive.partition_merge_containment_tests"] =
                        static_cast<double>(partition_merge.containment_tests);
                    out.diagnostics["adaptive.partition_merge_containment_overflow"] =
                        static_cast<double>(partition_merge.containment_overflow);
                    out.diagnostics["adaptive.partition_merge_containment_ms"] =
                        partition_merge.containment_ms;
                    out.diagnostics["adaptive.partition_merge_line_ms"] =
                        partition_merge.line_merge_ms;
                    merged_by_partition = true;
                }
            }
            if (!merged_by_partition) {
                MergerConfig leaf_merge_config = config_.merger;
                leaf_merge_config.containment_prune = true;
                merge_stats = budgeted_leaf_merge(*oracle_,
                                                  boxes_,
                                                  leaf_merge_config,
                                                  adaptive_config.max_merge_ms,
                                                  adaptive_config.max_merge_rounds,
                                                  adaptive_config.max_merge_input_boxes,
                                                  adjacency_tolerance);
            }
            raw_boxes_ = boxes_;
        } else {
            merge_stats.input_boxes = static_cast<int>(boxes_.size());
            merge_stats.output_boxes = static_cast<int>(boxes_.size());
            merge_stats.stop_reason = 0;
        }
        const double merge_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - merge_start).count();
        const bool use_partition_backend = adaptive_config.planning_backend == "partition_native";
        AdjacencyBuildStats adjacency_stats;
        std::unordered_set<int> main_ids;
        int partition_island_count_for_profile = 0;
        int partition_largest_island_for_profile = 0;
        if (use_partition_backend) {
            if (!adaptive_partition_query_enabled_ || !adaptive_partition_) {
                rebuild_adaptive_partition(partition_config, nullptr);
            }
            if (adaptive_partition_query_enabled_ && adaptive_partition_) {
                const auto largest = adaptive_partition_->largest_component_box_ids_with_overlay();
                main_ids.insert(largest.begin(), largest.end());
                const auto& partition_stats = adaptive_partition_->stats();
                partition_island_count_for_profile = partition_stats.islands;
                partition_largest_island_for_profile = partition_stats.largest_island;
                out.diagnostics["adaptive.partition_skipped_graph_adjacency"] = 1.0;
            }
        }
        if (!use_partition_backend) {
            rebuild_adjacency();
            adjacency_stats = last_adjacency_build_stats();
            main_ids = adaptive_largest_island_ids(adjacency_);
        } else if (main_ids.empty()) {
            out.diagnostics["adaptive.partition_missing_no_graph_fallback"] = 1.0;
        }

        const auto coverage_start = Clock::now();
        const auto planning_domain = oracle_ ? oracle_->planning_intervals() : std::vector<Interval>{};
        int probe_attempted = 0;
        std::vector<Eigen::VectorXd> free_probes =
            oracle_ ? adaptive_generate_free_probes(*oracle_,
                                                    planning_domain,
                                                    adaptive_config.seed_probe_count,
                                                    adaptive_config.seed_probe_rng_seed,
                                                    probe_attempted)
                    : std::vector<Eigen::VectorXd>{};
        out.seed_probe_count = probe_attempted;
        out.seed_probe_free_count = static_cast<int>(free_probes.size());
        int uncovered_anchor_attempts = 0;
        StageContext probe_context = StageContext::from_runtime(config_.runtime);
        FindFreeBoxOptions probe_options = config_.grower.find_free_box;
        probe_options.max_depth = target_leaf_depth;
        probe_options.reject_seed_collision = false;
        probe_options.deadline_ms = 5.0;
        const int anchor_cap = std::max(0, adaptive_config.seed_anchor_probe_cap);
	        BoxSpatialIndex coverage_index;
	        const bool use_partition_coverage =
	            use_partition_backend && adaptive_partition_query_enabled_ && adaptive_partition_;
	        if (!use_partition_coverage) {
	            coverage_index.rebuild(boxes_, adjacency_tolerance);
	        }
	        for (const auto& point : free_probes) {
	            const int owner = use_partition_coverage
	                ? adaptive_partition_->locate_containing_box(point, false, adjacency_tolerance)
	                : [&]() {
	                      const int owner_index = coverage_index.covering_box(boxes_, point, adjacency_tolerance);
	                      return owner_index >= 0 ? boxes_[static_cast<std::size_t>(owner_index)].id : -1;
	                  }();
	            if (owner >= 0) {
	                out.seed_probe_box_covered += 1;
	                if (main_ids.find(owner) != main_ids.end()) {
	                    out.seed_probe_main_accessible += 1;
                }
                continue;
            }
            if (uncovered_anchor_attempts >= anchor_cap || planning_domain.empty()) {
                continue;
            }
            ++uncovered_anchor_attempts;
            const auto ffb = find_free_box_in_domain(point, planning_domain, probe_context, probe_options);
            if (!ffb.found) {
                continue;
            }
            out.seed_probe_anchor_success += 1;
	            const BoxNode anchor = adaptive_make_box_from_intervals(ffb.intervals,
	                                                                    ffb.node,
	                                                                    -1,
	                                                                    ffb.validation_detail.safety_status,
	                                                                    ffb.validation_detail.strict_audit_required);
            const bool anchor_main_accessible = use_partition_coverage
                ? adaptive_partition_->box_adjacent_to_any(anchor, main_ids, adjacency_tolerance)
                : (!use_partition_backend &&
                   adaptive_has_adjacency_to_any(boxes_, anchor, &main_ids, adjacency_tolerance));
	            if (anchor_main_accessible) {
	                out.seed_probe_main_accessible += 1;
	            }
	        }
        out.coverage_probe_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - coverage_start).count();
        const double free_den = static_cast<double>(std::max(1, out.seed_probe_free_count));
        out.p_box_covered = static_cast<double>(out.seed_probe_box_covered) / free_den;
        out.p_anchor_success = static_cast<double>(out.seed_probe_anchor_success) / free_den;
        out.p_main_accessible = static_cast<double>(out.seed_probe_main_accessible) / free_den;

        out.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
        out.profile = {};
        out.profile.raw_boxes = static_cast<int>(raw_boxes_.size());
        out.profile.final_boxes = static_cast<int>(boxes_.size());
        out.profile.segment_edges = static_cast<int>(segment_edges_.size());
        out.profile.grow_ms = out.leaf_sweep_ms;
        out.profile.total_ms = out.total_ms;
        if (use_partition_backend) {
            out.profile.grow_adjacency_islands = partition_island_count_for_profile;
            out.profile.adjacency_islands = partition_island_count_for_profile;
            out.profile.grow_largest_island = partition_largest_island_for_profile;
        } else {
            const auto graph_islands = find_islands(adjacency_);
            out.profile.grow_adjacency_islands = static_cast<int>(graph_islands.size());
            out.profile.adjacency_islands = out.profile.grow_adjacency_islands;
            for (const auto& island : graph_islands) {
                out.profile.grow_largest_island =
                    std::max(out.profile.grow_largest_island, static_cast<int>(island.size()));
            }
        }
        out.profile.diagnostics = out.leaf_sweep.diagnostics;
        out.profile.diagnostics["adaptive.fast_leaf_sweep"] = 1.0;
        out.profile.diagnostics["adaptive.offline_query_agnostic_build"] = 1.0;
        out.profile.diagnostics["adaptive.qroot_pairs_total"] = 0.0;
        out.profile.diagnostics["adaptive.qroot_uncovered_endpoints"] = 0.0;
        out.profile.diagnostics["adaptive.leaf_sweep_ms"] = out.leaf_sweep_ms;
        record_depth_semantics_diagnostics(out.profile.diagnostics,
                                           "adaptive.",
                                           adaptive_config.shallow_start_depth,
                                           initial_leaf_depth,
                                           target_leaf_depth,
                                           config_.grower.find_free_box,
                                           target_leaf_depth);
        out.profile.diagnostics["adaptive.overlap_depth_threshold"] = adaptive_config.overlap_depth_threshold;
        out.profile.diagnostics["adaptive.overlap_depth_min_threshold"] = adaptive_config.overlap_depth_min_threshold;
        out.profile.diagnostics["adaptive.overlap_depth_decay_per_depth"] = adaptive_config.overlap_depth_decay_per_depth;
        out.profile.diagnostics["adaptive.merge_ms"] = merge_ms;
        out.profile.diagnostics["adaptive.merge_input_boxes"] = static_cast<double>(merge_stats.input_boxes);
        out.profile.diagnostics["adaptive.merge_output_boxes"] = static_cast<double>(merge_stats.output_boxes);
        out.profile.diagnostics["adaptive.merge_grid_ms"] = merge_stats.grid_ms;
        out.profile.diagnostics["adaptive.merge_grid_merges"] = static_cast<double>(merge_stats.grid_merges);
        out.profile.diagnostics["adaptive.merge_grid_rounds"] = static_cast<double>(merge_stats.grid_rounds);
        out.profile.diagnostics["adaptive.merge_tree_ms"] = merge_stats.tree_ms;
        out.profile.diagnostics["adaptive.merge_tree_merges"] = static_cast<double>(merge_stats.tree_merges);
        out.profile.diagnostics["adaptive.merge_tree_rounds"] = static_cast<double>(merge_stats.tree_rounds);
        out.profile.diagnostics["adaptive.merge_containment_ms"] = merge_stats.containment_ms;
        out.profile.diagnostics["adaptive.merge_exact_ms"] = merge_stats.exact_ms;
        out.profile.diagnostics["adaptive.merge_containment_pruned"] = static_cast<double>(merge_stats.containment_pruned);
        out.profile.diagnostics["adaptive.merge_exact_merges"] = static_cast<double>(merge_stats.exact_merges);
        out.profile.diagnostics["adaptive.merge_rounds"] = static_cast<double>(merge_stats.rounds);
        out.profile.diagnostics["adaptive.merge_stop_reason"] = static_cast<double>(merge_stats.stop_reason);
        out.profile.diagnostics["adaptive.partition_merge_enabled"] =
            out.diagnostics.find("adaptive.partition_merge_enabled") != out.diagnostics.end() ? 1.0 : 0.0;
        out.profile.diagnostics["adaptive.partition_merge_released_boxes"] =
            out.diagnostics.find("adaptive.partition_merge_released_boxes") != out.diagnostics.end()
                ? out.diagnostics["adaptive.partition_merge_released_boxes"]
                : 0.0;
        out.profile.diagnostics["adaptive.partition_merge_containment_skipped"] =
            out.diagnostics.find("adaptive.partition_merge_containment_skipped") != out.diagnostics.end()
                ? out.diagnostics["adaptive.partition_merge_containment_skipped"]
                : 0.0;
        out.profile.diagnostics["adaptive.partition_merge_containment_bucket_entries"] =
            out.diagnostics.find("adaptive.partition_merge_containment_bucket_entries") != out.diagnostics.end()
                ? out.diagnostics["adaptive.partition_merge_containment_bucket_entries"]
                : 0.0;
        out.profile.diagnostics["adaptive.partition_merge_containment_candidates"] =
            out.diagnostics.find("adaptive.partition_merge_containment_candidates") != out.diagnostics.end()
                ? out.diagnostics["adaptive.partition_merge_containment_candidates"]
                : 0.0;
        out.profile.diagnostics["adaptive.partition_merge_containment_tests"] =
            out.diagnostics.find("adaptive.partition_merge_containment_tests") != out.diagnostics.end()
                ? out.diagnostics["adaptive.partition_merge_containment_tests"]
                : 0.0;
        out.profile.diagnostics["adaptive.partition_merge_containment_overflow"] =
            out.diagnostics.find("adaptive.partition_merge_containment_overflow") != out.diagnostics.end()
                ? out.diagnostics["adaptive.partition_merge_containment_overflow"]
                : 0.0;
        out.profile.diagnostics["adaptive.partition_merge_containment_ms"] =
            out.diagnostics.find("adaptive.partition_merge_containment_ms") != out.diagnostics.end()
                ? out.diagnostics["adaptive.partition_merge_containment_ms"]
                : 0.0;
        out.profile.diagnostics["adaptive.partition_merge_line_ms"] =
            out.diagnostics.find("adaptive.partition_merge_line_ms") != out.diagnostics.end()
                ? out.diagnostics["adaptive.partition_merge_line_ms"]
                : 0.0;
        out.profile.diagnostics["adaptive.partition_skipped_graph_adjacency"] =
            out.diagnostics.find("adaptive.partition_skipped_graph_adjacency") != out.diagnostics.end()
                ? out.diagnostics["adaptive.partition_skipped_graph_adjacency"]
                : 0.0;
        out.profile.diagnostics["adaptive.adjacency_ms"] = adjacency_stats.build_ms;
        out.profile.diagnostics["adaptive.adjacency_boxes"] = static_cast<double>(adjacency_stats.boxes);
        out.profile.diagnostics["adaptive.adjacency_selected_dims"] = static_cast<double>(adjacency_stats.selected_dims);
        out.profile.diagnostics["adaptive.adjacency_primary_dim"] = static_cast<double>(adjacency_stats.primary_dim);
        out.profile.diagnostics["adaptive.adjacency_candidates"] = static_cast<double>(adjacency_stats.candidate_pairs);
        out.profile.diagnostics["adaptive.adjacency_exact_tests"] = static_cast<double>(adjacency_stats.exact_tests);
        out.profile.diagnostics["adaptive.adjacency_edges"] = static_cast<double>(adjacency_stats.edges);
        out.profile.diagnostics["adaptive.adaptive_ms"] = 0.0;
	    out.profile.diagnostics["adaptive.coverage_probe_ms"] = out.coverage_probe_ms;
	    out.profile.diagnostics["adaptive.total_ms"] = out.total_ms;
    out.profile.diagnostics["adaptive.depth_enabled"] = adaptive_depth_enabled ? 1.0 : 0.0;
    out.profile.diagnostics["adaptive.depth_min"] = static_cast<double>(adaptive_depth_min);
    out.profile.diagnostics["adaptive.depth_max"] = static_cast<double>(target_leaf_depth);
    out.profile.diagnostics["adaptive.depth_min_covered_probes"] =
        static_cast<double>(adaptive_config.adaptive_depth_min_covered_probes);
    out.profile.diagnostics["adaptive.depth_min_main_probes"] =
        static_cast<double>(adaptive_config.adaptive_depth_min_main_probes);
    out.profile.diagnostics["adaptive.depth_min_main_ratio"] =
        adaptive_config.adaptive_depth_min_main_ratio;
    out.profile.diagnostics["adaptive.depth_min_cells"] =
        static_cast<double>(adaptive_config.adaptive_depth_min_cells);
    out.profile.diagnostics["adaptive.depth_min_main_cells"] =
        static_cast<double>(adaptive_config.adaptive_depth_min_main_cells);
    out.profile.diagnostics["adaptive.depth_max_online_cells"] =
        static_cast<double>(adaptive_config.adaptive_depth_max_online_cells);
	    out.profile.diagnostics["adaptive.shallow_free_count"] = static_cast<double>(out.shallow_free_count);
        out.profile.diagnostics["adaptive.shallow_collision_count"] = static_cast<double>(out.shallow_collision_count);
        out.profile.diagnostics["adaptive.deferred"] = static_cast<double>(out.adaptive_deferred);
        out.profile.diagnostics["adaptive.unresolved_domains"] = static_cast<double>(out.unresolved_domains);
        out.profile.diagnostics["adaptive.seed_probe_count"] = static_cast<double>(out.seed_probe_count);
        out.profile.diagnostics["adaptive.seed_probe_free_count"] = static_cast<double>(out.seed_probe_free_count);
        out.profile.diagnostics["adaptive.seed_probe_box_covered"] = static_cast<double>(out.seed_probe_box_covered);
        out.profile.diagnostics["adaptive.seed_probe_anchor_success"] = static_cast<double>(out.seed_probe_anchor_success);
        out.profile.diagnostics["adaptive.seed_probe_main_accessible"] = static_cast<double>(out.seed_probe_main_accessible);
        out.profile.diagnostics["adaptive.seed_anchor_probe_cap"] = static_cast<double>(anchor_cap);
        out.profile.diagnostics["adaptive.seed_anchor_probe_attempts"] = static_cast<double>(uncovered_anchor_attempts);
	        out.profile.diagnostics["adaptive.p_box_covered"] = out.p_box_covered;
	        out.profile.diagnostics["adaptive.p_anchor_success"] = out.p_anchor_success;
	        out.profile.diagnostics["adaptive.p_main_accessible"] = out.p_main_accessible;
	        out.profile.diagnostics["adaptive.p_anchor_to_main_uncovered"] = out.p_anchor_to_main_uncovered;
	        out.profile.diagnostics["adaptive.selected_leaf_depth"] =
	            static_cast<double>(out.selected_leaf_depth);
	        out.profile.diagnostics["adaptive.depth_readiness_met"] =
	            out.adaptive_depth_readiness_met ? 1.0 : 0.0;
	        rebuild_adaptive_partition(partition_config, &out.profile);
        if (adaptive_partition_ && !adaptive_partition_->empty()) {
            const auto& partition_stats = adaptive_partition_->stats();
            out.partition_cell_count = partition_stats.cells;
            out.partition_grid_cell_count = partition_stats.grid_cells;
            out.partition_non_grid_cell_count = partition_stats.non_grid_cells;
            out.partition_face_index_entries = partition_stats.face_index_entries;
            out.partition_islands = partition_stats.islands;
            out.partition_largest_island = partition_stats.largest_island;
        }
        if (oracle_) {
            const OracleCounters counters = oracle_->counters();
            normalize_external_evidence_diagnostics(out.profile.diagnostics, &counters);
        } else {
            normalize_external_evidence_diagnostics(out.profile.diagnostics);
        }
        record_portal_membership_policy(out.profile.diagnostics, config_.portal_membership_policy);
        out.diagnostics = out.profile.diagnostics;
        last_build_ = out.profile;
        if (config_.database.checkpoint_after_build && database_) {
            database_->checkpoint();
        }
        return out;
    }

    out.leaf_sweep = build_leaf_sweep(obstacles,
                                      adaptive_config.shallow_start_depth,
                                      initial_leaf_depth,
                                      leaf_config);
    out.leaf_sweep_ms = out.leaf_sweep.total_ms;
    out.shallow_free_count = static_cast<int>(out.leaf_sweep.free_boxes.size());
    out.shallow_collision_count = static_cast<int>(out.leaf_sweep.collision_boxes.size());

    const double adjacency_tolerance = config_.query.adjacency_tolerance;
    const auto merge_start = Clock::now();
    BudgetedMergeStats merge_stats;
    if (config_.enable_merger && !boxes_.empty()) {
        bool merged_by_partition = false;
        if (adaptive_config.planning_backend == "partition_native") {
            rebuild_adaptive_partition(partition_config, nullptr);
            if (adaptive_partition_query_enabled_ && adaptive_partition_) {
                AdaptiveGridPartitionMergeOptions options;
                options.max_ms = adaptive_config.max_merge_ms;
                options.max_rounds = adaptive_config.max_merge_rounds;
                options.grid_line_merge = true;
                options.containment_prune = false;
                const auto partition_merge =
                    adaptive_partition_->merge_boxes(boxes_, options, adjacency_tolerance);
                for (int released_id : partition_merge.released_box_ids) {
                    oracle_->release_box(released_id);
                }
                merge_stats.input_boxes = partition_merge.input_boxes;
                merge_stats.output_boxes = partition_merge.output_boxes;
                merge_stats.grid_merges = partition_merge.grid_merges;
                merge_stats.grid_rounds = partition_merge.rounds;
                merge_stats.containment_pruned = partition_merge.containment_pruned;
                merge_stats.stop_reason = partition_merge.stop_reason == 0 ? 1 : partition_merge.stop_reason;
                merge_stats.total_ms = partition_merge.total_ms;
                merge_stats.grid_ms = partition_merge.total_ms;
                out.diagnostics["adaptive.partition_merge_enabled"] = 1.0;
                out.diagnostics["adaptive.partition_merge_released_boxes"] =
                    static_cast<double>(partition_merge.released_box_ids.size());
                out.diagnostics["adaptive.partition_merge_containment_skipped"] =
                    static_cast<double>(partition_merge.containment_skipped);
                out.diagnostics["adaptive.partition_merge_containment_bucket_entries"] =
                    static_cast<double>(partition_merge.containment_bucket_entries);
                out.diagnostics["adaptive.partition_merge_containment_candidates"] =
                    static_cast<double>(partition_merge.containment_candidates);
                out.diagnostics["adaptive.partition_merge_containment_tests"] =
                    static_cast<double>(partition_merge.containment_tests);
                out.diagnostics["adaptive.partition_merge_containment_overflow"] =
                    static_cast<double>(partition_merge.containment_overflow);
                out.diagnostics["adaptive.partition_merge_containment_ms"] =
                    partition_merge.containment_ms;
                out.diagnostics["adaptive.partition_merge_line_ms"] =
                    partition_merge.line_merge_ms;
                merged_by_partition = true;
            }
        }
        if (!merged_by_partition) {
            MergerConfig leaf_merge_config = config_.merger;
            leaf_merge_config.containment_prune = true;
            merge_stats = budgeted_leaf_merge(*oracle_,
                                              boxes_,
                                              leaf_merge_config,
                                              adaptive_config.max_merge_ms,
                                              adaptive_config.max_merge_rounds,
                                              adaptive_config.max_merge_input_boxes,
                                              adjacency_tolerance);
        }
        raw_boxes_ = boxes_;
    } else {
        merge_stats.input_boxes = static_cast<int>(boxes_.size());
        merge_stats.output_boxes = static_cast<int>(boxes_.size());
        merge_stats.stop_reason = 0;
    }
	    const double merge_ms = std::chrono::duration<double, std::milli>(Clock::now() - merge_start).count();
	    const bool use_partition_backend = adaptive_config.planning_backend == "partition_native";
	    AdjacencyBuildStats initial_adjacency_stats;
	    std::unordered_set<int> main_ids;
	    auto refresh_main_from_partition = [&]() -> bool {
	        if (!adaptive_partition_query_enabled_ || !adaptive_partition_) {
	            return false;
	        }
	        const auto largest = adaptive_partition_->largest_component_box_ids_with_overlay();
	        main_ids.clear();
	        main_ids.insert(largest.begin(), largest.end());
	        return !main_ids.empty();
	    };
	    if (use_partition_backend) {
	        if (!adaptive_partition_query_enabled_ || !adaptive_partition_) {
	            rebuild_adaptive_partition(partition_config, nullptr);
	        }
	        if (refresh_main_from_partition()) {
	            out.diagnostics["adaptive.partition_skipped_graph_adjacency"] = 1.0;
	        }
	    }
	    if (!use_partition_backend) {
	        rebuild_adjacency();
	        initial_adjacency_stats = last_adjacency_build_stats();
	        main_ids = adaptive_largest_island_ids(adjacency_);
	    } else if (main_ids.empty()) {
	        out.diagnostics["adaptive.partition_missing_no_graph_fallback"] = 1.0;
	    }
    std::vector<BoxNode> scoring_boxes = boxes_;
    std::vector<AdaptiveFrontierItem> deferred;
    deferred.reserve(out.leaf_sweep.collision_boxes.size());
    const auto planning_domain = oracle_ ? oracle_->planning_intervals() : std::vector<Interval>{};
    int probe_attempted = 0;
    const int requested_probe_count = adaptive_depth_enabled
        ? std::max(0, adaptive_config.adaptive_depth_probe_count)
        : std::max(0, adaptive_config.seed_probe_count);
    const int requested_probe_seed = adaptive_depth_enabled
        ? adaptive_config.adaptive_depth_probe_seed
        : adaptive_config.seed_probe_rng_seed;
    const auto probe_seed_start = Clock::now();
    std::vector<Eigen::VectorXd> free_probes =
        oracle_ ? adaptive_generate_free_probes(*oracle_,
                                                planning_domain,
                                                requested_probe_count,
                                                requested_probe_seed,
                                                probe_attempted)
                : std::vector<Eigen::VectorXd>{};
    if (adaptive_depth_enabled && oracle_ &&
        adaptive_config.adaptive_depth_min_free_probes > 0 &&
        static_cast<int>(free_probes.size()) < adaptive_config.adaptive_depth_min_free_probes) {
        const int supplement_limit = std::max(
            requested_probe_count,
            std::min(8192, std::max(requested_probe_count * 4,
                                    adaptive_config.adaptive_depth_min_free_probes * 64)));
        int supplement_seed_offset = 1;
        while (probe_attempted < supplement_limit &&
               static_cast<int>(free_probes.size()) < adaptive_config.adaptive_depth_min_free_probes) {
            const int batch = std::min(std::max(128, requested_probe_count), supplement_limit - probe_attempted);
            int extra_attempted = 0;
            auto extra = adaptive_generate_free_probes(*oracle_,
                                                       planning_domain,
                                                       batch,
                                                       requested_probe_seed + supplement_seed_offset,
                                                       extra_attempted);
            probe_attempted += extra_attempted;
            free_probes.insert(free_probes.end(),
                               std::make_move_iterator(extra.begin()),
                               std::make_move_iterator(extra.end()));
            ++supplement_seed_offset;
            if (batch <= 0 || extra_attempted <= 0) {
                break;
            }
        }
    }
    const double initial_probe_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - probe_seed_start).count();
    out.seed_probe_count = probe_attempted;
    out.seed_probe_free_count = static_cast<int>(free_probes.size());

    auto item_less = [](const AdaptiveFrontierItem& lhs, const AdaptiveFrontierItem& rhs) {
        return lhs.score < rhs.score;
    };
    std::priority_queue<AdaptiveFrontierItem,
                        std::vector<AdaptiveFrontierItem>,
                        decltype(item_less)>
        frontier(item_less);

    auto refresh_score = [&](AdaptiveFrontierItem& item) {
        item.score = adaptive_frontier_score(scoring_boxes,
                                             item,
                                             main_ids,
                                             adaptive_config.overlap_depth_threshold,
                                             adjacency_tolerance);
    };
    auto push_frontier = [&](AdaptiveFrontierItem item) {
        item.free_seed_hits = adaptive_count_seed_hits(item, free_probes);
        if (item.free_seed_hits > 0) {
            out.diagnostics["adaptive.frontier_seed_hit_pushes"] += 1.0;
            out.diagnostics["adaptive.frontier_seed_hits_total"] += static_cast<double>(item.free_seed_hits);
        }
        refresh_score(item);
        frontier.push(std::move(item));
    };

    for (const auto& collision_box : out.leaf_sweep.collision_boxes) {
        AdaptiveFrontierItem item;
        item.node = collision_box.tree_id >= 0 ? collision_box.tree_id : collision_box.id;
        item.intervals = collision_box.joint_intervals;
        item.changed_dim = -1;
        if (!planning_domain.empty() && !intervals_overlap_local(item.intervals, planning_domain, 0.0)) {
            out.diagnostics["adaptive.initial_frontier_outside_domain"] += 1.0;
            continue;
        }
        push_frontier(std::move(item));
    }

    const auto adaptive_start = Clock::now();
    auto elapsed_total_ms = [&]() {
        return std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
    };
    auto budget_exhausted = [&]() {
        if (adaptive_config.time_budget_ms > 0.0 &&
            elapsed_total_ms() >= adaptive_config.time_budget_ms) {
            return true;
        }
        return adaptive_config.node_budget > 0 &&
               out.adaptive_validated >= adaptive_config.node_budget;
    };
    auto promote_deferred = [&]() {
        if (!adaptive_config.seed_promote_uncovered || deferred.empty() || free_probes.empty()) {
            return;
        }
        std::vector<AdaptiveFrontierItem> keep;
        keep.reserve(deferred.size());
        for (auto& item : deferred) {
            const int hits = adaptive_count_seed_hits(item, free_probes);
            if (hits > 0) {
                item.free_seed_hits = hits;
                refresh_score(item);
                frontier.push(std::move(item));
                out.adaptive_promoted += 1;
                out.diagnostics["adaptive.promoted_by_seed_probe"] += 1.0;
            } else {
                keep.push_back(std::move(item));
            }
        }
        deferred = std::move(keep);
    };

    const auto& split_descriptor = oracle_->database().split_policy_descriptor();
    std::size_t first_unconnected_new_index = boxes_.size();
    int pending_adjacency_boxes = 0;
    constexpr int kAdaptiveAdjacencyBatchSize = 512;
    std::vector<AdaptiveDepthSnapshot> depth_snapshots;
    double checkpoint_probe_ms_total = 0.0;
    auto evaluate_depth_snapshot = [&](int depth, bool allow_anchor_probe) {
        const auto snapshot_start = Clock::now();
        if (use_partition_backend && adaptive_partition_query_enabled_ && adaptive_partition_) {
            refresh_main_from_partition();
        } else if (!use_partition_backend && pending_adjacency_boxes > 0) {
            connect_incremental_boxes(adjacency_,
                                      boxes_,
                                      first_unconnected_new_index,
                                      adjacency_tolerance);
            first_unconnected_new_index = boxes_.size();
            pending_adjacency_boxes = 0;
            main_ids = adaptive_largest_island_ids(adjacency_);
        }
        AdaptiveDepthSnapshot snapshot;
        snapshot.depth = depth;
        snapshot.free_probe_count = static_cast<int>(free_probes.size());
        snapshot.collision_count = static_cast<int>(out.leaf_sweep.collision_boxes.size());
        if (use_partition_backend && adaptive_partition_query_enabled_ && adaptive_partition_) {
            const auto& stats = adaptive_partition_->stats();
            snapshot.cell_count = stats.cells;
            snapshot.island_count = stats.islands;
            snapshot.main_island_cell_count = static_cast<int>(main_ids.size());
        } else {
            snapshot.cell_count = static_cast<int>(boxes_.size());
            snapshot.island_count = static_cast<int>(find_islands(adjacency_).size());
            snapshot.main_island_cell_count = static_cast<int>(main_ids.size());
        }
        BoxSpatialIndex coverage_index;
        const bool use_partition_coverage =
            use_partition_backend && adaptive_partition_query_enabled_ && adaptive_partition_;
        if (!use_partition_coverage) {
            coverage_index.rebuild(boxes_, adjacency_tolerance);
        }
        std::vector<const Eigen::VectorXd*> uncovered;
        uncovered.reserve(free_probes.size());
        for (const auto& point : free_probes) {
            const int owner = use_partition_coverage
                ? adaptive_partition_->locate_containing_box(point, false, adjacency_tolerance)
                : [&]() {
                      const int owner_index = coverage_index.covering_box(boxes_, point, adjacency_tolerance);
                      return owner_index >= 0 ? boxes_[static_cast<std::size_t>(owner_index)].id : -1;
                  }();
            if (owner >= 0) {
                snapshot.covered_count += 1;
                if (main_ids.find(owner) != main_ids.end()) {
                    snapshot.main_accessible_count += 1;
                }
            } else {
                uncovered.push_back(&point);
            }
        }
        const int anchor_cap = allow_anchor_probe
            ? std::max(0, adaptive_depth_enabled
                           ? adaptive_config.adaptive_depth_anchor_probe_cap
                           : adaptive_config.seed_anchor_probe_cap)
            : 0;
        if (anchor_cap > 0 && !planning_domain.empty() && !uncovered.empty()) {
            StageContext probe_context = StageContext::from_runtime(config_.runtime);
            FindFreeBoxOptions probe_options = config_.grower.find_free_box;
            probe_options.max_depth = target_leaf_depth;
            probe_options.reject_seed_collision = false;
            probe_options.deadline_ms = adaptive_depth_enabled ? 3.0 : 5.0;
            const double max_probe_ms = adaptive_depth_enabled
                ? std::max(0.0, adaptive_config.adaptive_depth_max_probe_ms)
                : 0.0;
            for (const Eigen::VectorXd* point : uncovered) {
                if (snapshot.anchor_probe_attempts >= anchor_cap) {
                    break;
                }
                const double elapsed_ms =
                    std::chrono::duration<double, std::milli>(Clock::now() - snapshot_start).count();
                if (max_probe_ms > 0.0 && elapsed_ms >= max_probe_ms) {
                    break;
                }
                ++snapshot.anchor_probe_attempts;
                const auto ffb = find_free_box_in_domain(*point, planning_domain, probe_context, probe_options);
                if (!ffb.found) {
                    continue;
                }
                snapshot.anchor_success_count += 1;
                const BoxNode anchor = adaptive_make_box_from_intervals(
                    ffb.intervals,
                    ffb.node,
                    -1,
                    ffb.validation_detail.safety_status,
                    ffb.validation_detail.strict_audit_required);
                const bool anchor_main_accessible = use_partition_coverage
                    ? adaptive_partition_->box_adjacent_to_any(anchor, main_ids, adjacency_tolerance)
                    : (!use_partition_backend &&
                       adaptive_has_adjacency_to_any(boxes_, anchor, &main_ids, adjacency_tolerance));
                if (anchor_main_accessible) {
                    snapshot.anchor_to_main_count += 1;
                }
            }
        }
        const double free_den = static_cast<double>(std::max(1, snapshot.free_probe_count));
        snapshot.p_box_covered = static_cast<double>(snapshot.covered_count) / free_den;
        snapshot.p_main_accessible = static_cast<double>(snapshot.main_accessible_count) / free_den;
        snapshot.main_connected_ratio =
            static_cast<double>(snapshot.main_accessible_count) /
            static_cast<double>(std::max(1, snapshot.covered_count));
        snapshot.p_anchor_to_main_uncovered =
            static_cast<double>(snapshot.anchor_to_main_count) /
            static_cast<double>(std::max(1, snapshot.anchor_probe_attempts));
        snapshot.probe_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - snapshot_start).count();
        const int min_covered_probes = std::max(0, adaptive_config.adaptive_depth_min_covered_probes);
        const int min_main_probes = std::max(0, adaptive_config.adaptive_depth_min_main_probes);
        const int min_cells = std::max(0, adaptive_config.adaptive_depth_min_cells);
        const int min_main_cells = std::max(0, adaptive_config.adaptive_depth_min_main_cells);
        const bool probe_gate =
            snapshot.covered_count >= min_covered_probes &&
            snapshot.main_accessible_count >= min_main_probes &&
            (min_covered_probes <= 0 ||
             snapshot.main_connected_ratio >= adaptive_config.adaptive_depth_min_main_ratio);
        const bool cell_gate =
            snapshot.cell_count >= min_cells &&
            snapshot.main_island_cell_count >= min_main_cells;
        snapshot.readiness_met =
            adaptive_depth_enabled &&
            snapshot.cell_count > 0 &&
            snapshot.main_island_cell_count > 0 &&
            probe_gate &&
            cell_gate &&
            (adaptive_config.adaptive_depth_max_online_cells <= 0 ||
             snapshot.cell_count <= adaptive_config.adaptive_depth_max_online_cells);
        return snapshot;
    };
    auto apply_final_depth_snapshot = [&](const AdaptiveDepthSnapshot& snapshot) {
        out.selected_leaf_depth = snapshot.depth;
        out.adaptive_depth_readiness_met = snapshot.readiness_met;
        out.adaptive_depth_stop_reason = snapshot.stop_reason;
        out.seed_probe_box_covered = snapshot.covered_count;
        out.seed_probe_anchor_success = snapshot.anchor_success_count;
        out.seed_probe_main_accessible = snapshot.main_accessible_count + snapshot.anchor_to_main_count;
        out.p_box_covered = snapshot.p_box_covered;
        const double free_den = static_cast<double>(std::max(1, snapshot.free_probe_count));
        out.p_anchor_success = static_cast<double>(snapshot.anchor_success_count) / free_den;
        out.p_main_accessible = static_cast<double>(out.seed_probe_main_accessible) / free_den;
        out.p_anchor_to_main_uncovered = snapshot.p_anchor_to_main_uncovered;
    };
    auto record_depth_snapshot = [&](AdaptiveDepthSnapshot snapshot) {
        checkpoint_probe_ms_total += snapshot.probe_ms;
        depth_snapshots.push_back(std::move(snapshot));
    };
    auto next_depth_checkpoint = [&](int depth) {
        const int step = depth < 16 ? 1 : 2;
        return std::min(target_leaf_depth, depth + step);
    };
    bool adaptive_depth_stop = false;
    int next_checkpoint_depth = initial_leaf_depth;
    if (adaptive_depth_enabled) {
        auto initial_snapshot = evaluate_depth_snapshot(initial_leaf_depth, true);
        if (initial_snapshot.readiness_met) {
            initial_snapshot.stop_reason = "coverage_ready";
            adaptive_depth_stop = true;
        } else if (initial_leaf_depth >= target_leaf_depth) {
            initial_snapshot.stop_reason = "max_depth";
            adaptive_depth_stop = true;
        } else {
            initial_snapshot.stop_reason = "checkpoint";
            next_checkpoint_depth = next_depth_checkpoint(initial_leaf_depth);
        }
        record_depth_snapshot(std::move(initial_snapshot));
    }
    std::vector<AdaptiveFrontierItem> checkpoint_hold;
    auto restore_checkpoint_hold = [&]() {
        for (auto& held : checkpoint_hold) {
            frontier.push(std::move(held));
        }
        checkpoint_hold.clear();
    };
    {
        const bool collect_overlap_ratio =
            adaptive_config.overlap_ratio_threshold > 0.0 &&
            adaptive_config.defer_min_depth >= 0;
        ScopedAdaptiveFullOverlapStats overlap_stats(*oracle_, collect_overlap_ratio);
        while (!frontier.empty() && !budget_exhausted() && !adaptive_depth_stop) {
            AdaptiveFrontierItem item = frontier.top();
            frontier.pop();
            if (item.intervals.empty()) {
                out.diagnostics["adaptive.empty_frontier_items"] += 1.0;
                continue;
            }
            const int depth = adaptive_virtual_depth(item.node);
            if (adaptive_depth_enabled && depth > next_checkpoint_depth) {
                checkpoint_hold.push_back(std::move(item));
                if (frontier.empty()) {
                    restore_checkpoint_hold();
                    auto snapshot = evaluate_depth_snapshot(next_checkpoint_depth, true);
                    if (snapshot.readiness_met) {
                        snapshot.stop_reason = "coverage_ready";
                        adaptive_depth_stop = true;
                    } else if (next_checkpoint_depth >= target_leaf_depth) {
                        snapshot.stop_reason = "max_depth";
                        adaptive_depth_stop = true;
                    } else {
                        snapshot.stop_reason = "checkpoint";
                        next_checkpoint_depth = next_depth_checkpoint(next_checkpoint_depth);
                    }
                    record_depth_snapshot(std::move(snapshot));
                }
                continue;
            }
            adaptive_add_depth_counter(out.diagnostics, "adaptive.depth.validated.", depth);
            BoxValidation validation = BoxValidation::Unknown;
            OracleValidationDetail detail;
            try {
                validation = oracle_->validate_node(item.node, item.intervals, item.changed_dim);
                detail = oracle_->last_validation_detail();
            } catch (const std::exception&) {
                out.diagnostics["adaptive.validation_exceptions"] += 1.0;
                validation = BoxValidation::Unknown;
            }
            out.adaptive_validated += 1;
            item.overlap_depth = detail.aabb_overlap_depth;
            item.overlap_ratio = detail.aabb_overlap_volume_ratio;
            const bool item_has_seed_hit = item.free_seed_hits > 0;
            if (item_has_seed_hit) {
                out.diagnostics["adaptive.seed_hit_validated"] += 1.0;
            }

            if (validation == BoxValidation::Free) {
                BoxNode candidate = adaptive_make_box_from_intervals(item.intervals,
                                                                     item.node,
                                                                     next_box_id(),
                                                                     detail.safety_status,
                                                                     detail.strict_audit_required);
                bool contained = false;
                for (const auto& existing : boxes_) {
                    if (intervals_subset_local(candidate.joint_intervals,
                                               existing.joint_intervals,
                                               1e-12)) {
                        contained = true;
                        break;
                    }
                }
                if (contained) {
                    out.diagnostics["adaptive.free_contained_rejects"] += 1.0;
                    continue;
                }
                const std::size_t new_index = boxes_.size();
                (void)new_index;
                boxes_.push_back(candidate);
                raw_boxes_.push_back(candidate);
                scoring_boxes.push_back(candidate);
	                oracle_->reserve_node(candidate.tree_id, candidate.id);
	                out.adaptive_free_added += 1;
	                pending_adjacency_boxes += 1;
	                adaptive_add_depth_counter(out.diagnostics, "adaptive.depth.free.", depth);
	                if (item_has_seed_hit) {
	                    out.diagnostics["adaptive.seed_hit_free"] += 1.0;
	                }
	                if (use_partition_backend && adaptive_partition_query_enabled_ && adaptive_partition_) {
	                    const int appended =
	                        adaptive_partition_->append_boxes(boxes_, new_index, adjacency_tolerance);
	                    out.diagnostics["adaptive.partition_incremental_boxes_appended"] +=
	                        static_cast<double>(std::max(0, appended));
	                    if (pending_adjacency_boxes >= kAdaptiveAdjacencyBatchSize) {
	                        refresh_main_from_partition();
	                        pending_adjacency_boxes = 0;
	                        out.diagnostics["adaptive.partition_main_refreshes"] += 1.0;
	                    }
	                } else {
	                    adjacency_[candidate.id];
	                    if (pending_adjacency_boxes >= kAdaptiveAdjacencyBatchSize) {
	                    connect_incremental_boxes(adjacency_,
	                                              boxes_,
	                                              first_unconnected_new_index,
	                                              adjacency_tolerance);
	                    first_unconnected_new_index = boxes_.size();
	                    pending_adjacency_boxes = 0;
	                    main_ids = adaptive_largest_island_ids(adjacency_);
	                    out.diagnostics["adaptive.adjacency_batch_updates"] += 1.0;
	                    }
	                }
	                continue;
	            }

            double active_overlap_depth_threshold = adaptive_config.overlap_depth_threshold;
            if (adaptive_config.overlap_depth_decay_per_depth > 0.0 &&
                depth > adaptive_config.defer_min_depth) {
                active_overlap_depth_threshold =
                    active_overlap_depth_threshold /
                    (1.0 + adaptive_config.overlap_depth_decay_per_depth *
                               static_cast<double>(depth - adaptive_config.defer_min_depth));
            }
            if (adaptive_config.overlap_depth_min_threshold > 0.0) {
                active_overlap_depth_threshold =
                    std::max(adaptive_config.overlap_depth_min_threshold,
                             active_overlap_depth_threshold);
            }
            const bool high_overlap =
                depth >= adaptive_config.defer_min_depth &&
                ((adaptive_config.overlap_depth_threshold > 0.0 &&
                  item.overlap_depth >= active_overlap_depth_threshold) ||
                 (adaptive_config.overlap_ratio_threshold > 0.0 &&
                  item.overlap_ratio >= adaptive_config.overlap_ratio_threshold));
            const bool protected_by_seed = item_has_seed_hit;
            const AdaptiveConnectivityDominance connectivity =
                adaptive_connectivity_dominance(scoring_boxes, item, main_ids, adjacency_tolerance);
            const bool protected_by_adjacency =
                high_overlap && !protected_by_seed &&
                (connectivity.connector_candidate || connectivity.adjacent_main > 0);
            if (depth >= target_leaf_depth) {
                deferred.push_back(std::move(item));
                out.adaptive_deferred += 1;
                out.diagnostics["adaptive.deferred_depth_cap"] += 1.0;
                adaptive_add_depth_counter(out.diagnostics, "adaptive.depth.deferred.", depth);
                if (item_has_seed_hit) {
                    out.diagnostics["adaptive.seed_hit_deferred"] += 1.0;
                }
                continue;
            }
            if (high_overlap && !protected_by_seed && !protected_by_adjacency) {
                deferred.push_back(std::move(item));
                out.adaptive_deferred += 1;
                out.diagnostics["adaptive.deferred_high_overlap"] += 1.0;
                adaptive_add_depth_counter(out.diagnostics, "adaptive.depth.deferred.", depth);
                continue;
            }
            if (depth >= adaptive_config.defer_min_depth &&
                !protected_by_seed &&
                connectivity.has_free_context &&
                connectivity.isolated) {
                deferred.push_back(std::move(item));
                out.adaptive_deferred += 1;
                out.diagnostics["adaptive.deferred_connectivity_isolated"] += 1.0;
                adaptive_add_depth_counter(out.diagnostics, "adaptive.depth.deferred.", depth);
                continue;
            }
            if (depth >= adaptive_config.defer_min_depth &&
                !protected_by_seed &&
                connectivity.has_free_context &&
                connectivity.single_component &&
                connectivity.adjacent_main == 0) {
                deferred.push_back(std::move(item));
                out.adaptive_deferred += 1;
                out.diagnostics["adaptive.deferred_connectivity_single_component"] += 1.0;
                adaptive_add_depth_counter(out.diagnostics, "adaptive.depth.deferred.", depth);
                continue;
            }

            AdaptiveFrontierItem left;
            AdaptiveFrontierItem right;
            if (!adaptive_virtual_split_node(split_descriptor, item, left, right)) {
                deferred.push_back(std::move(item));
                out.adaptive_deferred += 1;
                out.diagnostics["adaptive.deferred_split_failure"] += 1.0;
                continue;
            }
            out.adaptive_splits += 1;
            if (item_has_seed_hit) {
                out.diagnostics["adaptive.seed_hit_splits"] += 1.0;
            }
            adaptive_add_depth_counter(out.diagnostics, "adaptive.depth.split.", depth);
            if (planning_domain.empty() || intervals_overlap_local(left.intervals, planning_domain, 0.0)) {
                push_frontier(std::move(left));
            } else {
                out.diagnostics["adaptive.split_child_outside_domain"] += 1.0;
            }
            if (planning_domain.empty() || intervals_overlap_local(right.intervals, planning_domain, 0.0)) {
                push_frontier(std::move(right));
            } else {
                out.diagnostics["adaptive.split_child_outside_domain"] += 1.0;
            }
            if (adaptive_config.promotion_interval > 0 &&
                out.adaptive_validated % adaptive_config.promotion_interval == 0) {
                promote_deferred();
            }
        }
    }
    restore_checkpoint_hold();
    if (adaptive_depth_enabled && !adaptive_depth_stop && budget_exhausted()) {
        auto snapshot = evaluate_depth_snapshot(depth_snapshots.empty()
                                                    ? initial_leaf_depth
                                                    : depth_snapshots.back().depth,
                                                true);
        snapshot.stop_reason = "budget";
        adaptive_depth_stop = true;
        record_depth_snapshot(std::move(snapshot));
    } else if (adaptive_depth_enabled && !adaptive_depth_stop && frontier.empty()) {
        auto snapshot = evaluate_depth_snapshot(depth_snapshots.empty()
                                                    ? initial_leaf_depth
                                                    : depth_snapshots.back().depth,
                                                true);
        snapshot.stop_reason = "frontier_empty";
        adaptive_depth_stop = true;
        record_depth_snapshot(std::move(snapshot));
    }
    promote_deferred();
	    while (!frontier.empty()) {
	        deferred.push_back(frontier.top());
	        frontier.pop();
	    }
	    out.unresolved_domains = static_cast<int>(deferred.size());
	    out.adaptive_ms = std::chrono::duration<double, std::milli>(Clock::now() - adaptive_start).count();

		    AdjacencyBuildStats final_adjacency_stats;
		    if (use_partition_backend && adaptive_partition_query_enabled_ && adaptive_partition_) {
		        refresh_main_from_partition();
		    } else if (!use_partition_backend) {
		        rebuild_adjacency();
	        final_adjacency_stats = last_adjacency_build_stats();
	        main_ids = adaptive_largest_island_ids(adjacency_);
		    } else {
		        out.diagnostics["adaptive.partition_missing_no_graph_fallback"] = 1.0;
		    }
    if (depth_snapshots.empty()) {
        auto snapshot = evaluate_depth_snapshot(adaptive_depth_enabled ? initial_leaf_depth : target_leaf_depth,
                                                true);
        snapshot.stop_reason = adaptive_depth_enabled ? "max_depth" : "fixed_depth";
        record_depth_snapshot(std::move(snapshot));
    } else if (!adaptive_depth_enabled || depth_snapshots.back().stop_reason == "checkpoint") {
        auto snapshot = evaluate_depth_snapshot(adaptive_depth_enabled
                                                    ? depth_snapshots.back().depth
                                                    : target_leaf_depth,
                                                true);
        snapshot.stop_reason = adaptive_depth_enabled ? "max_depth" : "fixed_depth";
        record_depth_snapshot(std::move(snapshot));
    }
    apply_final_depth_snapshot(depth_snapshots.back());
    out.adaptive_depth_snapshots_json = adaptive_depth_snapshots_to_json(depth_snapshots);
    out.coverage_probe_ms = initial_probe_ms + checkpoint_probe_ms_total;

    out.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
    out.profile = {};
    out.profile.raw_boxes = static_cast<int>(raw_boxes_.size());
    out.profile.final_boxes = static_cast<int>(boxes_.size());
	    out.profile.segment_edges = static_cast<int>(segment_edges_.size());
	    out.profile.grow_ms = out.leaf_sweep_ms + out.adaptive_ms;
	    out.profile.total_ms = out.total_ms;
	    out.profile.grow_largest_island = 0;
	    if (use_partition_backend) {
	        if (adaptive_partition_query_enabled_ && adaptive_partition_) {
	            const auto& partition_stats = adaptive_partition_->stats();
	            out.profile.grow_adjacency_islands = partition_stats.islands;
	            out.profile.adjacency_islands = partition_stats.islands;
	            out.profile.grow_largest_island = partition_stats.largest_island;
	        }
	    } else {
	        const auto graph_islands = find_islands(adjacency_);
	        out.profile.grow_adjacency_islands = static_cast<int>(graph_islands.size());
	        out.profile.adjacency_islands = out.profile.grow_adjacency_islands;
	        for (const auto& island : graph_islands) {
	            out.profile.grow_largest_island =
	                std::max(out.profile.grow_largest_island, static_cast<int>(island.size()));
	        }
	    }
    out.profile.connector_ms = 0.0;
    out.profile.diagnostics = out.leaf_sweep.diagnostics;
    merge_diagnostic_snapshot(out.profile.diagnostics, out.diagnostics);
    out.profile.diagnostics["adaptive.leaf_sweep_ms"] = out.leaf_sweep_ms;
    record_depth_semantics_diagnostics(out.profile.diagnostics,
                                       "adaptive.",
                                       adaptive_config.shallow_start_depth,
                                       initial_leaf_depth,
                                       target_leaf_depth,
                                       config_.grower.find_free_box,
                                       target_leaf_depth);
    out.profile.diagnostics["adaptive.overlap_depth_threshold"] = adaptive_config.overlap_depth_threshold;
    out.profile.diagnostics["adaptive.overlap_depth_min_threshold"] = adaptive_config.overlap_depth_min_threshold;
    out.profile.diagnostics["adaptive.overlap_depth_decay_per_depth"] = adaptive_config.overlap_depth_decay_per_depth;
    out.profile.diagnostics["adaptive.merge_ms"] = merge_ms;
    out.profile.diagnostics["adaptive.merge_input_boxes"] = static_cast<double>(merge_stats.input_boxes);
    out.profile.diagnostics["adaptive.merge_output_boxes"] = static_cast<double>(merge_stats.output_boxes);
    out.profile.diagnostics["adaptive.merge_grid_ms"] = merge_stats.grid_ms;
    out.profile.diagnostics["adaptive.merge_grid_merges"] = static_cast<double>(merge_stats.grid_merges);
    out.profile.diagnostics["adaptive.merge_grid_rounds"] = static_cast<double>(merge_stats.grid_rounds);
    out.profile.diagnostics["adaptive.merge_tree_ms"] = merge_stats.tree_ms;
    out.profile.diagnostics["adaptive.merge_tree_merges"] = static_cast<double>(merge_stats.tree_merges);
    out.profile.diagnostics["adaptive.merge_tree_rounds"] = static_cast<double>(merge_stats.tree_rounds);
    out.profile.diagnostics["adaptive.merge_containment_ms"] = merge_stats.containment_ms;
    out.profile.diagnostics["adaptive.merge_exact_ms"] = merge_stats.exact_ms;
    out.profile.diagnostics["adaptive.merge_containment_pruned"] = static_cast<double>(merge_stats.containment_pruned);
    out.profile.diagnostics["adaptive.merge_exact_merges"] = static_cast<double>(merge_stats.exact_merges);
    out.profile.diagnostics["adaptive.merge_rounds"] = static_cast<double>(merge_stats.rounds);
    out.profile.diagnostics["adaptive.merge_stop_reason"] = static_cast<double>(merge_stats.stop_reason);
    out.profile.diagnostics["adaptive.partition_merge_enabled"] =
        out.diagnostics.find("adaptive.partition_merge_enabled") != out.diagnostics.end() ? 1.0 : 0.0;
    out.profile.diagnostics["adaptive.partition_merge_released_boxes"] =
        out.diagnostics.find("adaptive.partition_merge_released_boxes") != out.diagnostics.end()
            ? out.diagnostics["adaptive.partition_merge_released_boxes"]
            : 0.0;
    out.profile.diagnostics["adaptive.partition_merge_containment_skipped"] =
        out.diagnostics.find("adaptive.partition_merge_containment_skipped") != out.diagnostics.end()
            ? out.diagnostics["adaptive.partition_merge_containment_skipped"]
            : 0.0;
    out.profile.diagnostics["adaptive.partition_merge_containment_bucket_entries"] =
        out.diagnostics.find("adaptive.partition_merge_containment_bucket_entries") != out.diagnostics.end()
            ? out.diagnostics["adaptive.partition_merge_containment_bucket_entries"]
            : 0.0;
    out.profile.diagnostics["adaptive.partition_merge_containment_candidates"] =
        out.diagnostics.find("adaptive.partition_merge_containment_candidates") != out.diagnostics.end()
            ? out.diagnostics["adaptive.partition_merge_containment_candidates"]
            : 0.0;
    out.profile.diagnostics["adaptive.partition_merge_containment_tests"] =
        out.diagnostics.find("adaptive.partition_merge_containment_tests") != out.diagnostics.end()
            ? out.diagnostics["adaptive.partition_merge_containment_tests"]
            : 0.0;
    out.profile.diagnostics["adaptive.partition_merge_containment_overflow"] =
        out.diagnostics.find("adaptive.partition_merge_containment_overflow") != out.diagnostics.end()
            ? out.diagnostics["adaptive.partition_merge_containment_overflow"]
            : 0.0;
    out.profile.diagnostics["adaptive.partition_merge_containment_ms"] =
        out.diagnostics.find("adaptive.partition_merge_containment_ms") != out.diagnostics.end()
            ? out.diagnostics["adaptive.partition_merge_containment_ms"]
            : 0.0;
    out.profile.diagnostics["adaptive.partition_merge_line_ms"] =
        out.diagnostics.find("adaptive.partition_merge_line_ms") != out.diagnostics.end()
            ? out.diagnostics["adaptive.partition_merge_line_ms"]
            : 0.0;
    out.profile.diagnostics["adaptive.initial_adjacency_ms"] = initial_adjacency_stats.build_ms;
    out.profile.diagnostics["adaptive.initial_adjacency_candidates"] = static_cast<double>(initial_adjacency_stats.candidate_pairs);
    out.profile.diagnostics["adaptive.initial_adjacency_exact_tests"] = static_cast<double>(initial_adjacency_stats.exact_tests);
    out.profile.diagnostics["adaptive.adjacency_ms"] = final_adjacency_stats.build_ms;
    out.profile.diagnostics["adaptive.adjacency_boxes"] = static_cast<double>(final_adjacency_stats.boxes);
    out.profile.diagnostics["adaptive.adjacency_selected_dims"] = static_cast<double>(final_adjacency_stats.selected_dims);
    out.profile.diagnostics["adaptive.adjacency_primary_dim"] = static_cast<double>(final_adjacency_stats.primary_dim);
    out.profile.diagnostics["adaptive.adjacency_candidates"] = static_cast<double>(final_adjacency_stats.candidate_pairs);
    out.profile.diagnostics["adaptive.adjacency_exact_tests"] = static_cast<double>(final_adjacency_stats.exact_tests);
    out.profile.diagnostics["adaptive.adjacency_edges"] = static_cast<double>(final_adjacency_stats.edges);
    out.profile.diagnostics["adaptive.adaptive_ms"] = out.adaptive_ms;
    out.profile.diagnostics["adaptive.coverage_probe_ms"] = out.coverage_probe_ms;
    out.profile.diagnostics["adaptive.total_ms"] = out.total_ms;
    out.profile.diagnostics["adaptive.depth_enabled"] = adaptive_depth_enabled ? 1.0 : 0.0;
    out.profile.diagnostics["adaptive.depth_min"] = static_cast<double>(adaptive_depth_min);
    out.profile.diagnostics["adaptive.depth_max"] = static_cast<double>(target_leaf_depth);
    out.profile.diagnostics["adaptive.depth_min_covered_probes"] =
        static_cast<double>(adaptive_config.adaptive_depth_min_covered_probes);
    out.profile.diagnostics["adaptive.depth_min_main_probes"] =
        static_cast<double>(adaptive_config.adaptive_depth_min_main_probes);
    out.profile.diagnostics["adaptive.depth_min_main_ratio"] =
        adaptive_config.adaptive_depth_min_main_ratio;
    out.profile.diagnostics["adaptive.depth_min_cells"] =
        static_cast<double>(adaptive_config.adaptive_depth_min_cells);
    out.profile.diagnostics["adaptive.depth_min_main_cells"] =
        static_cast<double>(adaptive_config.adaptive_depth_min_main_cells);
    out.profile.diagnostics["adaptive.depth_max_online_cells"] =
        static_cast<double>(adaptive_config.adaptive_depth_max_online_cells);
    out.profile.diagnostics["adaptive.shallow_free_count"] = static_cast<double>(out.shallow_free_count);
    out.profile.diagnostics["adaptive.shallow_collision_count"] = static_cast<double>(out.shallow_collision_count);
    out.profile.diagnostics["adaptive.free_added"] = static_cast<double>(out.adaptive_free_added);
    out.profile.diagnostics["adaptive.validated"] = static_cast<double>(out.adaptive_validated);
    out.profile.diagnostics["adaptive.splits"] = static_cast<double>(out.adaptive_splits);
	    out.profile.diagnostics["adaptive.deferred"] = static_cast<double>(out.adaptive_deferred);
	    out.profile.diagnostics["adaptive.promoted"] = static_cast<double>(out.adaptive_promoted);
	    out.profile.diagnostics["adaptive.unresolved_domains"] = static_cast<double>(out.unresolved_domains);
    const int final_anchor_cap = adaptive_depth_enabled
        ? std::max(0, adaptive_config.adaptive_depth_anchor_probe_cap)
        : std::max(0, adaptive_config.seed_anchor_probe_cap);
    const int final_anchor_attempts = depth_snapshots.empty()
        ? 0
        : depth_snapshots.back().anchor_probe_attempts;
	    out.profile.diagnostics["adaptive.seed_probe_count"] = static_cast<double>(out.seed_probe_count);
    out.profile.diagnostics["adaptive.seed_probe_free_count"] = static_cast<double>(out.seed_probe_free_count);
    out.profile.diagnostics["adaptive.seed_probe_box_covered"] = static_cast<double>(out.seed_probe_box_covered);
    out.profile.diagnostics["adaptive.seed_probe_anchor_success"] = static_cast<double>(out.seed_probe_anchor_success);
    out.profile.diagnostics["adaptive.seed_probe_main_accessible"] = static_cast<double>(out.seed_probe_main_accessible);
	    out.profile.diagnostics["adaptive.seed_anchor_probe_cap"] = static_cast<double>(final_anchor_cap);
	    out.profile.diagnostics["adaptive.seed_anchor_probe_attempts"] = static_cast<double>(final_anchor_attempts);
    out.profile.diagnostics["adaptive.p_box_covered"] = out.p_box_covered;
    out.profile.diagnostics["adaptive.p_anchor_success"] = out.p_anchor_success;
    out.profile.diagnostics["adaptive.p_main_accessible"] = out.p_main_accessible;
    out.profile.diagnostics["adaptive.p_anchor_to_main_uncovered"] = out.p_anchor_to_main_uncovered;
    out.profile.diagnostics["adaptive.selected_leaf_depth"] = static_cast<double>(out.selected_leaf_depth);
    out.profile.diagnostics["adaptive.depth_readiness_met"] =
        out.adaptive_depth_readiness_met ? 1.0 : 0.0;
	    if (use_partition_backend && adaptive_partition_query_enabled_ && adaptive_partition_) {
	        refresh_adaptive_partition_diagnostics(&out.profile);
	    } else {
	        rebuild_adaptive_partition(partition_config, &out.profile);
	    }
    if (adaptive_partition_ && !adaptive_partition_->empty()) {
        const auto& partition_stats = adaptive_partition_->stats();
        out.partition_cell_count = partition_stats.cells;
        out.partition_grid_cell_count = partition_stats.grid_cells;
        out.partition_non_grid_cell_count = partition_stats.non_grid_cells;
        out.partition_face_index_entries = partition_stats.face_index_entries;
        out.partition_islands = partition_stats.islands;
        out.partition_largest_island = partition_stats.largest_island;
    }
    if (oracle_) {
        const OracleCounters counters = oracle_->counters();
        normalize_external_evidence_diagnostics(out.profile.diagnostics, &counters);
    } else {
        normalize_external_evidence_diagnostics(out.profile.diagnostics);
    }
    record_portal_membership_policy(out.profile.diagnostics, config_.portal_membership_policy);
    out.diagnostics = out.profile.diagnostics;
    last_build_ = out.profile;
    if (config_.database.checkpoint_after_build && database_) {
        database_->checkpoint();
    }
    return out;
}

LeafSweepRefineResult RBFPlanningForest::build_leaf_sweep_refined(
    const std::vector<Obstacle>& obstacles,
    const LeafSweepRefineConfig& refine_config,
    const std::vector<Eigen::VectorXd>& priority_points,
    const std::vector<Eigen::VectorXd>& offline_anchor_points) {
    using Clock = std::chrono::steady_clock;
    const auto total_start = Clock::now();

    LeafSweepRefineResult out;
    LeafSweepConfig leaf_config;
    leaf_config.obstacle_cluster_gap = refine_config.obstacle_cluster_gap;
    leaf_config.n_threads = refine_config.leaf_threads;
    leaf_config.validation_batch_size = refine_config.validation_batch_size;
    leaf_config.timeout_ms = refine_config.leaf_timeout_ms;
    leaf_config.store_group_results = refine_config.store_group_results;
    leaf_config.use_virtual_topology = refine_config.use_virtual_topology;
    leaf_config.parallel_virtual_validation = refine_config.parallel_virtual_validation;
    leaf_config.collision_overlap_prune_min_depth = refine_config.collision_overlap_prune_min_depth;
    leaf_config.collision_overlap_prune_threshold = refine_config.collision_overlap_prune_threshold;
    leaf_config.collision_overlap_prune_ratio_threshold =
        refine_config.collision_overlap_prune_ratio_threshold;

    out.leaf_sweep = build_leaf_sweep(obstacles,
                                      refine_config.leaf_start_depth,
                                      refine_config.leaf_max_depth,
                                      leaf_config);
    const auto priority_prune = prune_leaf_sweep_to_priority(out.leaf_sweep,
                                                             boxes_,
                                                             raw_boxes_,
                                                             priority_points,
                                                             refine_config.priority_prune_radius);
    if (refine_config.priority_prune_radius > 0.0 && !priority_points.empty()) {
        clear_dynamic_collision_cache();
        populate_dynamic_collision_cache(out.leaf_sweep, static_cast<int>(obstacles.size()));
        reserve_existing_boxes();
        adjacency_.clear();
        segment_edges_.clear();
        invalidate_query_cache();
    }
    out.leaf_sweep_ms = out.leaf_sweep.total_ms;
    out.leaf_free_count = static_cast<int>(out.leaf_sweep.free_boxes.size());
    out.leaf_collision_count = static_cast<int>(out.leaf_sweep.collision_boxes.size());

    const double adjacency_tolerance = config_.query.adjacency_tolerance;
    MergerResult leaf_merge_result;
    const auto leaf_merge_start = Clock::now();
    if (config_.enable_merger && !boxes_.empty()) {
        MergerConfig leaf_merge_config = config_.merger;
        leaf_merge_config.exact_face_merge = true;
        leaf_merge_config.greedy_hull_merge = false;
        leaf_merge_config.containment_prune = true;
        leaf_merge_config.adjacency_tolerance = adjacency_tolerance;
        leaf_merge_config.max_rounds = std::max(1, leaf_merge_config.max_rounds);
        leaf_merge_result = fast_exact_face_merge_leaf(*oracle_, boxes_, leaf_merge_config);
        raw_boxes_ = boxes_;
    }
    const double leaf_merge_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - leaf_merge_start).count();
    rebuild_adjacency();
    const auto refine_start = Clock::now();
    Deadline refine_deadline = refine_config.refine_timeout_ms > 0.0
        ? Deadline::after_ms(refine_config.refine_timeout_ms)
        : Deadline{};
    StageContext refine_context = StageContext::from_runtime(config_.runtime, refine_deadline);
    FindFreeBoxOptions refine_options = config_.grower.find_free_box;
    refine_options.max_depth = refine_config.deep_ffb_depth;
    refine_options.reject_seed_collision = false;
    int next_id = next_box_id();
    auto find_in_domain = [this](const Eigen::VectorXd& seed,
                                 const std::vector<Interval>& domain,
                                 StageContext& context,
                                 const FindFreeBoxOptions& options) {
        return this->find_free_box_in_domain(seed, domain, context, options);
    };
    const auto offline_anchors = run_offline_anchor_grower(*oracle_,
                                                           refine_config,
                                                           out.leaf_sweep.collision_boxes,
                                                           offline_anchor_points,
                                                           find_in_domain,
                                                           config_.grower.commit_policy,
                                                           boxes_,
                                                           raw_boxes_,
                                                           adjacency_,
                                                           next_id,
                                                           refine_context,
                                                           refine_options,
                                                           adjacency_tolerance);
    const auto qroot = run_query_root_box_grower(*oracle_,
                                                 refine_config,
                                                 out.leaf_sweep.collision_boxes,
                                                 priority_points,
                                                 find_in_domain,
                                                 config_.grower.commit_policy,
                                                 boxes_,
                                                 raw_boxes_,
                                                 adjacency_,
                                                 next_id,
                                                 refine_context,
                                                 refine_options,
                                                 adjacency_tolerance);
    out.deep_boxes_added = qroot.boxes_added;
    out.deep_domain_attempts = qroot.pair_attempts;
    out.deep_ffb_success = qroot.ffb_success;
    out.deep_ffb_fail = qroot.ffb_fail;
    out.deep_commit_rejects = qroot.commit_rejects;
    out.deep_domain_rejects = qroot.domain_rejects;
    out.deep_contained_rejects = qroot.contained_rejects;
    out.deep_adjacency_rejects = qroot.adjacency_rejects;
    out.deep_anchor_roots_added = qroot.endpoint_anchors_added;
    out.deep_refine_ms = std::chrono::duration<double, std::milli>(Clock::now() - refine_start).count();
    out.rrt_grower_ms = 0.0;
    out.rrt_grower_boxes_added = 0;
    out.rrt_grower_ffb_success = 0;
    out.rrt_grower_ffb_fail = 0;

    const auto connector_start = Clock::now();
    bool connector_ran = false;
    std::unordered_map<std::string, double> connector_diagnostics;
    if (config_.enable_connector && !boxes_.empty()) {
        StageContext connector_context = StageContext::from_runtime(config_.runtime);
        CollisionChecker checker(robot_, scene_);
        IslandConnectorConfig connector_config = config_.connector;
        if (connector_config.n_threads <= 1 && config_.runtime.n_threads > 1) {
            connector_config.n_threads = config_.runtime.n_threads;
        }
        int connector_next_id = next_id;
        IslandConnectorConfig box_only_config = connector_config;
        box_only_config.segment_edges_fallback_only = true;
        {
            IslandConnector connector(*oracle_, robot_, checker, box_only_config);
            const auto connector_result = connector.connect_all(boxes_,
                                                                adjacency_,
                                                                segment_edges_,
                                                                connector_next_id,
                                                                connector_context);
            out.profile.bridge_boxes_added += connector_result.bridge_boxes_added;
            out.profile.connector_attempted_pairs += connector_result.attempted_pairs;
            out.profile.connector_connected = connector_result.connected;
        }
        if (find_islands(adjacency_).size() > 1 &&
            connector_config.segment_edges_enabled &&
            (connector_config.rrt_segment_edges || connector_config.point_gap_segment_edges)) {
            IslandConnectorConfig fallback_config = connector_config;
            fallback_config.segment_edges_fallback_only = false;
            fallback_config.max_total_bridge_boxes = 0;
            fallback_config.max_pairs_per_gap = std::max(fallback_config.max_pairs_per_gap, 4);
            IslandConnector fallback_connector(*oracle_, robot_, checker, fallback_config);
            const auto fallback_result = fallback_connector.connect_all(boxes_,
                                                                        adjacency_,
                                                                        segment_edges_,
                                                                        connector_next_id,
                                                                        connector_context);
            out.profile.bridge_boxes_added += fallback_result.bridge_boxes_added;
            out.profile.segment_edges_added += fallback_result.segment_edges_added;
            out.profile.rrt_segment_edges_added += fallback_result.rrt_segment_edges_added;
            out.profile.point_gap_segment_edges_added += fallback_result.point_gap_segment_edges_added;
            out.profile.connector_attempted_pairs += fallback_result.attempted_pairs;
            out.profile.connector_connected = fallback_result.connected;
        }
        next_id = connector_next_id;
        connector_diagnostics = connector_context.diagnostics().snapshot();
        connector_ran = true;
    }
    out.connector_ms = std::chrono::duration<double, std::milli>(Clock::now() - connector_start).count();
    out.profile.connector_ms = out.connector_ms;

    const auto adjacency_start = Clock::now();
    out.profile.segment_edges = static_cast<int>(segment_edges_.size());
    if (!connector_ran) {
        rebuild_adjacency();
    }
    out.profile.adjacency_ms = std::chrono::duration<double, std::milli>(Clock::now() - adjacency_start).count();
    out.profile.raw_boxes = static_cast<int>(raw_boxes_.size());
    out.profile.final_boxes = static_cast<int>(boxes_.size());
    out.profile.grow_ms = out.leaf_sweep_ms + out.deep_refine_ms + out.rrt_grower_ms;
    out.profile.grow_adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
    out.profile.grow_largest_island = 0;
    for (const auto& island : find_islands(adjacency_)) {
        out.profile.grow_largest_island =
            std::max(out.profile.grow_largest_island, static_cast<int>(island.size()));
    }
    out.profile.adjacency_islands = out.profile.grow_adjacency_islands;
    out.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
    out.profile.total_ms = out.total_ms;
    out.profile.diagnostics = out.leaf_sweep.diagnostics;
    out.profile.diagnostics["leaf_refine.leaf_sweep_ms"] = out.leaf_sweep_ms;
    record_depth_semantics_diagnostics(out.profile.diagnostics,
                                       "leaf_refine.",
                                       refine_config.leaf_start_depth,
                                       refine_config.leaf_max_depth,
                                       refine_config.deep_ffb_depth,
                                       config_.grower.find_free_box,
                                       refine_config.deep_ffb_depth);
    out.profile.diagnostics["leaf_refine.leaf_free_count"] = static_cast<double>(out.leaf_free_count);
    out.profile.diagnostics["leaf_refine.leaf_collision_count"] = static_cast<double>(out.leaf_collision_count);
    out.profile.diagnostics["leaf_refine.priority_prune_radius"] = refine_config.priority_prune_radius;
    out.profile.diagnostics["leaf_refine.collision_overlap_prune_min_depth"] =
        static_cast<double>(refine_config.collision_overlap_prune_min_depth);
    out.profile.diagnostics["leaf_refine.collision_overlap_prune_threshold"] =
        refine_config.collision_overlap_prune_threshold;
    out.profile.diagnostics["leaf_refine.collision_overlap_prune_ratio_threshold"] =
        refine_config.collision_overlap_prune_ratio_threshold;
    out.profile.diagnostics["leaf_refine.priority_prune_free_before"] =
        static_cast<double>(priority_prune.free_before);
    out.profile.diagnostics["leaf_refine.priority_prune_free_after"] =
        static_cast<double>(priority_prune.free_after);
    out.profile.diagnostics["leaf_refine.priority_prune_collision_before"] =
        static_cast<double>(priority_prune.collision_before);
    out.profile.diagnostics["leaf_refine.priority_prune_collision_after"] =
        static_cast<double>(priority_prune.collision_after);
    out.profile.diagnostics["leaf_refine.leaf_merge_ms"] = leaf_merge_ms;
    out.profile.diagnostics["leaf_refine.leaf_merge_boxes_before"] =
        static_cast<double>(leaf_merge_result.boxes_before);
    out.profile.diagnostics["leaf_refine.leaf_merge_boxes_after"] =
        static_cast<double>(leaf_merge_result.boxes_after);
    out.profile.diagnostics["leaf_refine.leaf_merge_exact"] =
        static_cast<double>(leaf_merge_result.exact_merges);
    out.profile.diagnostics["leaf_refine.leaf_merge_pruned"] =
        static_cast<double>(leaf_merge_result.pruned_boxes);
    out.profile.diagnostics["leaf_refine.collision_cache_boxes"] =
        static_cast<double>(dynamic_collision_box_cache_.size());
    out.profile.diagnostics["leaf_refine.deep_refine_ms"] = out.deep_refine_ms;
    out.profile.diagnostics["leaf_refine.offline_anchor_ms"] = offline_anchors.total_ms;
    out.profile.diagnostics["leaf_refine.offline_anchor_candidates"] =
        static_cast<double>(offline_anchors.candidates_total);
    out.profile.diagnostics["leaf_refine.offline_anchor_candidates_covered"] =
        static_cast<double>(offline_anchors.candidates_covered);
    out.profile.diagnostics["leaf_refine.offline_anchor_roots_added"] =
        static_cast<double>(offline_anchors.boxes_added);
    out.profile.diagnostics["leaf_refine.offline_anchor_ffb_success"] =
        static_cast<double>(offline_anchors.ffb_success);
    out.profile.diagnostics["leaf_refine.offline_anchor_ffb_fail"] =
        static_cast<double>(offline_anchors.ffb_fail);
    out.profile.diagnostics["leaf_refine.offline_anchor_commit_rejects"] =
        static_cast<double>(offline_anchors.commit_rejects);
    out.profile.diagnostics["leaf_refine.offline_anchor_domain_rejects"] =
        static_cast<double>(offline_anchors.domain_rejects);
    out.profile.diagnostics["leaf_refine.offline_anchor_contained_rejects"] =
        static_cast<double>(offline_anchors.contained_rejects);
    out.profile.diagnostics["leaf_refine.offline_anchor_adjacency_rejects"] =
        static_cast<double>(offline_anchors.adjacency_rejects);
    out.profile.diagnostics["leaf_refine.offline_anchor_adjacency_candidates_tested"] =
        static_cast<double>(offline_anchors.adjacency_candidates_tested);
    out.profile.diagnostics["leaf_refine.offline_anchor_adjacency_edges_added"] =
        static_cast<double>(offline_anchors.adjacency_edges_added);
    out.profile.diagnostics["leaf_refine.offline_anchor_islands_before"] =
        static_cast<double>(offline_anchors.islands_before);
    out.profile.diagnostics["leaf_refine.offline_anchor_islands_after"] =
        static_cast<double>(offline_anchors.islands_after);
    out.profile.diagnostics["leaf_refine.offline_anchor_box_volume_mean"] =
        offline_anchors.boxes_added > 0
            ? offline_anchors.box_volume_sum / static_cast<double>(offline_anchors.boxes_added)
            : 0.0;
    out.profile.diagnostics["leaf_refine.offline_anchor_box_volume_max"] =
        offline_anchors.box_volume_max;
    out.profile.diagnostics["leaf_refine.offline_anchor_index_rebuild_ms"] =
        offline_anchors.index_rebuild_ms;
    out.profile.diagnostics["leaf_refine.offline_anchor_index_query_ms"] =
        offline_anchors.index_query_ms;
    out.profile.diagnostics["leaf_refine.deep_boxes_added"] = static_cast<double>(out.deep_boxes_added);
    out.profile.diagnostics["leaf_refine.deep_domain_attempts"] = static_cast<double>(out.deep_domain_attempts);
    out.profile.diagnostics["leaf_refine.deep_ffb_success"] = static_cast<double>(out.deep_ffb_success);
    out.profile.diagnostics["leaf_refine.deep_ffb_fail"] = static_cast<double>(out.deep_ffb_fail);
    out.profile.diagnostics["leaf_refine.deep_commit_rejects"] = static_cast<double>(out.deep_commit_rejects);
    out.profile.diagnostics["leaf_refine.deep_domain_rejects"] = static_cast<double>(out.deep_domain_rejects);
    out.profile.diagnostics["leaf_refine.deep_contained_rejects"] = static_cast<double>(out.deep_contained_rejects);
    out.profile.diagnostics["leaf_refine.deep_adjacency_rejects"] = static_cast<double>(out.deep_adjacency_rejects);
    out.profile.diagnostics["leaf_refine.deep_anchor_roots_added"] = static_cast<double>(out.deep_anchor_roots_added);
    out.profile.diagnostics["leaf_refine.qroot_ms"] = qroot.total_ms;
    out.profile.diagnostics["leaf_refine.qroot_pairs_total"] = static_cast<double>(qroot.pairs_total);
    out.profile.diagnostics["leaf_refine.qroot_pairs_connected_before"] =
        static_cast<double>(qroot.pairs_connected_before);
    out.profile.diagnostics["leaf_refine.qroot_pairs_connected_after"] =
        static_cast<double>(qroot.pairs_connected_after);
    out.profile.diagnostics["leaf_refine.qroot_uncovered_endpoints"] =
        static_cast<double>(qroot.uncovered_endpoints);
	out.profile.diagnostics["leaf_refine.qroot_endpoint_anchors_added"] =
		static_cast<double>(qroot.endpoint_anchors_added);
	out.profile.diagnostics["leaf_refine.qroot_endpoint_root_fallbacks"] =
		static_cast<double>(qroot.endpoint_root_fallbacks);
	out.profile.diagnostics["leaf_refine.qroot_boxes_added"] = static_cast<double>(qroot.boxes_added);
    out.profile.diagnostics["leaf_refine.qroot_ffb_success"] = static_cast<double>(qroot.ffb_success);
    out.profile.diagnostics["leaf_refine.qroot_ffb_fail"] = static_cast<double>(qroot.ffb_fail);
    out.profile.diagnostics["leaf_refine.qroot_adjacency_candidates_tested"] =
        static_cast<double>(qroot.adjacency_candidates_tested);
    out.profile.diagnostics["leaf_refine.qroot_adjacency_edges_added"] =
        static_cast<double>(qroot.adjacency_edges_added);
    out.profile.diagnostics["leaf_refine.qroot_index_rebuild_ms"] = qroot.index_rebuild_ms;
    out.profile.diagnostics["leaf_refine.qroot_index_query_ms"] = qroot.index_query_ms;
    out.profile.diagnostics["leaf_refine.qroot_islands_before"] = static_cast<double>(qroot.islands_before);
    out.profile.diagnostics["leaf_refine.qroot_islands_after"] = static_cast<double>(qroot.islands_after);
    out.profile.diagnostics["leaf_refine.rrt_grower_ms"] = out.rrt_grower_ms;
    out.profile.diagnostics["leaf_refine.rrt_grower_initial_boxes"] = 0.0;
    out.profile.diagnostics["leaf_refine.rrt_grower_boxes_added"] = static_cast<double>(out.rrt_grower_boxes_added);
    out.profile.diagnostics["leaf_refine.rrt_grower_ffb_success"] = static_cast<double>(out.rrt_grower_ffb_success);
    out.profile.diagnostics["leaf_refine.rrt_grower_ffb_fail"] = static_cast<double>(out.rrt_grower_ffb_fail);
    out.profile.diagnostics["leaf_refine.rrt_grower_deadline_reached"] = 0.0;
    out.profile.diagnostics["leaf_refine.connector_ms"] = out.connector_ms;
    for (const auto& [key, value] : connector_diagnostics) {
        out.profile.diagnostics[std::string("leaf_refine.") + key] = value;
    }
    out.profile.diagnostics["leaf_refine.total_ms"] = out.total_ms;
    record_portal_membership_policy(out.profile.diagnostics, config_.portal_membership_policy);
    out.diagnostics = out.profile.diagnostics;
    last_build_ = out.profile;
    invalidate_query_cache();
    if (config_.database.checkpoint_after_build && database_) {
        database_->checkpoint();
    }
    return out;
}

QueryResult RBFPlanningForest::query(const Eigen::Ref<const Eigen::VectorXd>& start,
                                 const Eigen::Ref<const Eigen::VectorXd>& goal) const {
    return run_query_internal(start, goal, true);
}

QueryResult RBFPlanningForest::run_query_internal(const Eigen::Ref<const Eigen::VectorXd>& start,
                                              const Eigen::Ref<const Eigen::VectorXd>& goal,
                                              bool allow_collision_shortcut) const {
    using Clock = std::chrono::steady_clock;
    QueryConfig query_config = config_.query;
    if (!allow_collision_shortcut) {
        query_config.collision_shortcut = false;
    }
    const bool do_collision_shortcut = query_config.collision_shortcut;
    const int active_query_index = env_int_or_default("RBF_ACTIVE_QUERY_INDEX", -1);
    const bool partition_last_query_cache_enabled =
        partition_native_mode() &&
        adaptive_partition_query_enabled_ &&
        adaptive_partition_ &&
        env_int_or_default("RBF_PARTITION_LAST_QUERY_CACHE", 0) != 0;
    auto same_vector = [](const Eigen::VectorXd& lhs,
                          const Eigen::Ref<const Eigen::VectorXd>& rhs) {
        return lhs.size() == rhs.size() &&
               (lhs.size() == 0 || (lhs - rhs).cwiseAbs().maxCoeff() <= 0.0);
    };
    if (partition_last_query_cache_enabled &&
        partition_last_query_cache_.valid &&
        partition_last_query_cache_.allow_collision_shortcut == allow_collision_shortcut &&
        partition_last_query_cache_.active_query_index == active_query_index &&
        same_vector(partition_last_query_cache_.start, start) &&
        same_vector(partition_last_query_cache_.goal, goal)) {
        QueryResult cached = partition_last_query_cache_.result;
        cached.query_time_ms = 0.0;
        cached.partition_search_ms = 0.0;
        cached.audit_time_ms = 0.0;
        cached.final_simplify_time_ms = 0.0;
        return cached;
    }
    QueryResult result;
    QueryResult partition_attempt;
    if (adaptive_partition_query_enabled_ &&
        adaptive_partition_ &&
        !adaptive_partition_->empty()) {
        AdaptiveGridPartitionQueryOptions partition_options;
        partition_options.nearest_if_outside = query_config.nearest_if_outside;
        partition_options.shortcut_boxes = query_config.shortcut_boxes;
        partition_options.max_expansions = last_build_.diagnostics.count("adaptive.grid_planning_max_expansions") > 0
            ? static_cast<int>(last_build_.diagnostics.at("adaptive.grid_planning_max_expansions"))
            : 0;
        partition_options.adjacency_tolerance = query_config.adjacency_tolerance;
        const auto partition_result = adaptive_partition_->query(start, goal, partition_options);
        result.start_box_id = partition_result.start_box_id;
        result.goal_box_id = partition_result.goal_box_id;
        result.partition_search_ms = partition_result.search_ms;
        result.query_time_ms = partition_result.search_ms;
        result.non_grid_cells_used = partition_result.non_grid_cells_used;
        if (partition_result.found) {
            result.success = true;
            result.box_sequence = partition_result.box_sequence;
            result.segment_edge_sequence = partition_result.segment_edge_sequence;
            if (result.segment_edge_sequence.size() + 1 != result.box_sequence.size()) {
                result.segment_edge_sequence.assign(
                    result.box_sequence.size() > 0 ? result.box_sequence.size() - 1 : 0,
                    -1);
            }
            result.partition_cells_used = static_cast<int>(result.box_sequence.size());
            result.path = partition_result.path;
            if (result.path.empty()) {
                if (partition_native_mode()) {
                    throw std::runtime_error(
                        "partition_native query returned a box sequence without partition waypoints");
                }
                result.path = extract_partition_waypoints_local(result.box_sequence,
                                                                result.segment_edge_sequence,
                                                                boxes_,
                                                                segment_edges_,
                                                                start,
                                                                goal,
                                                                query_config.adjacency_tolerance);
            }
            result.path_length = path_length(result.path);
            result.raw_path_length = result.path_length;
        }
        partition_attempt = result;
    }
    if (!result.success && !partition_native_mode()) {
        CorridorQuery query_engine(query_config);
        QueryResult graph_result = query_engine.run(query_cache(), start, goal);
        graph_result.partition_search_ms = partition_attempt.partition_search_ms;
        graph_result.partition_repair_ms = partition_attempt.partition_repair_ms;
        graph_result.partition_cells_used = partition_attempt.partition_cells_used;
        graph_result.non_grid_cells_used = partition_attempt.non_grid_cells_used;
        result = std::move(graph_result);
    }
    if (result.success && do_collision_shortcut && !query_config.strict_path_audit && result.path.size() > 2) {
        CollisionChecker checker = make_audit_checker(audit_robot_, scene_, query_config);
        result.path = collision_shortcut_path(result.path,
                                             checker,
                                             collision_shortcut_resolution(query_config));
        result.path_length = path_length(result.path);
    }
    summarize_query_path(result, boxes_, segment_edges_);
    if (!result.success && query_config.strict_path_audit && query_config.repair_on_audit_failure) {
        CollisionChecker checker = make_audit_checker(audit_robot_, scene_, query_config);
        const auto repair_t0 = Clock::now();
        RRTConnectConfig repair_config = config_.connector.rrt;
        repair_config.max_iters = std::max(repair_config.max_iters, query_config.repair_rrt_max_iters);
        if (query_config.repair_timeout_ms > 0.0) {
            repair_config.timeout_ms = query_config.repair_timeout_ms;
        }
        repair_config.segment_resolution = std::max(repair_config.segment_resolution, query_config.audit_resolution);
        std::vector<Eigen::VectorXd> repair_path = rrt_connect(
            start,
            goal,
            checker,
            audit_robot_,
            repair_config,
            derived_planner_seed(config_.grower.rng_seed, kSeedRepairGlobalOffset));
        if (!repair_path.empty()) {
            PathAuditCheck repair_audit = audit_waypoint_path(repair_path,
                                                             checker,
                                                             query_config.audit_resolution,
                                                             query_config.audit_segment_step);
            if (repair_audit.passed && do_collision_shortcut && repair_path.size() > 2) {
                std::vector<Eigen::VectorXd> shortened = collision_shortcut_path(
                    repair_path,
                    checker,
                    collision_shortcut_resolution(query_config));
                PathAuditCheck shortened_audit = audit_waypoint_path(shortened,
                                                                     checker,
                                                                     query_config.audit_resolution,
                                                                     query_config.audit_segment_step);
                if (shortened_audit.passed && path_length(shortened) <= path_length(repair_path) + 1e-12) {
                    repair_path = std::move(shortened);
                    repair_audit = shortened_audit;
                }
            }
            if (repair_audit.passed) {
                result.success = true;
                result.path = std::move(repair_path);
                result.path_length = path_length(result.path);
                result.raw_path_length = result.path_length;
                result.repair_count += 1;
                result.audit_status = PathAuditStatus::Repaired;
                result.audit_passed = true;
                result.failed_segment_index = repair_audit.failed_segment_index;
                result.remaining_unsafe_assumptions = 0;
            }
        }
        result.repair_time_ms += std::chrono::duration<double, std::milli>(Clock::now() - repair_t0).count();
    }
    if (result.success && query_config.strict_path_audit) {
        CollisionChecker checker = make_audit_checker(audit_robot_, scene_, query_config);
        auto try_final_simplify = [&]() {
            if (!query_config.final_rrt_simplify ||
                !(query_config.final_rrt_simplify_timeout_ms > 0.0) ||
                result.path_length <= 0.0) {
                return;
            }
            const auto simplify_t0 = Clock::now();
            auto simplify_elapsed_ms = [&]() {
                return std::chrono::duration<double, std::milli>(Clock::now() - simplify_t0).count();
            };
            RRTConnectConfig simplify_config = config_.connector.rrt;
            // Final OMPL-style simplification is audited in native C-space and is
            // intentionally not restricted to one active LECT root sector. The
            // raw box/segment statistics are computed before this replacement.
            simplify_config.max_iters = std::max(1, query_config.final_rrt_simplify_max_iters);
            simplify_config.segment_resolution = std::max(simplify_config.segment_resolution,
                                                          query_config.audit_resolution);
            simplify_config.segment_step = query_config.audit_segment_step;
            simplify_config.shortcut_path = true;
            const int attempts = std::max(1, query_config.final_rrt_simplify_attempts);
            for (int attempt = 0; attempt < attempts; ++attempt) {
                const double remaining_ms = query_config.final_rrt_simplify_timeout_ms - simplify_elapsed_ms();
                if (remaining_ms <= 0.0) {
                    break;
                }
                const int attempts_left = attempts - attempt;
                simplify_config.timeout_ms = std::max(1.0, remaining_ms / static_cast<double>(attempts_left));
                std::vector<Eigen::VectorXd> simplified = rrt_connect(start,
                                                                      goal,
                                                                      checker,
                                                                      audit_robot_,
                                                                      simplify_config,
                                                                      derived_planner_seed(config_.grower.rng_seed,
                                                                                           kSeedFinalSimplifyOffset,
                                                                                           attempt));
                if (!simplified.empty()) {
                    PathAuditCheck simplified_audit = audit_waypoint_path(simplified,
                                                                          checker,
                                                                          query_config.audit_resolution,
                                                                          query_config.audit_segment_step);
                    const double simplified_length = path_length(simplified);
                    if (simplified_audit.passed &&
                        simplified_length + 1e-12 < result.path_length) {
                        result.path = std::move(simplified);
                        result.path_length = simplified_length;
                        result.failed_segment_index = simplified_audit.failed_segment_index;
                        result.audit_passed = true;
                        result.audit_status = result.repair_count > 0 ? PathAuditStatus::Repaired : PathAuditStatus::Passed;
                    }
                }
            }
            const double simplify_ms =
                std::chrono::duration<double, std::milli>(Clock::now() - simplify_t0).count();
            result.final_simplify_time_ms += simplify_ms;
        };
        const auto audit_t0 = Clock::now();
        PathAuditCheck audit = audit_waypoint_path(result.path,
                                                   checker,
                                                   query_config.audit_resolution,
                                                   query_config.audit_segment_step);
        result.failed_segment_index = audit.failed_segment_index;
        if (audit.passed) {
            if (do_collision_shortcut && result.path.size() > 2) {
                std::vector<Eigen::VectorXd> shortened = collision_shortcut_path(
                    result.path,
                    checker,
                    collision_shortcut_resolution(query_config));
                PathAuditCheck shortened_audit = audit_waypoint_path(shortened,
                                                                     checker,
                                                                     query_config.audit_resolution,
                                                                     query_config.audit_segment_step);
                if (shortened_audit.passed && path_length(shortened) <= result.path_length + 1e-12) {
                    result.path = std::move(shortened);
                    result.path_length = path_length(result.path);
                    audit = shortened_audit;
                    result.failed_segment_index = audit.failed_segment_index;
                }
            }
            result.audit_status = result.repair_count > 0 ? PathAuditStatus::Repaired : PathAuditStatus::Passed;
            result.audit_passed = true;
            result.remaining_unsafe_assumptions = 0;
            try_final_simplify();
        } else if (query_config.repair_on_audit_failure) {
            const auto repair_t0 = Clock::now();
            const RRTConnectConfig repair_domain_config = oracle_
                ? with_query_root_hull_domain(config_.connector.rrt, *oracle_, start, goal)
                : config_.connector.rrt;
            const bool repaired = try_local_birrt_repair(result,
                                                         audit,
                                                         checker,
                                                         audit_robot_,
                                                         query_config,
                                                         repair_domain_config,
                                                         config_.grower.rng_seed);
            result.repair_time_ms = std::chrono::duration<double, std::milli>(Clock::now() - repair_t0).count();
            if (repaired) {
                PathAuditCheck repaired_audit = audit_waypoint_path(result.path,
                                                                    checker,
                                                                    query_config.audit_resolution,
                                                                    query_config.audit_segment_step);
                if (repaired_audit.passed && do_collision_shortcut && result.path.size() > 2) {
                    std::vector<Eigen::VectorXd> shortened = collision_shortcut_path(
                        result.path,
                        checker,
                        collision_shortcut_resolution(query_config));
                    PathAuditCheck shortened_audit = audit_waypoint_path(shortened,
                                                                         checker,
                                                                         query_config.audit_resolution,
                                                                         query_config.audit_segment_step);
                    if (shortened_audit.passed && path_length(shortened) <= result.path_length + 1e-12) {
                        result.path = std::move(shortened);
                        result.path_length = path_length(result.path);
                        repaired_audit = shortened_audit;
                    }
                }
                result.failed_segment_index = repaired_audit.failed_segment_index;
                result.audit_status = repaired_audit.passed ? PathAuditStatus::Repaired : PathAuditStatus::Failed;
                result.audit_passed = repaired_audit.passed;
                result.success = repaired_audit.passed;
                if (repaired_audit.passed) {
                    result.remaining_unsafe_assumptions = 0;
                    try_final_simplify();
                }
            } else {
                result.audit_status = PathAuditStatus::Failed;
                result.audit_passed = false;
                result.success = false;
            }
        } else {
            result.audit_status = PathAuditStatus::Failed;
            result.audit_passed = false;
            result.success = false;
        }
        result.audit_time_ms = std::chrono::duration<double, std::milli>(Clock::now() - audit_t0).count();
        summarize_query_path(result, boxes_, segment_edges_);
        if (result.audit_passed) {
            result.remaining_unsafe_assumptions = 0;
        }
    }
    if (partition_last_query_cache_enabled) {
        partition_last_query_cache_.valid = true;
        partition_last_query_cache_.allow_collision_shortcut = allow_collision_shortcut;
        partition_last_query_cache_.active_query_index = active_query_index;
        partition_last_query_cache_.start = start;
        partition_last_query_cache_.goal = goal;
        partition_last_query_cache_.result = result;
    }
    return result;
}

int RBFPlanningForest::add_partition_portal_corridor_overlay(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    const std::vector<Eigen::VectorXd>& waypoint_path,
    const char* diagnostic_prefix,
    bool anchor_endpoints,
    bool skip_if_connected,
    int query_index,
    BuildProfile* profile) {
    BuildProfile* out_profile = profile != nullptr ? profile : &last_build_;
    const std::string prefix =
        (diagnostic_prefix != nullptr && diagnostic_prefix[0] != '\0')
            ? std::string(diagnostic_prefix)
            : std::string("partition_portal_corridor");
    auto& diagnostics = out_profile->diagnostics;
    diagnostics[prefix + ".portal_corridor_attempts"] += 1.0;
    const bool online_portal_prefix =
        prefix.find("hipac_online") != std::string::npos ||
        prefix.find("hipac_promote") != std::string::npos;

    if (!last_adaptive_partition_config_.hipac_portal_connectivity &&
        !(online_portal_prefix && last_adaptive_partition_config_.hipac_online_connectivity)) {
        diagnostics[prefix + ".portal_corridor_disabled"] += 1.0;
        return 0;
    }
    if (!partition_native_mode()) {
        diagnostics[prefix + ".portal_corridor_not_partition_native"] += 1.0;
        return 0;
    }
    if (!adaptive_partition_query_enabled_ || !adaptive_partition_ || adaptive_partition_->empty()) {
        diagnostics[prefix + ".portal_corridor_missing_partition"] += 1.0;
        return 0;
    }
    if (!oracle_ || waypoint_path.size() < 2) {
        diagnostics[prefix + ".portal_corridor_missing_waypoints"] += 1.0;
        return 0;
    }

    StageContext anchor_context = StageContext::from_runtime(config_.runtime);
    const std::size_t boxes_before_anchor = boxes_.size();
    int source_box_id = locate_box_partition_first(start, false);
    if (source_box_id < 0 && anchor_endpoints) {
        source_box_id = anchor_query_endpoint_box(start, anchor_context);
    }
    int target_box_id = locate_box_partition_first(goal, false);
    if (target_box_id < 0 && anchor_endpoints) {
        target_box_id = anchor_query_endpoint_box(goal, anchor_context);
    }
    for (const auto& [key, value] : anchor_context.diagnostics().snapshot()) {
        diagnostics[prefix + ".portal_corridor_anchor." + key] += value;
    }

    int anchors_added = 0;
    if (boxes_.size() > boxes_before_anchor) {
        const std::string anchor_prefix = prefix + ".portal_corridor_anchor";
        anchors_added = append_adaptive_partition_boxes(boxes_before_anchor,
                                                        out_profile,
                                                        anchor_prefix.c_str());
        source_box_id = locate_box_partition_first(start, false);
        target_box_id = locate_box_partition_first(goal, false);
    }
    if (anchors_added > 0) {
        diagnostics[prefix + ".portal_corridor_anchor_boxes_added"] +=
            static_cast<double>(anchors_added);
    }
    if (source_box_id < 0 || target_box_id < 0) {
        diagnostics[prefix + ".portal_corridor_missing_endpoint"] += 1.0;
        return anchors_added;
    }
    if (source_box_id == target_box_id ||
        (skip_if_connected && overlay_path_connected_partition_first(source_box_id, target_box_id))) {
        diagnostics[prefix + ".portal_corridor_already_connected"] += 1.0;
        return anchors_added;
    }

    const BoxNode* source_ptr = find_box_by_id(boxes_, source_box_id);
    const BoxNode* target_ptr = find_box_by_id(boxes_, target_box_id);
    if (source_ptr == nullptr || target_ptr == nullptr) {
        diagnostics[prefix + ".portal_corridor_missing_box"] += 1.0;
        return anchors_added;
    }
    const BoxNode source_box = *source_ptr;
    const BoxNode target_box = *target_ptr;

    const bool online_portal = online_portal_prefix;
    const auto domain = oracle_->planning_intervals();
    const bool transition_obb_prefix =
        prefix.find("hipac_online_transition") != std::string::npos ||
        prefix.find("hipac_promote_transition") != std::string::npos;
    if (transition_obb_prefix && last_adaptive_partition_config_.hipac_transition_obb_portal) {
        auto obb_t0 = std::chrono::steady_clock::now();
        std::vector<Eigen::VectorXd> obb_path;
        obb_path.reserve(waypoint_path.size() + 2U);
        auto append_unique = [&](const Eigen::VectorXd& waypoint) {
            if (waypoint.size() != start.size()) {
                return;
            }
            if (obb_path.empty() || (obb_path.back() - waypoint).norm() > 1e-12) {
                obb_path.push_back(waypoint);
            }
        };
        append_unique(start);
        for (const auto& waypoint : waypoint_path) {
            append_unique(waypoint);
        }
        append_unique(goal);

        ObbPortalValidationStats obb_stats;
        const double obb_safety_epsilon =
            std::max(0.0, last_adaptive_partition_config_.hipac_transition_obb_safety_epsilon);
        diagnostics[prefix + ".obb_zonotope_attempts"] += 1.0;
        Eigen::VectorXd obb_center;
        Eigen::MatrixXd obb_generators;
        const bool obb_ok = validate_obb_zonotope_portal(
            robot_,
            scene_,
            domain,
            obb_path,
            last_adaptive_partition_config_.hipac_transition_obb_lateral_radius,
            last_adaptive_partition_config_.hipac_transition_obb_longitudinal_margin,
            obb_safety_epsilon,
            last_adaptive_partition_config_.segment_edge_obb_grow_iterations,
            last_adaptive_partition_config_.segment_edge_obb_binary_iterations,
            last_adaptive_partition_config_.obb_max_validations_per_window,
            obb_stats,
            &obb_center,
            &obb_generators);
        diagnostics[prefix + ".obb_zonotope_variables"] =
            static_cast<double>(obb_stats.variables);
        diagnostics[prefix + ".obb_zonotope_active_links"] =
            static_cast<double>(obb_stats.active_links);
        diagnostics[prefix + ".obb_zonotope_longitudinal_radius"] =
            obb_stats.longitudinal_radius;
        diagnostics[prefix + ".obb_zonotope_lateral_radius"] =
            obb_stats.lateral_radius;
        diagnostics[prefix + ".obb_zonotope_joint_limit_rejects"] +=
            static_cast<double>(obb_stats.joint_limit_rejects);
        diagnostics[prefix + ".obb_zonotope_degenerate_rejects"] +=
            static_cast<double>(obb_stats.degenerate_rejects);
        diagnostics[prefix + ".obb_zonotope_aabb_tests"] +=
            static_cast<double>(obb_stats.aabb_tests);
        diagnostics[prefix + ".obb_zonotope_aabb_rejects"] +=
            static_cast<double>(obb_stats.aabb_rejects);
        diagnostics[prefix + ".obb_zonotope_gjk_tests"] +=
            static_cast<double>(obb_stats.gjk_tests);
        diagnostics[prefix + ".obb_zonotope_gjk_rejects"] +=
            static_cast<double>(obb_stats.gjk_rejects);
        diagnostics[prefix + ".obb_zonotope_gjk_iterations"] +=
            static_cast<double>(obb_stats.gjk_iterations);
        diagnostics[prefix + ".obb_zonotope_maybe_pairs"] +=
            static_cast<double>(obb_stats.maybe_pairs);
        diagnostics[prefix + ".obb_zonotope_waypoints"] +=
            static_cast<double>(obb_path.size());
        const double obb_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - obb_t0).count();
        diagnostics[prefix + ".obb_zonotope_ms"] += obb_ms;
        if (obb_ok) {
            const int edge_id = append_certified_portal_corridor_edge(
                segment_edges_,
                source_box,
                target_box,
                std::move(obb_path),
                SegmentEdgeValidation::ConservativeObbZonotope,
                -1,
                query_index,
                &obb_center,
                &obb_generators,
                SegmentEdgeType::TransitionOBBCorridor);
            if (edge_id >= 0) {
                const std::string edge_prefix = prefix + ".partition_native_obb_zonotope_portal";
                sync_adaptive_partition_segment_edges(out_profile, edge_prefix.c_str());
                diagnostics[prefix + ".obb_zonotope_success"] += 1.0;
                diagnostics[prefix + ".portal_corridor_added"] += 1.0;
                diagnostics[prefix + ".portal_corridor_obb_zonotope_added"] += 1.0;
                invalidate_query_cache();
                return anchors_added + 1;
            }
            diagnostics[prefix + ".obb_zonotope_edge_fail"] += 1.0;
        } else {
            diagnostics[prefix + ".obb_zonotope_fail"] += 1.0;
        }
    }

    const int max_internal_boxes = online_portal
        ? std::max(0, last_adaptive_partition_config_.hipac_online_max_hidden_boxes_per_portal)
        : std::max(0, last_adaptive_partition_config_.hipac_portal_max_internal_boxes);
    const int max_recursion_depth =
        std::max(0, last_adaptive_partition_config_.hipac_portal_max_recursion_depth);
    if (max_internal_boxes <= 0) {
        diagnostics[prefix + ".portal_corridor_internal_cap_zero"] += 1.0;
        return anchors_added;
    }
    const int requested_depth = last_adaptive_partition_config_.hipac_portal_ffb_depth > 0
        ? last_adaptive_partition_config_.hipac_portal_ffb_depth
        : std::max({config_.query_bridge_pave_depth,
                    config_.connector.pave.find_free_box.max_depth,
                    last_adaptive_partition_config_.target_max_depth});

    StageContext context = StageContext::from_runtime(config_.runtime);
    const double tol = config_.query.adjacency_tolerance;
    std::vector<BoxNode> internal_boxes;
    internal_boxes.reserve(static_cast<std::size_t>(std::min(max_internal_boxes, 32)));
    int next_internal_id = -1000000;
    bool chain_ok = false;

    auto append_portal_if_ready = [&]() -> int {
        if (!chain_ok || internal_boxes.empty()) {
            diagnostics[prefix + ".portal_corridor_chain_fail"] += 1.0;
            return anchors_added;
        }

        const int edge_id = append_portal_corridor_edge(segment_edges_,
                                                        source_box,
                                                        target_box,
                                                        std::move(internal_boxes),
                                                        -1,
                                                        tol,
                                                        query_index);
        if (edge_id < 0) {
            diagnostics[prefix + ".portal_corridor_edge_fail"] += 1.0;
            return anchors_added;
        }

        const std::string edge_prefix = prefix + ".partition_native_portal";
        sync_adaptive_partition_segment_edges(out_profile, edge_prefix.c_str());
        diagnostics[prefix + ".portal_corridor_added"] += 1.0;
        invalidate_query_cache();
        return anchors_added + 1;
    };

    auto build_cell_native_chain = [&]() -> bool {
        const auto& split_descriptor = oracle_->database().split_policy_descriptor();
        const int dims = oracle_->n_dims();
        const int max_cell_depth =
            std::max(1, std::min({requested_depth,
                                  config_.database.max_tree_depth,
                                  oracle_->max_tree_depth() - 1,
                                  60}));
        const int selected_build_depth =
            out_profile != nullptr
                ? static_cast<int>(std::round(
                      diagnostic_map_value(out_profile->diagnostics, "adaptive.selected_leaf_depth")))
                : 0;
        const int min_cell_depth = std::max(
            1,
            std::min(max_cell_depth,
                     selected_build_depth > 0
                         ? selected_build_depth
                         : std::max(1, last_adaptive_partition_config_.shallow_max_depth)));
        std::vector<int> candidate_depths;
        for (int depth = min_cell_depth; depth < max_cell_depth; depth += 4) {
            candidate_depths.push_back(depth);
        }
        if (candidate_depths.empty() || candidate_depths.back() != max_cell_depth) {
            candidate_depths.push_back(max_cell_depth);
        }
        struct NativeCellCacheEntry {
            bool free = false;
            OracleNodeId node = kInvalidOracleNodeId;
            std::vector<Interval> intervals;
            BoxSafetyStatus safety_status = BoxSafetyStatus::Unknown;
            bool strict_audit_required = false;
        };
        std::unordered_map<std::string, NativeCellCacheEntry> cell_cache;
        cell_cache.reserve(static_cast<std::size_t>(max_internal_boxes * 4 + 16));
        int cell_validations = 0;
        int cell_free = 0;
        int cell_not_free = 0;
        int cell_invalid = 0;
        int cell_cache_hits = 0;
        int non_adjacent = 0;
        int recursion_splits = 0;
        int internal_cap_hits = 0;

        auto cache_key = [](OracleNodeId node, const std::vector<Interval>& intervals) {
            return std::to_string(node) + ":" +
                   std::to_string(lect_database::fingerprint_intervals(intervals));
        };

        auto classify_cell_at_point_at_depth = [&](const Eigen::VectorXd& point,
                                                   int cell_depth,
                                                   BoxNode& candidate) -> bool {
            if (point.size() != dims ||
                !oracle_->contains_point(oracle_->root_node(), point) ||
                !intervals_contain_point_strict_local(domain, point, 1e-12)) {
                ++cell_invalid;
                return false;
            }
            Eigen::VectorXd tree_seed = oracle_->tree_configuration_for_query(point);
            if (tree_seed.size() != dims) {
                ++cell_invalid;
                return false;
            }
            OracleNodeId node = oracle_->root_node();
            std::vector<Interval> tree_intervals = oracle_->node_intervals(node);
            int changed_dim = -1;
            for (int level = 0; level < cell_depth; ++level) {
                int split_dim = -1;
                if (!split_descriptor.depth_dimensions.empty() &&
                    level < static_cast<int>(split_descriptor.depth_dimensions.size())) {
                    split_dim = split_descriptor.depth_dimensions[static_cast<std::size_t>(level)];
                } else {
                    split_dim = level % dims;
                }
                if (split_dim < 0 ||
                    split_dim >= dims ||
                    split_dim >= static_cast<int>(tree_intervals.size())) {
                    ++cell_invalid;
                    return false;
                }
                auto& interval = tree_intervals[static_cast<std::size_t>(split_dim)];
                const double split_value = interval.center();
                if (!(split_value > interval.lo && split_value < interval.hi)) {
                    ++cell_invalid;
                    return false;
                }
                const bool right_child = tree_seed[split_dim] > split_value;
                if (node > (std::numeric_limits<OracleNodeId>::max() - 2) / 2) {
                    ++cell_invalid;
                    return false;
                }
                node = static_cast<OracleNodeId>(2 * node + (right_child ? 2 : 1));
                if (right_child) {
                    interval.lo = split_value;
                } else {
                    interval.hi = split_value;
                }
                changed_dim = split_dim;
            }
            std::vector<Interval> native_intervals =
                oracle_->query_intervals_for_node(node, tree_intervals, point);
            if (native_intervals.size() != static_cast<std::size_t>(dims) ||
                !intervals_contain_point_strict_local(native_intervals, point, std::max(1e-12, tol))) {
                ++cell_invalid;
                return false;
            }

            const std::string key = cache_key(node, native_intervals);
            const auto cache_it = cell_cache.find(key);
            if (cache_it != cell_cache.end()) {
                ++cell_cache_hits;
                if (!cache_it->second.free) {
                    return false;
                }
                candidate = adaptive_make_box_from_intervals(cache_it->second.intervals,
                                                             cache_it->second.node,
                                                             next_internal_id--,
                                                             cache_it->second.safety_status,
                                                             cache_it->second.strict_audit_required);
                candidate.seed_config = point;
                return true;
            }

            ++cell_validations;
            const BoxValidation validation = oracle_->validate_node(node, native_intervals, changed_dim);
            const OracleValidationDetail detail = oracle_->last_validation_detail();
            NativeCellCacheEntry entry;
            entry.free = validation == BoxValidation::Free &&
                         detail.safety_status == BoxSafetyStatus::CertifiedFree &&
                         !detail.strict_audit_required;
            entry.node = node;
            entry.intervals = native_intervals;
            entry.safety_status = detail.safety_status;
            entry.strict_audit_required = detail.strict_audit_required;
            cell_cache.emplace(key, entry);
            if (!entry.free) {
                ++cell_not_free;
                return false;
            }
            ++cell_free;
            candidate = adaptive_make_box_from_intervals(native_intervals,
                                                         node,
                                                         next_internal_id--,
                                                         detail.safety_status,
                                                         detail.strict_audit_required);
            candidate.seed_config = point;
            return true;
        };
        auto classify_cell_at_point = [&](const Eigen::VectorXd& point,
                                          BoxNode& candidate) -> bool {
            for (int depth : candidate_depths) {
                if (classify_cell_at_point_at_depth(point, depth, candidate)) {
                    return true;
                }
            }
            return false;
        };

        auto boundary_seed_from_box = [&](const BoxNode& box,
                                          const Eigen::VectorXd& from,
                                          const Eigen::VectorXd& to) {
            if (box.n_dims() != from.size() || to.size() != from.size()) {
                return to;
            }
            const Eigen::VectorXd delta = to - from;
            const double norm = delta.norm();
            if (norm <= 1e-12) {
                return to;
            }
            double exit_param = 1.0;
            for (int dim = 0; dim < from.size(); ++dim) {
                const double d = delta[dim];
                if (std::abs(d) < 1e-15) {
                    continue;
                }
                const auto& interval = box.joint_intervals[static_cast<std::size_t>(dim)];
                const double boundary = d > 0.0 ? interval.hi : interval.lo;
                const double t = (boundary - from[dim]) / d;
                if (t > 1e-12 && t < exit_param) {
                    exit_param = t;
                }
            }
            const double face_epsilon = std::max(16.0 * std::max(0.0, tol), 1e-6);
            Eigen::VectorXd seed = from + std::clamp(exit_param, 0.0, 1.0) * delta +
                                   face_epsilon * (delta / norm);
            for (int dim = 0; dim < seed.size() &&
                              dim < static_cast<int>(domain.size()); ++dim) {
                seed[dim] = std::min(domain[static_cast<std::size_t>(dim)].hi,
                                     std::max(domain[static_cast<std::size_t>(dim)].lo,
                                              seed[dim]));
            }
            return seed;
        };

        std::function<bool(BoxNode&, const Eigen::VectorXd&, const Eigen::VectorXd&, int)> connect_to_point;
        connect_to_point = [&](BoxNode& current,
                               const Eigen::VectorXd& from,
                               const Eigen::VectorXd& to,
                               int depth) -> bool {
            if (current.contains(to, tol)) {
                return true;
            }
            if (target_box.contains(to, tol) && boxes_connected(current, target_box, tol)) {
                return true;
            }
            if (static_cast<int>(internal_boxes.size()) >= max_internal_boxes) {
                ++internal_cap_hits;
                return false;
            }
            const Eigen::VectorXd seed = boundary_seed_from_box(current, from, to);
            BoxNode candidate;
            if (classify_cell_at_point(seed, candidate)) {
                if (candidate.safety_status == BoxSafetyStatus::CertifiedFree &&
                    !candidate.strict_audit_required &&
                    boxes_connected(current, candidate, tol)) {
                    internal_boxes.push_back(candidate);
                    current = internal_boxes.back();
                    if (current.contains(to, tol) ||
                        (target_box.contains(to, tol) && boxes_connected(current, target_box, tol))) {
                        return true;
                    }
                    if ((seed - from).norm() <= 1e-12) {
                        return false;
                    }
                    return connect_to_point(current, seed, to, depth);
                }
                ++non_adjacent;
            }
            if (depth <= 0 || from.size() != to.size()) {
                return false;
            }
            ++recursion_splits;
            const Eigen::VectorXd midpoint = 0.5 * (from + to);
            if (!connect_to_point(current, from, midpoint, depth - 1)) {
                return false;
            }
            return connect_to_point(current, midpoint, to, depth - 1);
        };

        bool ok = true;
        BoxNode current = source_box;
        Eigen::VectorXd previous = source_box.center();
        for (const auto& waypoint : waypoint_path) {
            if (waypoint.size() != previous.size()) {
                ok = false;
                break;
            }
        }
        if (ok) {
            for (std::size_t index = 1; index < waypoint_path.size(); ++index) {
                if (!connect_to_point(current, previous, waypoint_path[index], max_recursion_depth)) {
                    ok = false;
                    break;
                }
                previous = waypoint_path[index];
            }
        }
        if (ok && !boxes_connected(current, target_box, tol)) {
            ok = connect_to_point(current, previous, target_box.center(), max_recursion_depth) &&
                 boxes_connected(current, target_box, tol);
        }

        diagnostics[prefix + ".portal_corridor_cell_native_min_depth"] =
            static_cast<double>(min_cell_depth);
        diagnostics[prefix + ".portal_corridor_cell_native_max_depth"] =
            static_cast<double>(max_cell_depth);
        diagnostics[prefix + ".portal_corridor_cell_native_depth_candidates"] =
            static_cast<double>(candidate_depths.size());
        diagnostics[prefix + ".portal_corridor_cell_native_validations"] +=
            static_cast<double>(cell_validations);
        diagnostics[prefix + ".portal_corridor_cell_native_free"] += static_cast<double>(cell_free);
        diagnostics[prefix + ".portal_corridor_cell_native_not_free"] +=
            static_cast<double>(cell_not_free);
        diagnostics[prefix + ".portal_corridor_cell_native_invalid"] +=
            static_cast<double>(cell_invalid);
        diagnostics[prefix + ".portal_corridor_cell_native_cache_hits"] +=
            static_cast<double>(cell_cache_hits);
        diagnostics[prefix + ".portal_corridor_cell_native_non_adjacent"] +=
            static_cast<double>(non_adjacent);
        diagnostics[prefix + ".portal_corridor_cell_native_recursion_splits"] +=
            static_cast<double>(recursion_splits);
        diagnostics[prefix + ".portal_corridor_cell_native_internal_cap_hit"] +=
            static_cast<double>(internal_cap_hits);
        diagnostics[prefix + ".portal_corridor_cell_native_internal_boxes"] +=
            static_cast<double>(internal_boxes.size());
        diagnostics[prefix + ".portal_corridor_internal_boxes"] +=
            static_cast<double>(internal_boxes.size());
        if (ok) {
            diagnostics[prefix + ".portal_corridor_cell_native_success"] += 1.0;
        } else {
            diagnostics[prefix + ".portal_corridor_cell_native_fail"] += 1.0;
        }
        return ok;
    };

    if (last_adaptive_partition_config_.hipac_portal_cell_native_validate) {
        chain_ok = build_cell_native_chain();
        if (chain_ok) {
            return append_portal_if_ready();
        }
        internal_boxes.clear();
    }

    const bool allow_ffb_resolver =
        !last_adaptive_partition_config_.hipac_portal_cell_native_validate ||
        (online_portal && last_adaptive_partition_config_.hipac_online_ffb_portal_fallback);
    if (!allow_ffb_resolver) {
        return anchors_added;
    }

    FindFreeBoxOptions ffb_options = config_.connector.pave.find_free_box;
    ffb_options.max_depth = std::max(1, std::min(requested_depth, config_.database.max_tree_depth));
    ffb_options.skip_existing_cover_check = true;
    ffb_options.reject_seed_collision = false;
    ffb_options.deadline_ms =
        std::max(0.0, last_adaptive_partition_config_.hipac_portal_ffb_deadline_ms);
    const int max_ffb_calls = online_portal
        ? std::max(0, last_adaptive_partition_config_.hipac_online_max_ffb_calls_per_portal)
        : -1;
    int ffb_calls = 0;
    int ffb_success = 0;
    int ffb_fail = 0;
    int non_adjacent = 0;
    int non_certified = 0;
    int recursion_splits = 0;

    std::function<bool(BoxNode&, const Eigen::VectorXd&, const Eigen::VectorXd&, int)> connect_to_point_ffb;
    connect_to_point_ffb = [&](BoxNode& current,
                               const Eigen::VectorXd& from,
                               const Eigen::VectorXd& to,
                               int depth) -> bool {
        if (current.contains(to, tol)) {
            return true;
        }
        if (target_box.contains(to, tol) && boxes_connected(current, target_box, tol)) {
            return true;
        }
        if (static_cast<int>(internal_boxes.size()) >= max_internal_boxes) {
            diagnostics[prefix + ".portal_corridor_internal_cap_hit"] += 1.0;
            return false;
        }
        if (max_ffb_calls >= 0 && ffb_calls >= max_ffb_calls) {
            diagnostics[prefix + ".portal_corridor_ffb_cap_hit"] += 1.0;
            return false;
        }
        ++ffb_calls;
        FindFreeBoxResult found = find_free_box_in_domain(to, domain, context, ffb_options);
        if (found.found) {
            BoxNode candidate = adaptive_make_box_from_intervals(found.intervals,
                                                                 found.node,
                                                                 next_internal_id--,
                                                                 BoxSafetyStatus::CertifiedFree,
                                                                 false);
            candidate.seed_config = to;
            if (candidate.safety_status != BoxSafetyStatus::CertifiedFree ||
                candidate.strict_audit_required) {
                ++non_certified;
            } else if (boxes_connected(current, candidate, tol)) {
                ++ffb_success;
                internal_boxes.push_back(candidate);
                current = internal_boxes.back();
                return true;
            } else {
                ++non_adjacent;
            }
        } else {
            ++ffb_fail;
        }
        if (depth <= 0 || from.size() != to.size()) {
            return false;
        }
        ++recursion_splits;
        const Eigen::VectorXd midpoint = 0.5 * (from + to);
        if (!connect_to_point_ffb(current, from, midpoint, depth - 1)) {
            return false;
        }
        return connect_to_point_ffb(current, midpoint, to, depth - 1);
    };

    chain_ok = true;
    {
        BoxNode current = source_box;
        Eigen::VectorXd previous = source_box.center();
        for (const auto& waypoint : waypoint_path) {
            if (waypoint.size() != previous.size()) {
                chain_ok = false;
                break;
            }
        }
        if (chain_ok) {
            for (std::size_t index = 1; index < waypoint_path.size(); ++index) {
                if (!connect_to_point_ffb(current, previous, waypoint_path[index], max_recursion_depth)) {
                    chain_ok = false;
                    break;
                }
                previous = waypoint_path[index];
            }
        }
        if (chain_ok && !boxes_connected(current, target_box, tol)) {
            chain_ok = connect_to_point_ffb(current, previous, target_box.center(), max_recursion_depth) &&
                       boxes_connected(current, target_box, tol);
        }
    }

    diagnostics[prefix + ".portal_corridor_ffb_calls"] += static_cast<double>(ffb_calls);
    diagnostics[prefix + ".portal_corridor_ffb_success"] += static_cast<double>(ffb_success);
    diagnostics[prefix + ".portal_corridor_ffb_fail"] += static_cast<double>(ffb_fail);
    diagnostics[prefix + ".portal_corridor_non_adjacent"] += static_cast<double>(non_adjacent);
    diagnostics[prefix + ".portal_corridor_non_certified"] += static_cast<double>(non_certified);
    diagnostics[prefix + ".portal_corridor_recursion_splits"] += static_cast<double>(recursion_splits);
    diagnostics[prefix + ".portal_corridor_internal_boxes"] +=
        static_cast<double>(internal_boxes.size());

    for (const auto& [key, value] : context.diagnostics().snapshot()) {
        diagnostics[prefix + ".portal_corridor_context." + key] += value;
    }

    return append_portal_if_ready();
}

int RBFPlanningForest::add_segment_edge_partition_first(
    int source_box_id,
    int target_box_id,
    std::vector<Eigen::VectorXd> waypoints,
    SegmentEdgeType type,
    int segment_resolution,
    SegmentEdgeValidation validation,
    bool strict_audit_required,
    int query_index,
    BuildProfile* profile,
    const char* diagnostic_prefix) {
    const bool use_partition_overlay = partition_native_mode();
    BuildProfile* out_profile = profile != nullptr ? profile : &last_build_;
    const std::string prefix =
        (diagnostic_prefix != nullptr && diagnostic_prefix[0] != '\0')
            ? std::string(diagnostic_prefix)
            : std::string("partition_segment_edge");
    auto& diagnostics = out_profile->diagnostics;

    const bool path_is_rrt_bridge_like =
        type == SegmentEdgeType::RRTConnector || waypoints.size() > 2U;
    std::vector<Eigen::VectorXd> partial_obb_centers;
    std::vector<Eigen::MatrixXd> partial_obb_generators;
    double partial_obb_covered_length = 0.0;
    std::string partial_obb_diag;
    const bool eligible_for_obb_cover =
        (last_adaptive_partition_config_.segment_edge_obb_cover ||
         (path_is_rrt_bridge_like && last_adaptive_partition_config_.rrt_bridge_obb_cover)) &&
        validation == SegmentEdgeValidation::CollisionChecked &&
        counts_as_segment_edge(type) &&
        waypoints.size() >= 2U &&
        oracle_ != nullptr;
    const bool strict_obb_bridge_cover =
        eligible_for_obb_cover && last_adaptive_partition_config_.strict_obb_bridge_cover;
    const bool obb_metadata_only =
        eligible_for_obb_cover &&
        !strict_obb_bridge_cover &&
        env_int_or_default("RBF_OBB_METADATA_ONLY", 0) != 0;
    const bool obb_metadata_require_cover =
        obb_metadata_only &&
        env_int_or_default("RBF_OBB_METADATA_ONLY_REQUIRE_COVER", 0) != 0;
    if (eligible_for_obb_cover) {
        const bool greedy_bridge_cover =
            path_is_rrt_bridge_like &&
            (last_adaptive_partition_config_.rrt_bridge_obb_cover ||
             last_adaptive_partition_config_.segment_edge_obb_cover);
        const std::string obb_diag = greedy_bridge_cover
            ? std::string("rrt_bridge_obb_cover")
            : std::string("segment_obb_cover");
        diagnostics[prefix + "." + obb_diag + "_attempts"] += 1.0;
        const BoxNode* source_ptr = find_box_by_id(boxes_, source_box_id);
        const BoxNode* target_ptr = find_box_by_id(boxes_, target_box_id);
        if (source_ptr != nullptr && target_ptr != nullptr) {
            std::optional<CollisionChecker> obb_audit_checker;
            auto audit_obb_edge_path = [&](const std::vector<Eigen::VectorXd>& candidate_path,
                                           const char* label) -> bool {
                diagnostics[prefix + "." + obb_diag + "_centerline_audit_attempts"] += 1.0;
                if (candidate_path.size() < 2U) {
                    diagnostics[prefix + "." + obb_diag + "_centerline_audit_empty"] += 1.0;
                    return false;
                }
                const auto audit_t0 = std::chrono::steady_clock::now();
                if (!obb_audit_checker.has_value()) {
                    obb_audit_checker.emplace(make_audit_checker(audit_robot_, scene_, config_.query));
                }
                const PathAuditCheck centerline_audit =
                    audit_waypoint_path(candidate_path,
                                        *obb_audit_checker,
                                        config_.query.audit_resolution,
                                        config_.query.audit_segment_step);
                diagnostics[prefix + "." + obb_diag + "_centerline_audit_ms"] +=
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - audit_t0).count();
                if (!centerline_audit.passed) {
                    diagnostics[prefix + "." + obb_diag + "_centerline_audit_rejects"] += 1.0;
                    diagnostics[prefix + "." + obb_diag + "_centerline_audit_failed_segment"] =
                        static_cast<double>(centerline_audit.failed_segment_index);
                    if (label != nullptr && label[0] != '\0') {
                        diagnostics[prefix + "." + obb_diag + "_centerline_audit_reject_" +
                                    std::string(label)] += 1.0;
                    }
                    return false;
                }
                diagnostics[prefix + "." + obb_diag + "_centerline_audit_pass"] += 1.0;
                return true;
            };
            auto obb_t0 = std::chrono::steady_clock::now();
            const double obb_safety_epsilon =
                std::max(0.0, last_adaptive_partition_config_.segment_edge_obb_safety_epsilon);
            std::vector<Eigen::VectorXd> obb_centerline;
            ObbPathCoverResult cover = cover_segment_or_bridge_path_with_obbs(
                robot_,
                scene_,
                oracle_->planning_intervals(),
                waypoints,
                greedy_bridge_cover,
                last_adaptive_partition_config_.segment_edge_obb_split_depth,
                last_adaptive_partition_config_.obb_max_window_segments,
                last_adaptive_partition_config_.segment_edge_obb_lateral_radius,
                last_adaptive_partition_config_.segment_edge_obb_longitudinal_margin,
                obb_safety_epsilon,
                last_adaptive_partition_config_.segment_edge_obb_grow_iterations,
                last_adaptive_partition_config_.segment_edge_obb_binary_iterations,
                last_adaptive_partition_config_.obb_max_validations_per_window,
                obb_centerline);
            const ObbPortalValidationStats& obb_stats = cover.stats;
            diagnostics[prefix + "." + obb_diag + "_windows_attempted"] +=
                static_cast<double>(cover.windows_attempted);
            diagnostics[prefix + "." + obb_diag + "_windows_success"] +=
                static_cast<double>(cover.windows_success);
            diagnostics[prefix + "." + obb_diag + "_regions"] +=
                static_cast<double>(cover.regions.size());
            diagnostics[prefix + "." + obb_diag + "_recursive_splits"] +=
                static_cast<double>(cover.recursive_splits);
            diagnostics[prefix + "." + obb_diag + "_failed_leaf_windows"] +=
                static_cast<double>(cover.failed_leaf_windows);
            diagnostics[prefix + "." + obb_diag + "_failed_leaf_length_sum"] +=
                cover.failed_leaf_length_sum;
            diagnostics[prefix + "." + obb_diag + "_failed_leaf_length_max"] =
                std::max(diagnostics[prefix + "." + obb_diag + "_failed_leaf_length_max"],
                         cover.failed_leaf_length_max);
            if ((cover.has_first_failed_leaf || cover.failed_leaf_windows > 0) &&
                diagnostics[prefix + "." + obb_diag + "_first_failed_leaf_recorded"] <= 0.0) {
                const Eigen::VectorXd& failed_a =
                    cover.has_first_failed_leaf ? cover.first_failed_leaf_a : waypoints.front();
                const Eigen::VectorXd& failed_b =
                    cover.has_first_failed_leaf ? cover.first_failed_leaf_b : waypoints.back();
                diagnostics[prefix + "." + obb_diag + "_first_failed_leaf_recorded"] = 1.0;
                diagnostics[prefix + "." + obb_diag + "_first_failed_leaf_length"] =
                    (failed_b - failed_a).norm();
                diagnostics[prefix + "." + obb_diag + "_first_failed_leaf_exact"] =
                    cover.has_first_failed_leaf ? 1.0 : 0.0;
                const int dims = static_cast<int>(failed_a.size());
                diagnostics[prefix + "." + obb_diag + "_first_failed_leaf_dims"] =
                    static_cast<double>(dims);
                for (int dim = 0; dim < dims; ++dim) {
                    diagnostics[prefix + "." + obb_diag + "_first_failed_leaf_a_" + std::to_string(dim)] =
                        failed_a[dim];
                    diagnostics[prefix + "." + obb_diag + "_first_failed_leaf_b_" + std::to_string(dim)] =
                        failed_b[dim];
                }
            }
            diagnostics[prefix + "." + obb_diag + "_candidates"] +=
                static_cast<double>(obb_stats.candidates);
            diagnostics[prefix + "." + obb_diag + "_validations"] +=
                static_cast<double>(obb_stats.validations);
            diagnostics[prefix + "." + obb_diag + "_valid_candidates"] +=
                static_cast<double>(obb_stats.valid_candidates);
            diagnostics[prefix + "." + obb_diag + "_grow_attempts"] +=
                static_cast<double>(obb_stats.grow_attempts);
            diagnostics[prefix + "." + obb_diag + "_joint_limit_rejects"] +=
                static_cast<double>(obb_stats.joint_limit_rejects);
            diagnostics[prefix + "." + obb_diag + "_gjk_tests"] +=
                static_cast<double>(obb_stats.gjk_tests);
            diagnostics[prefix + "." + obb_diag + "_maybe_pairs"] +=
                static_cast<double>(obb_stats.maybe_pairs);
            diagnostics[prefix + "." + obb_diag + "_sampled_support_attempts"] +=
                static_cast<double>(obb_stats.sampled_support_attempts);
            diagnostics[prefix + "." + obb_diag + "_sampled_support_success"] +=
                static_cast<double>(obb_stats.sampled_support_success);
            diagnostics[prefix + "." + obb_diag + "_sampled_support_fail"] +=
                static_cast<double>(obb_stats.sampled_support_fail);
            diagnostics[prefix + "." + obb_diag + "_sampled_support_samples"] +=
                static_cast<double>(obb_stats.sampled_support_samples);
            diagnostics[prefix + "." + obb_diag + "_sampled_support_error_radius"] =
                std::max(diagnostics[prefix + "." + obb_diag + "_sampled_support_error_radius"],
                         obb_stats.sampled_support_error_radius);
            diagnostics[prefix + "." + obb_diag + "_clearance_support_attempts"] +=
                static_cast<double>(obb_stats.clearance_support_attempts);
            diagnostics[prefix + "." + obb_diag + "_clearance_support_success"] +=
                static_cast<double>(obb_stats.clearance_support_success);
            diagnostics[prefix + "." + obb_diag + "_clearance_support_fail"] +=
                static_cast<double>(obb_stats.clearance_support_fail);
            diagnostics[prefix + "." + obb_diag + "_clearance_support_samples"] +=
                static_cast<double>(obb_stats.clearance_support_samples);
            diagnostics[prefix + "." + obb_diag + "_clearance_support_error_radius"] =
                std::max(diagnostics[prefix + "." + obb_diag + "_clearance_support_error_radius"],
                         obb_stats.clearance_support_error_radius);
            if (std::isfinite(obb_stats.clearance_support_min_margin)) {
                const std::string margin_key =
                    prefix + "." + obb_diag + "_clearance_support_min_margin";
                const auto margin_it = diagnostics.find(margin_key);
                diagnostics[margin_key] =
                    margin_it == diagnostics.end()
                        ? obb_stats.clearance_support_min_margin
                        : std::min(margin_it->second, obb_stats.clearance_support_min_margin);
            }
            diagnostics[prefix + "." + obb_diag + "_longitudinal_radius"] =
                std::max(diagnostics[prefix + "." + obb_diag + "_longitudinal_radius"],
                         obb_stats.longitudinal_radius);
            diagnostics[prefix + "." + obb_diag + "_lateral_radius"] =
                std::max(diagnostics[prefix + "." + obb_diag + "_lateral_radius"],
                         obb_stats.lateral_radius);
            diagnostics[prefix + "." + obb_diag + "_region_volume_sum"] +=
                obb_stats.region_volume_sum;
            diagnostics[prefix + "." + obb_diag + "_region_volume_max"] =
                std::max(diagnostics[prefix + "." + obb_diag + "_region_volume_max"],
                         obb_stats.region_volume_max);
            diagnostics[prefix + "." + obb_diag + "_region_log_volume_sum"] +=
                obb_stats.region_log_volume_sum;
            diagnostics[prefix + "." + obb_diag + "_region_volume_count"] +=
                static_cast<double>(obb_stats.region_volume_count);
            diagnostics[prefix + "." + obb_diag + "_ms"] +=
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - obb_t0).count();
            auto try_clearance_retry_obb_edge = [&]() -> int {
                if (!greedy_bridge_cover || !strict_obb_bridge_cover || waypoints.size() < 2U) {
                    return -1;
                }
                const int retry_attempts =
                    std::max(0, env_int_or_default("RBF_OBB_CLEARANCE_RETRY_ATTEMPTS", 0));
                if (retry_attempts <= 0) {
                    return -1;
                }
                std::vector<double> clearances =
                    env_double_list_or_empty("RBF_OBB_CLEARANCE_RETRY_VALUES");
                if (clearances.empty()) {
                    const double fallback_clearance =
                        std::max(0.0, env_double_or_default("RBF_QUERY_BRIDGE_RRT_CLEARANCE", 0.0));
                    if (fallback_clearance > 0.0) {
                        clearances.push_back(fallback_clearance);
                    }
                }
                if (clearances.empty()) {
                    return -1;
                }
                CollisionChecker final_checker = make_audit_checker(audit_robot_, scene_, config_.query);
                RRTConnectConfig retry_config = config_.connector.rrt;
                retry_config.segment_resolution =
                    std::max(retry_config.segment_resolution, config_.query.audit_resolution);
                retry_config.segment_step = config_.query.audit_segment_step;
                retry_config.max_iters = std::max(
                    1,
                    env_int_or_default("RBF_OBB_CLEARANCE_RETRY_ITERS",
                                       std::max(1, retry_config.max_iters)));
                retry_config.timeout_ms = std::max(
                    0.0,
                    env_double_or_default("RBF_OBB_CLEARANCE_RETRY_TIMEOUT_MS",
                                          retry_config.timeout_ms));
                diagnostics[prefix + "." + obb_diag + "_clearance_retry_attempt_budget"] +=
                    static_cast<double>(retry_attempts);
                for (int attempt = 0; attempt < retry_attempts; ++attempt) {
                    const double clearance =
                        std::max(0.0, clearances[static_cast<std::size_t>(attempt) %
                                                  clearances.size()]);
                    if (!(clearance > 0.0)) {
                        continue;
                    }
                    diagnostics[prefix + "." + obb_diag + "_clearance_retry_attempts"] += 1.0;
                    Robot clearance_robot = make_sbf_clearance_robot(audit_robot_, clearance);
                    CollisionChecker clearance_checker(clearance_robot, scene_);
                    const int retry_seed = derived_planner_seed(
                        config_.grower.rng_seed,
                        kSeedQueryBridgeOffset,
                        attempt,
                        query_index < 0 ? 0 : query_index,
                        17017);
                    std::vector<Eigen::VectorXd> retry_path = rrt_connect(
                        waypoints.front(),
                        waypoints.back(),
                        clearance_checker,
                        clearance_robot,
                        retry_config,
                        retry_seed);
                    if (retry_path.empty()) {
                        diagnostics[prefix + "." + obb_diag + "_clearance_retry_no_path"] += 1.0;
                        continue;
                    }
                    const PathAuditCheck retry_audit =
                        audit_waypoint_path(retry_path,
                                            final_checker,
                                            config_.query.audit_resolution,
                                            config_.query.audit_segment_step);
                    if (!retry_audit.passed) {
                        diagnostics[prefix + "." + obb_diag + "_clearance_retry_audit_fail"] += 1.0;
                        continue;
                    }
                    std::vector<Eigen::VectorXd> retry_centerline;
                    ObbPathCoverResult retry_cover = cover_segment_or_bridge_path_with_obbs(
                        robot_,
                        scene_,
                        oracle_->planning_intervals(),
                        retry_path,
                        true,
                        last_adaptive_partition_config_.segment_edge_obb_split_depth,
                        last_adaptive_partition_config_.obb_max_window_segments,
                        last_adaptive_partition_config_.segment_edge_obb_lateral_radius,
                        last_adaptive_partition_config_.segment_edge_obb_longitudinal_margin,
                        obb_safety_epsilon,
                        last_adaptive_partition_config_.segment_edge_obb_grow_iterations,
                        last_adaptive_partition_config_.segment_edge_obb_binary_iterations,
                        last_adaptive_partition_config_.obb_max_validations_per_window,
                        retry_centerline);
                    obb_accumulate_stats(cover.stats, retry_cover.stats);
                    diagnostics[prefix + "." + obb_diag + "_clearance_retry_windows_attempted"] +=
                        static_cast<double>(retry_cover.windows_attempted);
                    diagnostics[prefix + "." + obb_diag + "_clearance_retry_windows_success"] +=
                        static_cast<double>(retry_cover.windows_success);
                    diagnostics[prefix + "." + obb_diag + "_clearance_retry_failed_leaf_windows"] +=
                        static_cast<double>(retry_cover.failed_leaf_windows);
                    diagnostics[prefix + "." + obb_diag + "_clearance_retry_failed_leaf_length_max"] =
                        std::max(diagnostics[prefix + "." + obb_diag +
                                             "_clearance_retry_failed_leaf_length_max"],
                                 retry_cover.failed_leaf_length_max);
                    if (!retry_cover.success || retry_cover.regions.empty()) {
                        diagnostics[prefix + "." + obb_diag + "_clearance_retry_cover_fail"] += 1.0;
                        continue;
                    }
                    std::vector<Eigen::VectorXd> obb_centers;
                    std::vector<Eigen::MatrixXd> obb_generators;
                    obb_centers.reserve(retry_cover.regions.size());
                    obb_generators.reserve(retry_cover.regions.size());
                    for (const auto& region : retry_cover.regions) {
                        obb_centers.push_back(region.center);
                        obb_generators.push_back(region.generators);
                    }
                    const std::vector<Eigen::VectorXd>& retry_edge_path =
                        retry_centerline.empty() ? retry_path : retry_centerline;
                    if (!audit_obb_edge_path(retry_edge_path, "clearance_retry")) {
                        diagnostics[prefix + "." + obb_diag +
                                    "_clearance_retry_centerline_audit_fail"] += 1.0;
                        continue;
                    }
                    const int retry_edge_id = append_certified_portal_corridor_edge(
                        segment_edges_,
                        *source_ptr,
                        *target_ptr,
                        retry_edge_path,
                        SegmentEdgeValidation::ConservativeObbZonotope,
                        -1,
                        query_index,
                        nullptr,
                        nullptr,
                        SegmentEdgeType::RRTBridgeOBBCorridor,
                        &obb_centers,
                        &obb_generators);
                    if (retry_edge_id >= 0) {
                        diagnostics[prefix + "." + obb_diag + "_clearance_retry_success"] += 1.0;
                        diagnostics[prefix + "." + obb_diag + "_success"] += 1.0;
                        diagnostics[prefix + "." + obb_diag + "_replaced_segments"] +=
                            static_cast<double>(std::max<std::size_t>(1U, retry_path.size() - 1U));
                        if (use_partition_overlay) {
                            sync_adaptive_partition_segment_edges(out_profile, prefix.c_str());
                        } else {
                            auto append_unique = [&](int a, int b) {
                                auto& neighbors = adjacency_[a];
                                if (std::find(neighbors.begin(), neighbors.end(), b) == neighbors.end()) {
                                    neighbors.push_back(b);
                                }
                            };
                            append_unique(source_box_id, target_box_id);
                            append_unique(target_box_id, source_box_id);
                            invalidate_query_cache();
                        }
                        return retry_edge_id;
                    }
                    diagnostics[prefix + "." + obb_diag + "_clearance_retry_edge_fail"] += 1.0;
                }
                return -1;
            };
            if (cover.success && !cover.regions.empty()) {
                std::vector<Eigen::VectorXd> obb_centers;
                std::vector<Eigen::MatrixXd> obb_generators;
                obb_centers.reserve(cover.regions.size());
                obb_generators.reserve(cover.regions.size());
                for (const auto& region : cover.regions) {
                    obb_centers.push_back(region.center);
                    obb_generators.push_back(region.generators);
                }
                const SegmentEdgeType obb_edge_type = greedy_bridge_cover
                    ? SegmentEdgeType::RRTBridgeOBBCorridor
                    : SegmentEdgeType::SegmentOBBCorridor;
                if (obb_metadata_only) {
                    partial_obb_diag = obb_diag;
                    partial_obb_covered_length = cover.covered_length;
                    partial_obb_centers = std::move(obb_centers);
                    partial_obb_generators = std::move(obb_generators);
                    diagnostics[prefix + "." + obb_diag + "_success"] += 1.0;
                    diagnostics[prefix + "." + obb_diag + "_metadata_only"] += 1.0;
                    diagnostics[prefix + "." + obb_diag + "_metadata_only_segments"] +=
                        static_cast<double>(std::max<std::size_t>(1U, waypoints.size() - 1U));
                } else {
                const std::vector<Eigen::VectorXd>& obb_edge_path =
                    obb_centerline.empty() ? waypoints : obb_centerline;
                if (!audit_obb_edge_path(obb_edge_path, "primary")) {
                    diagnostics[prefix + "." + obb_diag + "_centerline_audit_fail"] += 1.0;
                    if (strict_obb_bridge_cover) {
                        const int retry_edge_id = try_clearance_retry_obb_edge();
                        if (retry_edge_id >= 0) {
                            return retry_edge_id;
                        }
                        diagnostics[prefix + "." + obb_diag + "_strict_reject"] += 1.0;
                        return -1;
                    }
                } else {
                const int obb_edge_id = append_certified_portal_corridor_edge(
                    segment_edges_,
                    *source_ptr,
                    *target_ptr,
                    obb_edge_path,
                    SegmentEdgeValidation::ConservativeObbZonotope,
                    -1,
                    query_index,
                    nullptr,
                    nullptr,
                    obb_edge_type,
                    &obb_centers,
                    &obb_generators);
                if (obb_edge_id >= 0) {
                    diagnostics[prefix + "." + obb_diag + "_success"] += 1.0;
                    diagnostics[prefix + "." + obb_diag + "_replaced_segments"] +=
                        static_cast<double>(std::max<std::size_t>(1U, waypoints.size() - 1U));
                    if (use_partition_overlay) {
                        sync_adaptive_partition_segment_edges(out_profile, prefix.c_str());
                    } else {
                        auto append_unique = [&](int a, int b) {
                            auto& neighbors = adjacency_[a];
                            if (std::find(neighbors.begin(), neighbors.end(), b) == neighbors.end()) {
                                neighbors.push_back(b);
                            }
                        };
                        append_unique(source_box_id, target_box_id);
                        append_unique(target_box_id, source_box_id);
                        invalidate_query_cache();
                    }
                    return obb_edge_id;
                }
                diagnostics[prefix + "." + obb_diag + "_edge_fail"] += 1.0;
                if (strict_obb_bridge_cover) {
                    const int retry_edge_id = try_clearance_retry_obb_edge();
                    if (retry_edge_id >= 0) {
                        return retry_edge_id;
                    }
                    diagnostics[prefix + "." + obb_diag + "_strict_reject"] += 1.0;
                    return -1;
                }
                }
                }
            } else {
                diagnostics[prefix + "." + obb_diag + "_fail"] += 1.0;
                if (obb_metadata_require_cover) {
                    diagnostics[prefix + "." + obb_diag +
                                "_metadata_require_cover_reject"] += 1.0;
                    return -1;
                }
                if (!cover.regions.empty() && cover.covered_length > 0.0) {
                    partial_obb_diag = obb_diag;
                    partial_obb_covered_length = cover.covered_length;
                    partial_obb_centers.reserve(cover.regions.size());
                    partial_obb_generators.reserve(cover.regions.size());
                    for (const auto& region : cover.regions) {
                        partial_obb_centers.push_back(region.center);
                        partial_obb_generators.push_back(region.generators);
                    }
                    diagnostics[prefix + "." + obb_diag + "_partial_edges"] += 1.0;
                    diagnostics[prefix + "." + obb_diag + "_partial_regions"] +=
                        static_cast<double>(partial_obb_centers.size());
                    diagnostics[prefix + "." + obb_diag + "_partial_covered_length"] +=
                        partial_obb_covered_length;
                }
                if (strict_obb_bridge_cover) {
                    const int retry_edge_id = try_clearance_retry_obb_edge();
                    if (retry_edge_id >= 0) {
                        return retry_edge_id;
                    }
                    diagnostics[prefix + "." + obb_diag + "_strict_reject"] += 1.0;
                    return -1;
                }
            }
        } else {
            diagnostics[prefix + "." + obb_diag + "_missing_box"] += 1.0;
            if (obb_metadata_require_cover) {
                diagnostics[prefix + "." + obb_diag +
                            "_metadata_require_cover_reject"] += 1.0;
                return -1;
            }
            if (strict_obb_bridge_cover) {
                diagnostics[prefix + "." + obb_diag + "_strict_reject"] += 1.0;
                return -1;
            }
        }
    }

    const int edge_id = use_partition_overlay
        ? append_segment_edge(segment_edges_,
                              source_box_id,
                              target_box_id,
                              std::move(waypoints),
                              type,
                              segment_resolution,
                              validation,
                              strict_audit_required,
                              query_index)
        : add_segment_edge(segment_edges_,
                           adjacency_,
                           source_box_id,
                           target_box_id,
                           std::move(waypoints),
                           type,
                           segment_resolution,
                           validation,
                           strict_audit_required,
                           query_index);
    if (edge_id < 0) {
        return -1;
    }
    if (!partial_obb_centers.empty() && partial_obb_centers.size() == partial_obb_generators.size()) {
        auto edge_it = std::find_if(segment_edges_.begin(),
                                    segment_edges_.end(),
                                    [&](const SegmentEdge& edge) {
                                        return edge.id == edge_id;
                                    });
        if (edge_it != segment_edges_.end()) {
            edge_it->obb_centers = std::move(partial_obb_centers);
            edge_it->obb_generators = std::move(partial_obb_generators);
            edge_it->obb_covered_length =
                std::min(edge_it->length, std::max(0.0, partial_obb_covered_length));
            if (!partial_obb_diag.empty()) {
                diagnostics[prefix + "." + partial_obb_diag + "_partial_committed"] += 1.0;
            }
        }
    }
    if (use_partition_overlay) {
        sync_adaptive_partition_segment_edges(out_profile, prefix.c_str());
    } else {
        invalidate_query_cache();
    }
    return edge_id;
}

}  // namespace rbf
