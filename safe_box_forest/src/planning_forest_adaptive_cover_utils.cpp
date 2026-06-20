#include "planning_forest_adaptive_cover_utils.h"

#include "planning_forest_qroot_helpers.h"
#include "planning_forest_query_utils.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <random>
#include <sstream>

namespace rbf {

namespace {

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

}  // namespace

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

AdaptiveLeafBuildSetup make_adaptive_leaf_build_setup(
    const AdaptiveLeafSweepConfig& adaptive_config) {
    AdaptiveLeafBuildSetup setup;
    setup.adaptive_depth_enabled = adaptive_config.adaptive_depth_enabled;
    setup.adaptive_depth_min = std::max(
        adaptive_config.shallow_start_depth,
        adaptive_config.adaptive_depth_min > 0
            ? adaptive_config.adaptive_depth_min
            : adaptive_config.shallow_max_depth);
    setup.adaptive_depth_max = std::max(
        setup.adaptive_depth_min,
        adaptive_config.adaptive_depth_max > 0
            ? adaptive_config.adaptive_depth_max
            : adaptive_config.target_max_depth);
    setup.initial_leaf_depth = setup.adaptive_depth_enabled
        ? setup.adaptive_depth_min
        : adaptive_config.shallow_max_depth;
    setup.target_leaf_depth = setup.adaptive_depth_enabled
        ? setup.adaptive_depth_max
        : adaptive_config.target_max_depth;

    setup.leaf_config.obstacle_cluster_gap = adaptive_config.obstacle_cluster_gap;
    setup.leaf_config.n_threads = std::max(1, adaptive_config.threads);
    setup.leaf_config.validation_batch_size = std::max(1, adaptive_config.validation_batch_size);
    setup.leaf_config.timeout_ms = adaptive_config.time_budget_ms > 0.0
        ? adaptive_config.time_budget_ms
        : 0.0;
    setup.leaf_config.store_group_results = adaptive_config.store_group_results;
    setup.leaf_config.use_virtual_topology = adaptive_config.use_virtual_topology;
    setup.leaf_config.parallel_virtual_validation = adaptive_config.parallel_virtual_validation;
    setup.leaf_config.max_free_boxes = std::max(0, adaptive_config.max_free_boxes);
    setup.leaf_config.max_collision_boxes = std::max(0, adaptive_config.max_unresolved_domains);
    setup.leaf_config.collision_overlap_prune_min_depth = -1;
    setup.leaf_config.collision_overlap_prune_threshold = 0.0;
    setup.leaf_config.collision_overlap_prune_min_threshold = 0.0;
    setup.leaf_config.collision_overlap_prune_decay_per_depth = 0.0;
    setup.leaf_config.collision_overlap_prune_ratio_threshold = 0.0;

    setup.partition_config = adaptive_config;
    setup.partition_config.shallow_max_depth = setup.initial_leaf_depth;
    setup.partition_config.target_max_depth = setup.target_leaf_depth;
    if (setup.adaptive_depth_enabled || setup.partition_config.grid_target_depth <= 0) {
        setup.partition_config.grid_target_depth = setup.target_leaf_depth;
    }
    return setup;
}

void initialize_adaptive_leaf_sweep_result(AdaptiveLeafSweepResult& result,
                                           const AdaptiveLeafSweepConfig& config) {
    result.diagnostics["adaptive.offline_query_agnostic_build"] = 1.0;
    result.diagnostics["adaptive.qroot_pairs_total"] = 0.0;
    result.diagnostics["adaptive.qroot_uncovered_endpoints"] = 0.0;
    result.diagnostics["adaptive.fast_virtual_checkpoint_mode"] =
        config.fast_virtual_checkpoint_mode ? 1.0 : 0.0;
    result.diagnostics["adaptive.terminal_controller_enabled"] =
        config.fast_virtual_checkpoint_mode ? 0.0 : 1.0;
}

bool adaptive_depth_snapshot_readiness_met(const AdaptiveDepthSnapshot& snapshot,
                                           const AdaptiveLeafSweepConfig& config) {
    const int min_covered_probes = std::max(0, config.adaptive_depth_min_covered_probes);
    const int min_main_probes = std::max(0, config.adaptive_depth_min_main_probes);
    const int min_cells = std::max(0, config.adaptive_depth_min_cells);
    const int min_main_cells = std::max(0, config.adaptive_depth_min_main_cells);
    if (snapshot.cell_count <= 0 || snapshot.main_island_cell_count <= 0) {
        return false;
    }
    const bool probe_gate =
        snapshot.covered_count >= min_covered_probes &&
        snapshot.main_accessible_count >= min_main_probes &&
        (min_covered_probes <= 0 ||
         snapshot.main_connected_ratio >= config.adaptive_depth_min_main_ratio);
    const bool cell_gate =
        snapshot.cell_count >= min_cells &&
        snapshot.main_island_cell_count >= min_main_cells;
    return probe_gate &&
           cell_gate &&
           (config.adaptive_depth_max_online_cells <= 0 ||
            snapshot.cell_count <= config.adaptive_depth_max_online_cells);
}

AdaptiveDepthSnapshot adaptive_snapshot_from_fast_candidate(const AdaptiveLeafSweepResult& candidate,
                                                            int depth,
                                                            const AdaptiveLeafSweepConfig& config) {
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
    snapshot.readiness_met = adaptive_depth_snapshot_readiness_met(snapshot, config);
    return snapshot;
}

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

int adaptive_next_depth_checkpoint(int depth, int target_leaf_depth) {
    const int step = depth < 16 ? 1 : 2;
    return std::min(target_leaf_depth, depth + step);
}

void apply_adaptive_final_depth_snapshot(AdaptiveLeafSweepResult& result,
                                         const AdaptiveDepthSnapshot& snapshot) {
    result.selected_leaf_depth = snapshot.depth;
    result.adaptive_depth_readiness_met = snapshot.readiness_met;
    result.adaptive_depth_stop_reason = snapshot.stop_reason;
    result.seed_probe_box_covered = snapshot.covered_count;
    result.seed_probe_anchor_success = snapshot.anchor_success_count;
    result.seed_probe_main_accessible =
        snapshot.main_accessible_count + snapshot.anchor_to_main_count;
    result.p_box_covered = snapshot.p_box_covered;
    const double free_den = static_cast<double>(std::max(1, snapshot.free_probe_count));
    result.p_anchor_success = static_cast<double>(snapshot.anchor_success_count) / free_den;
    result.p_main_accessible = static_cast<double>(result.seed_probe_main_accessible) / free_den;
    result.p_anchor_to_main_uncovered = snapshot.p_anchor_to_main_uncovered;
}

double adaptive_active_overlap_depth_threshold(const AdaptiveLeafSweepConfig& config,
                                               int depth) {
    double threshold = config.overlap_depth_threshold;
    if (config.overlap_depth_decay_per_depth > 0.0 &&
        depth > config.defer_min_depth) {
        threshold =
            threshold /
            (1.0 + config.overlap_depth_decay_per_depth *
                       static_cast<double>(depth - config.defer_min_depth));
    }
    if (config.overlap_depth_min_threshold > 0.0) {
        threshold = std::max(config.overlap_depth_min_threshold, threshold);
    }
    return threshold;
}

bool adaptive_item_high_overlap(const AdaptiveLeafSweepConfig& config,
                                const AdaptiveFrontierItem& item,
                                int depth) {
    if (depth < config.defer_min_depth) {
        return false;
    }
    const double depth_threshold = adaptive_active_overlap_depth_threshold(config, depth);
    return (config.overlap_depth_threshold > 0.0 &&
            item.overlap_depth >= depth_threshold) ||
           (config.overlap_ratio_threshold > 0.0 &&
            item.overlap_ratio >= config.overlap_ratio_threshold);
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

std::vector<Eigen::VectorXd> adaptive_generate_initial_free_probes(
    DatabaseBoxOracle& oracle,
    const std::vector<Interval>& domain,
    const AdaptiveLeafSweepConfig& config,
    bool adaptive_depth_enabled,
    int& attempted) {
    attempted = 0;
    const int requested_probe_count = adaptive_depth_enabled
        ? std::max(0, config.adaptive_depth_probe_count)
        : std::max(0, config.seed_probe_count);
    const int requested_probe_seed = adaptive_depth_enabled
        ? config.adaptive_depth_probe_seed
        : config.seed_probe_rng_seed;
    std::vector<Eigen::VectorXd> free_probes =
        adaptive_generate_free_probes(oracle,
                                      domain,
                                      requested_probe_count,
                                      requested_probe_seed,
                                      attempted);
    if (!adaptive_depth_enabled ||
        config.adaptive_depth_min_free_probes <= 0 ||
        static_cast<int>(free_probes.size()) >= config.adaptive_depth_min_free_probes) {
        return free_probes;
    }
    const int supplement_limit = std::max(
        requested_probe_count,
        std::min(8192, std::max(requested_probe_count * 4,
                                config.adaptive_depth_min_free_probes * 64)));
    int supplement_seed_offset = 1;
    while (attempted < supplement_limit &&
           static_cast<int>(free_probes.size()) < config.adaptive_depth_min_free_probes) {
        const int batch = std::min(std::max(128, requested_probe_count),
                                   supplement_limit - attempted);
        int extra_attempted = 0;
        auto extra = adaptive_generate_free_probes(oracle,
                                                   domain,
                                                   batch,
                                                   requested_probe_seed + supplement_seed_offset,
                                                   extra_attempted);
        attempted += extra_attempted;
        free_probes.insert(free_probes.end(),
                           std::make_move_iterator(extra.begin()),
                           std::make_move_iterator(extra.end()));
        ++supplement_seed_offset;
        if (batch <= 0 || extra_attempted <= 0) {
            break;
        }
    }
    return free_probes;
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

ScopedAdaptiveFullOverlapStats::ScopedAdaptiveFullOverlapStats(DatabaseBoxOracle& oracle, bool enabled)
    : oracle_(oracle), previous_(oracle.validation_config().collect_full_overlap_stats) {
    if (enabled) {
        oracle_.set_collect_full_overlap_stats(true);
    }
}

ScopedAdaptiveFullOverlapStats::~ScopedAdaptiveFullOverlapStats() {
    oracle_.set_collect_full_overlap_stats(previous_);
}

}  // namespace rbf
