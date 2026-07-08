#include "planning_forest_adaptive_frontier.h"

#include <SBF/planning_result.h>

#include <utility>

namespace rbf {

bool AdaptiveFrontierQueueLess::operator()(const AdaptiveFrontierItem& lhs,
                                           const AdaptiveFrontierItem& rhs) const {
    return lhs.score < rhs.score;
}

bool adaptive_intervals_overlap(const std::vector<Interval>& lhs,
                                const std::vector<Interval>& rhs,
                                double tolerance) {
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

void adaptive_push_frontier_item(
    AdaptiveFrontierItem item,
    const std::vector<Eigen::VectorXd>& free_probes,
    const std::vector<BoxNode>& scoring_boxes,
    const std::unordered_set<int>& main_ids,
    const AdaptiveLeafSweepConfig& config,
    double adjacency_tolerance,
    AdaptiveFrontierQueue& frontier,
    AdaptiveLeafSweepResult& result) {
    item.free_seed_hits = adaptive_count_seed_hits(item, free_probes);
    if (item.free_seed_hits > 0) {
        result.diagnostics["adaptive.frontier_seed_hit_pushes"] += 1.0;
        result.diagnostics["adaptive.frontier_seed_hits_total"] +=
            static_cast<double>(item.free_seed_hits);
    }
    item.score = adaptive_frontier_score(scoring_boxes,
                                         item,
                                         main_ids,
                                         config.overlap_depth_threshold,
                                         adjacency_tolerance);
    frontier.push(std::move(item));
}

void adaptive_seed_initial_frontier(
    const std::vector<BoxNode>& collision_boxes,
    const std::vector<Interval>& planning_domain,
    const std::vector<Eigen::VectorXd>& free_probes,
    const std::vector<BoxNode>& scoring_boxes,
    const std::unordered_set<int>& main_ids,
    const AdaptiveLeafSweepConfig& config,
    double adjacency_tolerance,
    AdaptiveFrontierQueue& frontier,
    AdaptiveLeafSweepResult& result) {
    for (const auto& collision_box : collision_boxes) {
        AdaptiveFrontierItem item;
        item.node = collision_box.tree_id >= 0 ? collision_box.tree_id : collision_box.id;
        item.intervals = collision_box.joint_intervals;
        item.changed_dim = -1;
        if (!planning_domain.empty() &&
            !adaptive_intervals_overlap(item.intervals, planning_domain, 0.0)) {
            result.diagnostics["adaptive.initial_frontier_outside_domain"] += 1.0;
            continue;
        }
        adaptive_push_frontier_item(std::move(item),
                                    free_probes,
                                    scoring_boxes,
                                    main_ids,
                                    config,
                                    adjacency_tolerance,
                                    frontier,
                                    result);
    }
}

bool adaptive_defer_frontier_item_if_needed(
    AdaptiveFrontierItem& item,
    int depth,
    int target_leaf_depth,
    const AdaptiveConnectivityDominance& connectivity,
    const AdaptiveLeafSweepConfig& config,
    std::vector<AdaptiveFrontierItem>& deferred,
    AdaptiveLeafSweepResult& result) {
    const bool high_overlap = adaptive_item_high_overlap(config, item, depth);
    const bool protected_by_seed = item.free_seed_hits > 0;
    const bool protected_by_adjacency =
        high_overlap && !protected_by_seed &&
        (connectivity.connector_candidate || connectivity.adjacent_main > 0);
    auto defer_item = [&](const char* diagnostic_key, bool record_seed_hit) {
        deferred.push_back(std::move(item));
        result.adaptive_deferred += 1;
        result.diagnostics[diagnostic_key] += 1.0;
        adaptive_add_depth_counter(result.diagnostics, "adaptive.depth.deferred.", depth);
        if (record_seed_hit && protected_by_seed) {
            result.diagnostics["adaptive.seed_hit_deferred"] += 1.0;
        }
    };

    if (depth >= target_leaf_depth) {
        defer_item("adaptive.deferred_depth_cap", true);
        return true;
    }
    if (high_overlap && !protected_by_seed && !protected_by_adjacency) {
        defer_item("adaptive.deferred_high_overlap", false);
        return true;
    }
    if (depth >= config.defer_min_depth &&
        !protected_by_seed &&
        connectivity.has_free_context &&
        connectivity.isolated) {
        defer_item("adaptive.deferred_connectivity_isolated", false);
        return true;
    }
    if (depth >= config.defer_min_depth &&
        !protected_by_seed &&
        connectivity.has_free_context &&
        connectivity.single_component &&
        connectivity.adjacent_main == 0) {
        defer_item("adaptive.deferred_connectivity_single_component", false);
        return true;
    }
    return false;
}

void adaptive_split_frontier_item_and_enqueue(
    AdaptiveFrontierItem item,
    int depth,
    const lect_database::SplitPolicyDescriptor& split_descriptor,
    const std::vector<Interval>& planning_domain,
    const std::vector<Eigen::VectorXd>& free_probes,
    const std::vector<BoxNode>& scoring_boxes,
    const std::unordered_set<int>& main_ids,
    const AdaptiveLeafSweepConfig& config,
    double adjacency_tolerance,
    std::vector<AdaptiveFrontierItem>& deferred,
    AdaptiveFrontierQueue& frontier,
    AdaptiveLeafSweepResult& result) {
    const bool item_has_seed_hit = item.free_seed_hits > 0;
    AdaptiveFrontierItem left;
    AdaptiveFrontierItem right;
    if (!adaptive_virtual_split_node(split_descriptor, item, left, right)) {
        deferred.push_back(std::move(item));
        result.adaptive_deferred += 1;
        result.diagnostics["adaptive.deferred_split_failure"] += 1.0;
        return;
    }
    result.adaptive_splits += 1;
    if (item_has_seed_hit) {
        result.diagnostics["adaptive.seed_hit_splits"] += 1.0;
    }
    adaptive_add_depth_counter(result.diagnostics, "adaptive.depth.split.", depth);
    auto enqueue_child = [&](AdaptiveFrontierItem child) {
        if (planning_domain.empty() ||
            adaptive_intervals_overlap(child.intervals, planning_domain, 0.0)) {
            adaptive_push_frontier_item(std::move(child),
                                        free_probes,
                                        scoring_boxes,
                                        main_ids,
                                        config,
                                        adjacency_tolerance,
                                        frontier,
                                        result);
        } else {
            result.diagnostics["adaptive.split_child_outside_domain"] += 1.0;
        }
    };
    enqueue_child(std::move(left));
    enqueue_child(std::move(right));
}

void adaptive_promote_deferred_by_seed(
    std::vector<AdaptiveFrontierItem>& deferred,
    const std::vector<Eigen::VectorXd>& free_probes,
    const std::vector<BoxNode>& scoring_boxes,
    const std::unordered_set<int>& main_ids,
    const AdaptiveLeafSweepConfig& config,
    double adjacency_tolerance,
    AdaptiveFrontierQueue& frontier,
    AdaptiveLeafSweepResult& result) {
    if (!config.seed_promote_uncovered || deferred.empty() || free_probes.empty()) {
        return;
    }
    std::vector<AdaptiveFrontierItem> keep;
    keep.reserve(deferred.size());
    for (auto& item : deferred) {
        const int hits = adaptive_count_seed_hits(item, free_probes);
        if (hits > 0) {
            item.free_seed_hits = hits;
            item.score = adaptive_frontier_score(scoring_boxes,
                                                 item,
                                                 main_ids,
                                                 config.overlap_depth_threshold,
                                                 adjacency_tolerance);
            frontier.push(std::move(item));
            result.adaptive_promoted += 1;
            result.diagnostics["adaptive.promoted_by_seed_probe"] += 1.0;
        } else {
            keep.push_back(std::move(item));
        }
    }
    deferred = std::move(keep);
}

}  // namespace rbf
