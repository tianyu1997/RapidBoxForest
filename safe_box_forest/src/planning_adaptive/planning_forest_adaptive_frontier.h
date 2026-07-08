#pragma once

#include "planning_forest_adaptive_cover_utils.h"

#include <queue>
#include <unordered_set>
#include <vector>

namespace rbf {

struct AdaptiveFrontierQueueLess {
    bool operator()(const AdaptiveFrontierItem& lhs,
                    const AdaptiveFrontierItem& rhs) const;
};

using AdaptiveFrontierQueue = std::priority_queue<
    AdaptiveFrontierItem,
    std::vector<AdaptiveFrontierItem>,
    AdaptiveFrontierQueueLess>;

bool adaptive_intervals_overlap(const std::vector<Interval>& lhs,
                                const std::vector<Interval>& rhs,
                                double tolerance = 0.0);

void adaptive_push_frontier_item(
    AdaptiveFrontierItem item,
    const std::vector<Eigen::VectorXd>& free_probes,
    const std::vector<BoxNode>& scoring_boxes,
    const std::unordered_set<int>& main_ids,
    const AdaptiveLeafSweepConfig& config,
    double adjacency_tolerance,
    AdaptiveFrontierQueue& frontier,
    AdaptiveLeafSweepResult& result);

void adaptive_seed_initial_frontier(
    const std::vector<BoxNode>& collision_boxes,
    const std::vector<Interval>& planning_domain,
    const std::vector<Eigen::VectorXd>& free_probes,
    const std::vector<BoxNode>& scoring_boxes,
    const std::unordered_set<int>& main_ids,
    const AdaptiveLeafSweepConfig& config,
    double adjacency_tolerance,
    AdaptiveFrontierQueue& frontier,
    AdaptiveLeafSweepResult& result);

bool adaptive_defer_frontier_item_if_needed(
    AdaptiveFrontierItem& item,
    int depth,
    int target_leaf_depth,
    const AdaptiveConnectivityDominance& connectivity,
    const AdaptiveLeafSweepConfig& config,
    std::vector<AdaptiveFrontierItem>& deferred,
    AdaptiveLeafSweepResult& result);

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
    AdaptiveLeafSweepResult& result);

void adaptive_promote_deferred_by_seed(
    std::vector<AdaptiveFrontierItem>& deferred,
    const std::vector<Eigen::VectorXd>& free_probes,
    const std::vector<BoxNode>& scoring_boxes,
    const std::unordered_set<int>& main_ids,
    const AdaptiveLeafSweepConfig& config,
    double adjacency_tolerance,
    AdaptiveFrontierQueue& frontier,
    AdaptiveLeafSweepResult& result);

}  // namespace rbf
