#pragma once

#include <SBF/box_graph.h>
#include <SBF/safe_box_forest.h>

#include "planning_forest_qroot_helpers.h"

#include <Eigen/Core>

#include <functional>
#include <vector>

namespace rbf {

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
    double adjacency_tolerance);

QueryRootGrowResult run_query_root_box_grower(
    BoxOracle& oracle,
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
    double adjacency_tolerance);

} // namespace rbf
