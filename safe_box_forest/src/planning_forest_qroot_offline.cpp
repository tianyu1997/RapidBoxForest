#include "planning_forest_qroot_growers.h"

#include "planning_forest_qroot_growers_internal.h"
#include "planning_forest_query_utils.h"

#include <algorithm>
#include <chrono>

namespace rbf {

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
        const int domain_idx = find_qroot_containing_domain_index(collision_domains,
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

} // namespace rbf
