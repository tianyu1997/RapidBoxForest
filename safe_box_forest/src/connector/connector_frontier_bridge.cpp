#include "connector_frontier_bridge.h"

#include "connector_internal.h"

#include <SBF/box_graph.h>
#include <SBF/find_free_box.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace rbf {
namespace {

bool frontier_bridge_better(const FrontierBridgeCandidate& lhs,
                            const FrontierBridgeCandidate& rhs) {
    if (std::abs(lhs.gap_sq - rhs.gap_sq) > 1e-18) {
        return lhs.gap_sq < rhs.gap_sq;
    }
    if (std::abs(lhs.projected_gap_sq - rhs.projected_gap_sq) > 1e-18) {
        return lhs.projected_gap_sq < rhs.projected_gap_sq;
    }
    if (std::abs(lhs.face_score - rhs.face_score) > 1e-18) {
        return lhs.face_score < rhs.face_score;
    }
    if (lhs.source_box_id != rhs.source_box_id) {
        return lhs.source_box_id < rhs.source_box_id;
    }
    return lhs.target_box_id < rhs.target_box_id;
}

}  // namespace

bool select_frontier_bridge_candidate(const std::vector<BoxNode>& boxes,
                                      const AdjacencyGraph& graph,
                                      BoxOracle& oracle,
                                      const IslandConnectorConfig& config,
                                      std::unordered_map<int, double>& best_gap_by_root,
                                      std::unordered_map<int, int>& stale_by_root,
                                      StageContext& context,
                                      FrontierBridgeCandidate& out) {
    const auto islands = find_islands(graph);
    if (islands.size() <= 1) {
        return false;
    }
    const auto map = make_box_map(boxes);
    const auto root = oracle.planning_intervals();
    const double epsilon = std::max(config.frontier_bridge_boundary_epsilon, 0.25 * config.pave.adjacency_tolerance);
    std::vector<FrontierBridgeCandidate> candidates;
    candidates.reserve(static_cast<std::size_t>(std::max(1, config.frontier_bridge_candidate_limit)));

    auto add_candidate = [&](const BoxNode& source, const BoxNode& target, double gap_sq) {
        if (source.n_dims() != target.n_dims()) {
            return;
        }
        const Eigen::VectorXd target_center = target.center();
        for (int dim = 0; dim < source.n_dims(); ++dim) {
            for (int side = 0; side <= 1; ++side) {
                const auto& source_interval = source.joint_intervals[static_cast<std::size_t>(dim)];
                const auto& target_interval = target.joint_intervals[static_cast<std::size_t>(dim)];
                if (source_interval.hi < target_interval.lo && side != 1) {
                    continue;
                }
                if (target_interval.hi < source_interval.lo && side != 0) {
                    continue;
                }
                const double score = face_seed_score(source, root, target_center, dim, side, epsilon);
                if (!std::isfinite(score)) {
                    continue;
                }
                Eigen::VectorXd seed = make_face_seed(source, root, target_center, dim, side, epsilon);
                if (point_covered_by_existing_box(boxes, seed)) {
                    continue;
                }
                const double projected_gap_sq = point_box_gap_squared(seed, target);
                if (!(projected_gap_sq < gap_sq - 1e-18)) {
                    continue;
                }
                candidates.push_back({source.id,
                                      target.id,
                                      source.root_id,
                                      std::move(seed),
                                      gap_sq,
                                      projected_gap_sq,
                                      score,
                                      0});
            }
        }
    };

    for (std::size_t lhs_island = 0; lhs_island < islands.size(); ++lhs_island) {
        for (std::size_t rhs_island = lhs_island + 1; rhs_island < islands.size(); ++rhs_island) {
            const auto pair_tasks = broadphase_bridge_pairs(map,
                                                            islands[lhs_island],
                                                            islands[rhs_island],
                                                            std::max(1, config.frontier_bridge_candidate_limit * 4),
                                                            std::max(4, config.frontier_bridge_candidate_limit / 4));
            context.diagnostics().add_counter("connector.frontier_bridge_broadphase_pairs", static_cast<double>(pair_tasks.size()));
            for (const auto& task : pair_tasks) {
                const BoxNode& lhs = *map.at(task.source_box_id);
                const BoxNode& rhs = *map.at(task.target_box_id);
                const double gap_sq = box_gap_squared(lhs, rhs);
                if (!std::isfinite(gap_sq) || gap_sq <= 1e-24) {
                    continue;
                }
                add_candidate(lhs, rhs, gap_sq);
                add_candidate(rhs, lhs, gap_sq);
            }
        }
    }

    if (candidates.empty()) {
        context.diagnostics().add_counter("connector.frontier_bridge_no_expandable_face");
        return false;
    }
    std::sort(candidates.begin(), candidates.end(), frontier_bridge_better);
    if (static_cast<int>(candidates.size()) > std::max(1, config.frontier_bridge_candidate_limit)) {
        candidates.resize(static_cast<std::size_t>(std::max(1, config.frontier_bridge_candidate_limit)));
    }

    FrontierBridgeCandidate chosen = candidates.front();
    const double nearest_gap = std::sqrt(std::max(0.0, chosen.gap_sq));
    const int root_key = chosen.source_root_id;
    const auto best_it = best_gap_by_root.find(root_key);
    bool improved = best_it == best_gap_by_root.end();
    if (!improved) {
        const double min_delta = std::max(1e-9, 1e-3 * std::max(1.0, best_it->second));
        improved = nearest_gap < best_it->second - min_delta;
    }
    int stale_count = 0;
    if (improved) {
        best_gap_by_root[root_key] = nearest_gap;
        stale_by_root[root_key] = 0;
    } else {
        stale_count = ++stale_by_root[root_key];
        set_max_diagnostic(context, "connector.frontier_bridge_gap_stall_count_max", static_cast<double>(stale_count));
    }

    if (stale_count > 0 && candidates.size() > 1) {
        chosen = candidates[static_cast<std::size_t>(stale_count) % candidates.size()];
        context.diagnostics().add_counter("connector.frontier_bridge_face_rotation_tasks");
    }
    if (config.frontier_bridge_adaptive_ffb &&
        stale_count >= std::max(1, config.frontier_bridge_gap_stall_iterations)) {
        const int stall_window = std::max(1, config.frontier_bridge_gap_stall_iterations);
        const int multiplier = 1 + (stale_count - stall_window) / stall_window;
        int depth = config.pave.find_free_box.max_depth + std::max(1, config.frontier_bridge_ffb_depth_increment) * multiplier;
        if (config.frontier_bridge_ffb_max_depth > 0) {
            depth = std::min(depth, config.frontier_bridge_ffb_max_depth);
        }
        if (depth > config.pave.find_free_box.max_depth) {
            chosen.ffb_depth_override = depth;
            set_max_diagnostic(context, "connector.frontier_bridge_adaptive_ffb_depth_max", static_cast<double>(depth));
        }
    }

    context.diagnostics().add_counter("connector.frontier_bridge_face_candidates", static_cast<double>(candidates.size()));
    context.diagnostics().set_value("connector.frontier_bridge_gap_latest", nearest_gap);
    set_max_diagnostic(context, "connector.frontier_bridge_gap_max", nearest_gap);
    out = std::move(chosen);
    return true;
}

bool add_frontier_bridge_box(const FrontierBridgeCandidate& candidate,
                             std::vector<BoxNode>& boxes,
                             BoxOracle& oracle,
                             AdjacencyGraph& graph,
                             int& next_box_id,
                             const IslandConnectorConfig& config,
                             StageContext& context) {
    auto map = make_box_map(boxes);
    const auto source_it = map.find(candidate.source_box_id);
    if (source_it == map.end()) {
        return false;
    }
    if (point_covered_by_existing_box(boxes, candidate.seed)) {
        context.diagnostics().add_counter("connector.frontier_bridge_seed_already_covered");
        return false;
    }
    FindFreeBoxOptions options = config.pave.find_free_box;
    if (candidate.ffb_depth_override > options.max_depth) {
        options.max_depth = candidate.ffb_depth_override;
        context.diagnostics().add_counter("connector.frontier_bridge_adaptive_ffb_tasks");
    }
    FindFreeBoxService ffb(oracle);
    auto result = ffb.find(candidate.seed, context, options);
    if (!result.found) {
        context.diagnostics().add_counter("connector.frontier_bridge_ffb_failures");
        context.diagnostics().add_counter("connector.frontier_bridge_ffb_fail_code." + std::to_string(result.fail_code));
        if (result.hit_unknown_depth_cap) {
            context.diagnostics().add_counter("connector.frontier_bridge_ffb_unknown_depth_cap");
        }
        if (result.hit_reserved_depth_cap) {
            context.diagnostics().add_counter("connector.frontier_bridge_ffb_reserved_depth_cap");
        }
        return false;
    }
    if (!intervals_contain_point(result.intervals, candidate.seed, config.pave.adjacency_tolerance)) {
        context.diagnostics().add_counter("connector.frontier_bridge_seed_miss");
        return false;
    }
    if (!allow_connector_box_commit(oracle, result, config.pave.commit_policy, context)) {
        context.diagnostics().add_counter("connector.frontier_bridge_commit_rejected");
        return false;
    }

    BoxNode box;
    box.id = next_box_id++;
    box.joint_intervals = std::move(result.intervals);
    box.seed_config = candidate.seed;
    box.tree_id = result.node;
    box.parent_box_id = candidate.source_box_id;
    box.root_id = source_it->second->root_id;
    box.safety_status = result.validation_detail.safety_status;
    box.strict_audit_required = result.validation_detail.strict_audit_required;
    box.compute_volume();
    if (!boxes_connected(*source_it->second, box, config.pave.adjacency_tolerance)) {
        context.diagnostics().add_counter("connector.frontier_bridge_disconnected_child");
        return false;
    }
    oracle.reserve_node(result.node, box.id);
    const int new_box_id = box.id;
    const std::size_t new_box_index = boxes.size();
    boxes.push_back(std::move(box));
    graph[new_box_id] = {};
    int local_edges = 0;
    for (std::size_t index = 0; index < new_box_index; ++index) {
        if (boxes_connected(boxes[index], boxes.back(), config.pave.adjacency_tolerance)) {
            const std::size_t before = graph[new_box_id].size();
            append_graph_edge_unique(graph, boxes[index].id, new_box_id);
            if (graph[new_box_id].size() > before) {
                local_edges += 1;
            }
        }
    }
    context.diagnostics().add_counter("connector.frontier_bridge_incremental_adjacency_checks",
                                      static_cast<double>(new_box_index));
    context.diagnostics().add_counter("connector.frontier_bridge_incremental_edges",
                                      static_cast<double>(local_edges));
    context.diagnostics().add_counter("connector.frontier_bridge_successes");
    return true;
}

bool try_point_validated_gap_edge(const std::vector<BoxNode>& boxes,
                                  AdjacencyGraph& graph,
                                  SegmentEdgeList* segment_edges,
                                  const CollisionChecker& checker,
                                  const IslandConnectorConfig& config,
                                  StageContext& context) {
    if (config.point_validated_gap_tolerance <= 0.0) {
        return false;
    }
    auto islands = find_islands(graph);
    if (islands.size() <= 1) {
        return false;
    }
    std::sort(islands.begin(), islands.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.size() > rhs.size();
    });
    const auto map = make_box_map(boxes);
    const auto& main_island = islands.front();
    const auto& target_island = islands.back();
    struct GapCandidate {
        int source = -1;
        int target = -1;
        double gap = 0.0;
        double distance = 0.0;
    };
    std::vector<GapCandidate> candidates;
    const auto pair_tasks = interval_gap_broadphase_pairs(map,
                                                          main_island,
                                                          target_island,
                                                          config.point_validated_gap_tolerance);
    context.diagnostics().add_counter("connector.point_gap_broadphase_pairs", static_cast<double>(pair_tasks.size()));
    for (const auto& task : pair_tasks) {
        const BoxNode& source = *map.at(task.source_box_id);
        const BoxNode& target = *map.at(task.target_box_id);
        candidates.push_back({task.source_box_id, task.target_box_id, interval_max_gap(source, target), (source.center() - target.center()).norm()});
    }
    std::sort(candidates.begin(), candidates.end(), [](const GapCandidate& lhs, const GapCandidate& rhs) {
        if (lhs.gap == rhs.gap) {
            return lhs.distance < rhs.distance;
        }
        return lhs.gap < rhs.gap;
    });
    if (!candidates.empty()) {
        context.diagnostics().set_value("connector.closest_gap", candidates.front().gap);
        context.diagnostics().set_value("connector.closest_gap_distance", candidates.front().distance);
    }
    for (const auto& candidate : candidates) {
        if (candidate.gap > config.point_validated_gap_tolerance) {
            break;
        }
        const BoxNode& source = *map.at(candidate.source);
        const BoxNode& target = *map.at(candidate.target);
        if (check_segment_with_step(checker,
                                    source.center(),
                                    target.center(),
                                    config.point_validated_gap_resolution,
                                    config.point_validated_gap_step)) {
            context.diagnostics().add_counter("connector.point_gap_collision_rejects");
            continue;
        }
        if (config.segment_edges_fallback_only) {
            continue;
        }
        std::vector<Eigen::VectorXd> waypoints{source.center(), target.center()};
        if (segment_edges != nullptr && config.segment_edges_enabled && config.point_gap_segment_edges) {
            add_segment_edge(*segment_edges,
                             graph,
                             candidate.source,
                             candidate.target,
                             std::move(waypoints),
                             SegmentEdgeType::PointValidatedGap,
                             config.point_validated_gap_resolution,
                             SegmentEdgeValidation::CollisionChecked,
                             false);
            context.diagnostics().add_counter("connector.segment_edges_added");
            context.diagnostics().add_counter("connector.point_gap_segment_edges_added");
        } else {
            graph[candidate.source].push_back(candidate.target);
            graph[candidate.target].push_back(candidate.source);
        }
        context.diagnostics().add_counter("connector.point_validated_gap_edges");
        context.diagnostics().set_value("connector.point_validated_gap", candidate.gap);
        return true;
    }
    return false;
}

}  // namespace rbf
