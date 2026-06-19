#include <SBF/connector.h>

#include "connector_birrt.h"
#include "connector_frontier_bridge.h"
#include "connector_internal.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace rbf {

IslandConnector::IslandConnector(BoxOracle& oracle,
                                 const Robot& robot,
                                 const CollisionChecker& checker,
                                 IslandConnectorConfig config)
    : oracle_(oracle), robot_(robot), checker_(checker), config_(std::move(config)) {}

IslandConnectorResult IslandConnector::connect_all(std::vector<BoxNode>& boxes,
                                                   AdjacencyGraph& graph,
                                                   int& next_box_id) {
    SegmentEdgeList transient_segment_edges;
    return connect_all(boxes, graph, transient_segment_edges, next_box_id);
}

IslandConnectorResult IslandConnector::connect_all(std::vector<BoxNode>& boxes,
                                                   AdjacencyGraph& graph,
                                                   SegmentEdgeList& segment_edges,
                                                   int& next_box_id) {
    RuntimeConfig runtime;
    runtime.mode = config_.n_threads > 1 ? ExecutionMode::Parallel : ExecutionMode::Inline;
    runtime.n_threads = config_.n_threads;
    runtime.batch_size = config_.pair_batch_size;
    runtime.parallel_threshold = config_.parallel_threshold;
    runtime.deterministic_reduce = config_.deterministic_reduce;
    StageContext context(runtime);
    return connect_all(boxes, graph, segment_edges, next_box_id, context);
}

IslandConnectorResult IslandConnector::connect_all(std::vector<BoxNode>& boxes,
                                                   AdjacencyGraph& graph,
                                                   int& next_box_id,
                                                   StageContext& context) {
    SegmentEdgeList transient_segment_edges;
    return connect_all(boxes, graph, transient_segment_edges, next_box_id, context);
}

IslandConnectorResult IslandConnector::connect_all(std::vector<BoxNode>& boxes,
                                                   AdjacencyGraph& graph,
                                                   SegmentEdgeList& segment_edges,
                                                   int& next_box_id,
                                                   StageContext& context) {
    IslandConnectorResult result;
    auto islands = find_islands(graph);
    context.diagnostics().add_counter("connector.islands_initial", static_cast<double>(islands.size()));
    if (islands.size() <= 1) {
        result.connected = true;
        return result;
    }

    std::unordered_map<int, double> frontier_best_gap_by_root;
    std::unordered_map<int, int> frontier_stale_by_root;
    while (config_.frontier_bridge && islands.size() > 1 &&
           result.bridge_boxes_added < config_.max_total_bridge_boxes) {
        if (context.should_stop()) {
            break;
        }
        FrontierBridgeCandidate candidate;
        if (!select_frontier_bridge_candidate(boxes,
                                              graph,
                                              oracle_,
                                              config_,
                                              frontier_best_gap_by_root,
                                              frontier_stale_by_root,
                                              context,
                                              candidate)) {
            break;
        }
        context.diagnostics().add_counter("connector.frontier_bridge_attempts");
        if (!add_frontier_bridge_box(candidate,
                                     boxes,
                                     oracle_,
                                     graph,
                                     next_box_id,
                                     config_,
                                     context)) {
            break;
        }
        result.bridge_boxes_added += 1;
        islands = find_islands(graph);
    }

    while (islands.size() > 1 &&
           (result.bridge_boxes_added < config_.max_total_bridge_boxes ||
            (config_.segment_edges_enabled && config_.rrt_segment_edges && !config_.segment_edges_fallback_only))) {
        if (context.should_stop()) {
            break;
        }
        auto map = make_box_map(boxes);
        std::sort(islands.begin(), islands.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.size() > rhs.size();
        });
        // E5: gather candidates between every island pair in a single round so
        // the parallel_for fills all worker threads (the gaps are independent).
        // This is intentionally not largest-island-only: in shelf-like scenes,
        // two small query-anchor islands can be much closer to each other than
        // either is to the largest component, and connecting them first gives the
        // box connector a shorter, easier target.
        // box_id -> island index, used at commit time to merge distinct components.
        std::unordered_map<int, int> island_of;
        for (std::size_t isl = 0; isl < islands.size(); ++isl) {
            for (int box_id : islands[isl]) {
                island_of[box_id] = static_cast<int>(isl);
            }
        }
        std::vector<BridgePairTask> candidates;
        const int per_gap_limit = std::max(1, config_.max_pairs_per_gap);
        for (std::size_t lhs_isl = 0; lhs_isl < islands.size(); ++lhs_isl) {
            for (std::size_t rhs_isl = lhs_isl + 1; rhs_isl < islands.size(); ++rhs_isl) {
                std::vector<BridgePairTask> gap_candidates = broadphase_bridge_pairs(map,
                                                                                     islands[lhs_isl],
                                                                                     islands[rhs_isl],
                                                                                     per_gap_limit,
                                                                                     std::max(4, config_.max_pairs_per_gap));
                for (auto& task : gap_candidates) {
                    task.task_id = static_cast<int>(candidates.size());
                    candidates.push_back(std::move(task));
                }
            }
        }
        const int broadphase_pairs_before_prune = static_cast<int>(candidates.size());
        const int global_candidate_limit =
            std::min(per_gap_limit,
                     std::max(4, 4 * std::max(1, static_cast<int>(islands.size()) - 1)));
        if (static_cast<int>(candidates.size()) > global_candidate_limit) {
            std::nth_element(candidates.begin(),
                             candidates.begin() + global_candidate_limit,
                             candidates.end(),
                             [](const BridgePairTask& lhs, const BridgePairTask& rhs) {
                                 return lhs.score < rhs.score;
                             });
            candidates.resize(static_cast<std::size_t>(global_candidate_limit));
        }
        std::sort(candidates.begin(), candidates.end(), [](const BridgePairTask& lhs, const BridgePairTask& rhs) {
            if (lhs.score == rhs.score) {
                if (lhs.source_box_id == rhs.source_box_id) {
                    return lhs.target_box_id < rhs.target_box_id;
                }
                return lhs.source_box_id < rhs.source_box_id;
            }
            return lhs.score < rhs.score;
        });
        for (int index = 0; index < static_cast<int>(candidates.size()); ++index) {
            candidates[static_cast<std::size_t>(index)].task_id = index;
        }
        context.diagnostics().add_counter("connector.bridge_broadphase_pairs_raw",
                                          static_cast<double>(broadphase_pairs_before_prune));
        context.diagnostics().add_counter("connector.bridge_broadphase_pairs", static_cast<double>(candidates.size()));
        context.diagnostics().add_counter("connector.bridge_broadphase_pairs_pruned",
                                          static_cast<double>(std::max(0, broadphase_pairs_before_prune -
                                                                          static_cast<int>(candidates.size()))));
        bool progressed = false;
        std::vector<BridgePairResult> successful_pairs;
        RRTConnectConfig pair_rrt = config_.rrt;
        if (config_.per_pair_timeout_ms > 0.0 &&
            (pair_rrt.timeout_ms <= 0.0 || config_.per_pair_timeout_ms < pair_rrt.timeout_ms)) {
            pair_rrt.timeout_ms = config_.per_pair_timeout_ms;
        }

        const bool run_parallel = context.executor().n_threads() > 1 &&
            static_cast<int>(candidates.size()) >= config_.parallel_threshold;
        if (run_parallel) {
            std::vector<BridgePairResult> pair_results(candidates.size());
            auto local_cancel = std::make_shared<std::atomic<bool>>(false);
            context.executor().parallel_for(0, static_cast<int>(candidates.size()), [&](int idx) {
                if (context.should_stop() || local_cancel->load(std::memory_order_relaxed)) {
                    return;
                }
                const auto& candidate = candidates[static_cast<std::size_t>(idx)];
                const BoxNode& source_box = *map.at(candidate.source_box_id);
                const BoxNode& target_box = *map.at(candidate.target_box_id);
                const auto& source_center = source_box.center();
                const auto& target_center = target_box.center();
                const RRTConnectConfig domain_rrt =
                    with_query_root_hull_domain(pair_rrt, oracle_, source_center, target_center);
                RRTConnectConfig box_rrt = domain_rrt;
                if (config_.pave.require_connected_chain) {
                    box_rrt.shortcut_path = true;
                }
                auto path = closest_box_point_segment(source_box, target_box, checker_, box_rrt.segment_resolution, box_rrt.segment_step);
                if (!path.empty()) {
                    context.diagnostics().add_counter("connector.direct_box_segment_successes");
                    context.diagnostics().add_counter("connector.rrt_successes");
                    pair_results[static_cast<std::size_t>(idx)] = {
                        candidate.task_id,
                        candidate.source_box_id,
                        candidate.target_box_id,
                        std::move(path),
                        true};
                    if (!config_.deterministic_reduce) {
                        local_cancel->store(true, std::memory_order_relaxed);
                    }
                    return;
                }
                const bool source_colliding = checker_.check_config(source_center);
                const bool target_colliding = checker_.check_config(target_center);
                if (source_colliding || target_colliding) {
                    context.diagnostics().add_counter("connector.center_collision_candidates");
                    if (source_colliding) {
                        context.diagnostics().add_counter("connector.source_center_collisions");
                    }
                    if (target_colliding) {
                        context.diagnostics().add_counter("connector.target_center_collisions");
                    }
                    return;
                }
                if (!config_.enable_birrt) {
                    context.diagnostics().add_counter("connector.birrt_disabled_skips");
                    return;
                }
                context.diagnostics().add_counter("connector.birrt_invocations");
                auto outcome = birrt_connect_impl(
                    source_center,
                    target_center,
                    checker_,
                    robot_,
                    box_rrt,
                    candidate.source_box_id + candidate.target_box_id + candidate.task_id,
                    local_cancel);
                record_birrt_stats(context.diagnostics(), outcome.stats);
                path = std::move(outcome.path);
                if (!path.empty()) {
                    context.diagnostics().add_counter("connector.rrt_successes");
                    pair_results[static_cast<std::size_t>(idx)] = {
                        candidate.task_id,
                        candidate.source_box_id,
                        candidate.target_box_id,
                        std::move(path),
                        true};
                    if (!config_.deterministic_reduce) {
                        local_cancel->store(true, std::memory_order_relaxed);
                    }
                } else {
                    context.diagnostics().add_counter("connector.rrt_failures");
                }
            });
            result.attempted_pairs += static_cast<int>(candidates.size());
            for (auto& item : pair_results) {
                if (item.success) {
                    successful_pairs.push_back(std::move(item));
                }
            }
        } else {
            for (const auto& candidate : candidates) {
                if (context.should_stop()) {
                    break;
                }
                result.attempted_pairs += 1;
                const BoxNode& source_box = *map.at(candidate.source_box_id);
                const BoxNode& target_box = *map.at(candidate.target_box_id);
                const auto& source_center = source_box.center();
                const auto& target_center = target_box.center();
                const RRTConnectConfig domain_rrt =
                    with_query_root_hull_domain(pair_rrt, oracle_, source_center, target_center);
                RRTConnectConfig box_rrt = domain_rrt;
                if (config_.pave.require_connected_chain) {
                    box_rrt.shortcut_path = true;
                }
                auto path = closest_box_point_segment(source_box, target_box, checker_, box_rrt.segment_resolution, box_rrt.segment_step);
                if (!path.empty()) {
                    context.diagnostics().add_counter("connector.direct_box_segment_successes");
                    context.diagnostics().add_counter("connector.rrt_successes");
                    successful_pairs.push_back({
                        candidate.task_id,
                        candidate.source_box_id,
                        candidate.target_box_id,
                        std::move(path),
                        true});
                    continue;
                }
                const bool source_colliding = checker_.check_config(source_center);
                const bool target_colliding = checker_.check_config(target_center);
                if (source_colliding || target_colliding) {
                    context.diagnostics().add_counter("connector.center_collision_candidates");
                    if (source_colliding) {
                        context.diagnostics().add_counter("connector.source_center_collisions");
                    }
                    if (target_colliding) {
                        context.diagnostics().add_counter("connector.target_center_collisions");
                    }
                    continue;
                }
                if (!config_.enable_birrt) {
                    context.diagnostics().add_counter("connector.birrt_disabled_skips");
                    continue;
                }
                context.diagnostics().add_counter("connector.birrt_invocations");
                path = rrt_connect(
                    source_center,
                    target_center,
                    checker_,
                    robot_,
                    context,
                    box_rrt,
                    candidate.source_box_id + candidate.target_box_id + candidate.task_id);
                if (path.empty()) {
                    context.diagnostics().add_counter("connector.rrt_failures");
                    continue;
                }
                context.diagnostics().add_counter("connector.rrt_successes");
                successful_pairs.push_back({
                    candidate.task_id,
                    candidate.source_box_id,
                    candidate.target_box_id,
                    std::move(path),
                    true});
            }
        }

        // E5 deterministic commit: process successful pairs in a stable order (by
        // task_id) and commit every bridge whose two islands are still in distinct
        // components, tracked by a union-find over island indices. This merges
        // multiple independent gaps per round while keeping commit fully serial and
        // order-independent of thread completion. Adjacency is recomputed once after
        // all commits.
        std::sort(successful_pairs.begin(), successful_pairs.end(),
                  [](const BridgePairResult& lhs, const BridgePairResult& rhs) {
                      return lhs.task_id < rhs.task_id;
                  });
        std::vector<int> uf(islands.size());
        for (std::size_t i = 0; i < uf.size(); ++i) {
            uf[i] = static_cast<int>(i);
        }
        auto uf_find = [&](int x) {
            while (uf[x] != x) {
                uf[x] = uf[uf[x]];
                x = uf[x];
            }
            return x;
        };
        bool boxes_added_this_round = false;
        for (const auto& chosen : successful_pairs) {
            const auto src_isl_it = island_of.find(chosen.source_box_id);
            const auto tgt_isl_it = island_of.find(chosen.target_box_id);
            if (src_isl_it == island_of.end() || tgt_isl_it == island_of.end()) {
                continue;
            }
            const int src_root = uf_find(src_isl_it->second);
            const int tgt_root = uf_find(tgt_isl_it->second);
            if (src_root == tgt_root) {
                // These two islands were already bridged earlier this round.
                continue;
            }
            std::vector<Eigen::VectorXd> bridge_path = chosen.waypoint_path;
            if (config_.pave.require_connected_chain && bridge_path.size() > 2) {
                const double pave_step =
                    config_.pave.gap_fill_sample_step > 0.0
                        ? std::max(0.05, config_.pave.gap_fill_sample_step * 2.0)
                        : std::max(0.05, config_.rrt.step_size * 0.5);
                bridge_path = densify_path_by_step(bridge_path, pave_step);
                context.diagnostics().set_value("connector.box_shortcut_densified_last_waypoints",
                                                static_cast<double>(bridge_path.size()));
            }
            int added = 0;
            bool box_connected = false;
            double pair_depth_failures_before = boundary_max_depth_failure_count(context);
            if (result.bridge_boxes_added < config_.max_total_bridge_boxes) {
                context.diagnostics().add_counter("connector.chain_pave_attempts");
                const int first_new_box_id = next_box_id;
                added = chain_pave_along_path(
                    bridge_path,
                    chosen.source_box_id,
                    boxes,
                    oracle_,
                    graph,
                    next_box_id,
                    context,
                    config_.pave);
                if (added > 0) {
                    const int local_edges = connect_new_boxes_to_island(
                        boxes,
                        graph,
                        first_new_box_id,
                        next_box_id,
                        islands[static_cast<std::size_t>(tgt_isl_it->second)],
                        config_.pave.adjacency_tolerance);
                    if (local_edges > 0) {
                        context.diagnostics().add_counter("connector.chain_pave_local_target_edges",
                                                          static_cast<double>(local_edges));
                    }
                    box_connected = graph_has_path(graph, chosen.source_box_id, chosen.target_box_id);
                    if (!box_connected) {
                        context.diagnostics().add_counter("connector.chain_pave_full_adjacency_rebuilds_avoided");
                        context.diagnostics().add_counter("connector.chain_pave_incremental_path_misses");
                        box_connected = graph_has_path(graph, chosen.source_box_id, chosen.target_box_id);
                    }
                    if (box_connected) {
                        context.diagnostics().add_counter("connector.chain_pave_box_connected");
                    } else {
                        context.diagnostics().add_counter("connector.chain_pave_partial_added");
                    }
                }
            }
            const bool pair_had_max_depth_ffb_failure =
                boundary_max_depth_failure_count(context) > pair_depth_failures_before + 0.5;
            bool added_segment_edge = false;
            if (!box_connected &&
                config_.segment_edges_enabled && config_.rrt_segment_edges &&
                !config_.segment_edges_fallback_only &&
                pair_had_max_depth_ffb_failure) {
                const int edge_id = add_segment_edge(segment_edges,
                                                     graph,
                                                     chosen.source_box_id,
                                                     chosen.target_box_id,
                                                     bridge_path,
                                                     SegmentEdgeType::RRTConnector,
                                                     config_.rrt.segment_resolution,
                                                     SegmentEdgeValidation::CollisionChecked,
                                                     false);
                if (edge_id >= 0) {
                    added_segment_edge = true;
                    result.segment_edges_added += 1;
                    result.rrt_segment_edges_added += 1;
                    context.diagnostics().add_counter("connector.segment_edges_added");
                    context.diagnostics().add_counter("connector.rrt_segment_edges_added");
                }
            } else if (!box_connected &&
                       config_.segment_edges_enabled && config_.rrt_segment_edges &&
                       !config_.segment_edges_fallback_only &&
                       !pair_had_max_depth_ffb_failure) {
                context.diagnostics().add_counter("connector.segment_edge_blocked_no_max_depth_ffb_failure");
            }
            if (box_connected) {
                context.diagnostics().add_counter("connector.chain_pave_successes");
                result.bridge_boxes_added += added;
                boxes_added_this_round = true;
                uf[src_root] = tgt_root;
                progressed = true;
            } else if (added > 0) {
                result.bridge_boxes_added += added;
                boxes_added_this_round = true;
                if (added_segment_edge) {
                    uf[src_root] = tgt_root;
                    progressed = true;
                }
            } else if (added_segment_edge) {
                uf[src_root] = tgt_root;
                progressed = true;
            } else {
                context.diagnostics().add_counter("connector.chain_pave_zero_added");
            }
        }
        if (boxes_added_this_round) {
            context.diagnostics().add_counter("connector.round_full_adjacency_rebuilds_avoided");
        }
        apply_segment_edges_to_adjacency(segment_edges, graph);
        if (!progressed) {
            break;
        }
        islands = find_islands(graph);
    }
    while (find_islands(graph).size() > 1) {
        if (boundary_max_depth_failure_count(context) <= 0.5) {
            context.diagnostics().add_counter("connector.point_gap_segment_blocked_no_max_depth_ffb_failure");
            break;
        }
        const std::size_t edge_count_before = segment_edges.size();
        if (!try_point_validated_gap_edge(boxes, graph, &segment_edges, checker_, config_, context)) {
            break;
        }
        if (segment_edges.size() > edge_count_before) {
            const int added_edges = static_cast<int>(segment_edges.size() - edge_count_before);
            result.segment_edges_added += added_edges;
            result.point_gap_segment_edges_added += added_edges;
        }
    }
    result.connected = find_islands(graph).size() <= 1;
    return result;
}

}  // namespace rbf
