#include "connector_chain_pave_internal.h"

#include <SBF/box_graph.h>
#include <SBF/connector.h>
#include <SBF/find_free_box.h>
#include <SBF/oracle.h>
#include <SBF/runtime.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <unordered_set>

namespace rbf {

int chain_pave_along_path(const std::vector<Eigen::VectorXd>& waypoint_path,
                          int anchor_box_id,
                          std::vector<BoxNode>& boxes,
                          BoxOracle& oracle,
                          AdjacencyGraph& graph,
                          int& next_box_id,
                          const ChainPaveConfig& config) {
    StageContext context = StageContext::serial();
    return chain_pave_along_path(waypoint_path, anchor_box_id, boxes, oracle, graph, next_box_id, context, config);
}

int chain_pave_along_path(const std::vector<Eigen::VectorXd>& waypoint_path,
                          int anchor_box_id,
                          std::vector<BoxNode>& boxes,
                          BoxOracle& oracle,
                          AdjacencyGraph& graph,
                          int& next_box_id,
                          StageContext& context,
                          const ChainPaveConfig& config) {
    if (waypoint_path.empty()) {
        return 0;
    }
    FindFreeBoxService ffb(oracle);
    auto find_at_depth = [&](const Eigen::VectorXd& seed,
                             StageContext& stage_context,
                             int depth) {
        FindFreeBoxOptions options = config.find_free_box;
        options.max_depth = std::max(1, depth);
        return ffb.find(seed, stage_context, options);
    };
    int current_box_id = anchor_box_id;
    int added = 0;

    ChainPaveCommitContext commit_context(boxes,
                                          oracle,
                                          graph,
                                          next_box_id,
                                          config,
                                          context,
                                          added);

    // Cover the C-space segment [from_pt -> to_pt] with connected boxes, extending
    // the chain from box `from_id`. First try to commit a box certified at to_pt
    // directly; if that box exists but is not adjacent to the current chain box (a
    // residual gap), bisect the segment and recurse. Crucially the segment lies on
    // the connector's collision-free bridge polyline, so every midpoint is itself
    // collision-free and certifiable -- the recursion fills the gap with real
    // boxes instead of cutting a corner through a C-space obstacle. Returns the id
    // of the furthest box reached (== from_id when no progress was made). With
    // budget == 0 this reduces to a single direct-commit attempt at to_pt.
    std::function<int(int, const Eigen::VectorXd&, const Eigen::VectorXd&, int)> cover =
        [&](int from_id,
            const Eigen::VectorXd& from_pt,
            const Eigen::VectorXd& to_pt,
            int budget) -> int {
        if (added >= config.max_chain || context.should_stop()) {
            return from_id;
        }
        // Obtain a box covering to_pt: commit a fresh certified box, or reuse an
        // existing box that already owns that canonical cell (so the chain can
        // follow the bridge through previously paved regions).
        int to_id = from_id;
        bool saw_certifiable = false;
        bool rejection_counted = false;
        bool bridge_to_existing_cover = false;
        const int added_before = added;
        const int existing_cover = commit_context.find_existing_cover(to_pt, from_id);
        if (existing_cover >= 0) {
            if (existing_cover == from_id) {
                context.diagnostics().add_counter("connector.chain_pave_existing_cover_current");
                to_id = from_id;
            } else {
                BoxNode* from_box = commit_context.box_by_id(from_id);
                BoxNode* cover_box = commit_context.box_by_id(existing_cover);
                if (from_box != nullptr && cover_box != nullptr &&
                    boxes_connected(*from_box, *cover_box, config.adjacency_tolerance)) {
                    commit_context.append_graph_edge(from_id, existing_cover);
                    context.diagnostics().add_counter("connector.chain_pave_existing_cover_adjacent");
                    to_id = existing_cover;
                } else {
                    context.diagnostics().add_counter("connector.chain_pave_existing_cover_non_adjacent");
                    bridge_to_existing_cover = true;
                }
            }
        }
        auto consume_result = [&](FindFreeBoxResult& result) {
            saw_certifiable = true;
            const int committed = commit_context.commit_box(result, to_pt, from_id);
            if (committed >= 0) {
                to_id = committed;
                if (added > added_before) {
                    context.diagnostics().add_counter("connector.chain_pave_boundary_commits");
                }
                return true;
            } else {
                const int owner = commit_context.find_box_owning_node_covering(result.node, to_pt);
                if (owner >= 0) {
                    BoxNode* owner_box = commit_context.box_by_id(owner);
                    BoxNode* from_box = commit_context.box_by_id(from_id);
                    if (owner_box != nullptr && from_box != nullptr &&
                        boxes_connected(*from_box, *owner_box, config.adjacency_tolerance)) {
                        to_id = owner;
                        return true;
                    }
                }
                if (!result.found &&
                    result.hit_reserved_depth_cap &&
                    intervals_contain_point(result.intervals,
                                            to_pt,
                                            config.adjacency_tolerance)) {
                    const int duplicate = commit_context.commit_reserved_cap_box(result,
                                                                                to_pt,
                                                                                from_id);
                    if (duplicate >= 0) {
                        to_id = duplicate;
                        if (added > added_before) {
                            context.diagnostics().add_counter("connector.chain_pave_boundary_commits");
                        }
                        return true;
                    }
                }
            }
            return false;
        };
        if (existing_cover < 0) {
            context.diagnostics().add_counter("connector.chain_pave_boundary_ffb_calls");
            auto result = find_at_depth(to_pt, context, config.find_free_box.max_depth);
            if (result.seed_collision) {
                record_chain_pave_boundary_ffb_failure(result, to_pt, oracle, config, context);
                context.diagnostics().add_counter("connector.chain_pave_boundary_reject_not_free");
                rejection_counted = true;
            } else if (result.found ||
                       (result.hit_reserved_depth_cap &&
                        intervals_contain_point(result.intervals,
                                                to_pt,
                                                config.adjacency_tolerance))) {
                consume_result(result);
            } else {
                record_chain_pave_boundary_ffb_failure(result, to_pt, oracle, config, context);
            }
        }
        if (bridge_to_existing_cover && budget <= 0 && config.fill_gaps) {
            budget = std::max(1, config.max_gap_fill_depth);
            context.diagnostics().add_counter("connector.chain_pave_existing_cover_bridge_attempts");
        }
        if (to_id == from_id && !rejection_counted && !bridge_to_existing_cover) {
            context.diagnostics().add_counter(
                saw_certifiable
                    ? "connector.chain_pave_boundary_reject_non_adjacent"
                    : "connector.chain_pave_boundary_reject_not_free");
        }
        if (budget <= 0) {
            return to_id;
        }
        // A committed box is a convex axis-aligned box: if a SINGLE box contains
        // BOTH endpoints, the whole segment between them is inside that box and is
        // therefore fully covered. This is the correct termination test -- checking
        // only the midpoint (as a weaker variant did) leaves the quarter/three-
        // quarter points of the segment uncovered, capping coverage well below
        // 100%. Recurse by bisection until each leaf sub-segment has both endpoints
        // inside one box (every midpoint lies on the collision-free bridge polyline
        // and is itself certifiable, so the recursion terminates with real boxes).
        {
            auto map = make_box_map(boxes);
            auto segment_in_one_box = [&](int id) {
                auto it = map.find(id);
                return it != map.end() && it->second->contains(from_pt) &&
                       it->second->contains(to_pt);
            };
            if (segment_in_one_box(from_id) || segment_in_one_box(to_id)) {
                return to_id;
            }
        }
        const Eigen::VectorXd mid = 0.5 * (from_pt + to_pt);
        if ((mid - to_pt).norm() < config.gap_fill_min_step ||
            (mid - from_pt).norm() < config.gap_fill_min_step) {
            return to_id;
        }
        const int via = cover(from_id, from_pt, mid, budget - 1);
        if (added >= config.max_chain || context.should_stop()) {
            return via;
        }
        return cover(via, mid, to_pt, budget - 1);
    };

    std::unordered_set<std::uint64_t> failed_boundary_seed_keys;

    if (waypoint_path.size() >= 2) {
        ChainPaveConnectedStats connected_stats;
        for (std::size_t seg = 1;
             seg < waypoint_path.size() && added < config.max_chain &&
             !context.should_stop();
             ++seg) {
            const Eigen::VectorXd& a = waypoint_path[seg - 1];
            const Eigen::VectorXd& b = waypoint_path[seg];
            const double seg_len = (b - a).norm();
            if (seg_len < 1e-12) {
                continue;
            }
            connected_stats.segments += 1;
            BoxNode* current_box = commit_context.box_by_id(current_box_id);
            if (current_box == nullptr) {
                break;
            }
            Eigen::VectorXd cursor =
                current_box->contains(a) ? a : chain_pave_closest_point_in_box(*current_box, a);
            const double front_step = std::max(
                config.gap_fill_sample_step > 0.0 ? config.gap_fill_sample_step
                                                  : config.gap_fill_min_step,
                1e-6);
            int guard = 0;
            const int guard_max = std::max(
                1,
                static_cast<int>(std::ceil(seg_len / front_step)) + 2);
            while (added < config.max_chain && !context.should_stop() &&
                   guard++ < guard_max) {
                current_box = commit_context.box_by_id(current_box_id);
                if (current_box == nullptr) {
                    break;
                }
                if (current_box->contains(b)) {
                    connected_stats.target_hits += 1;
                    break;
                }
                connected_stats.steps += 1;
                if (!current_box->contains(cursor)) {
                    cursor = chain_pave_closest_point_in_box(*current_box, cursor);
                }
                int reached = current_box_id;
                double attempt_step = front_step;
                for (int attempt = 0; attempt < 8 && reached == current_box_id;
                     ++attempt) {
                    const auto seeds =
                        chain_pave_boundary_seed_candidates(*current_box,
                                                            cursor,
                                                            b,
                                                            attempt_step,
                                                            config.adjacency_tolerance,
                                                            config.gap_fill_min_step);
                    const double min_seed_motion =
                        std::max(4.0 * std::max(0.0, config.adjacency_tolerance), 1e-12);
                    if (seeds.empty() ||
                        (seeds.front() - cursor).norm() < min_seed_motion) {
                        break;
                    }
                    for (std::size_t seed_rank = 0; seed_rank < seeds.size(); ++seed_rank) {
                        const auto& seed = seeds[seed_rank];
                        const auto key = chain_pave_boundary_seed_key(current_box_id,
                                                                      seg,
                                                                      *current_box,
                                                                      cursor,
                                                                      seed,
                                                                      config.adjacency_tolerance,
                                                                      config.gap_fill_min_step);
                        if (failed_boundary_seed_keys.find(key) != failed_boundary_seed_keys.end()) {
                            context.diagnostics().add_counter("connector.chain_pave_boundary_skip_failed_seed");
                            continue;
                        }
                        reached = cover(current_box_id, cursor, seed, 0);
                        if (reached == current_box_id) {
                            failed_boundary_seed_keys.insert(key);
                            context.diagnostics().add_counter("connector.chain_pave_boundary_failed_seed_memoized");
                        }
                        if (reached != current_box_id ||
                            added >= config.max_chain || context.should_stop()) {
                            break;
                        }
                    }
                    attempt_step *= 0.5;
                }
                if (reached == current_box_id) {
                    connected_stats.reach_failures += 1;
                    context.diagnostics().add_counter("connector.chain_pave_boundary_stall");
                    break;
                }
                current_box_id = reached;
                if (BoxNode* reached_box = commit_context.box_by_id(current_box_id)) {
                    cursor = chain_pave_closest_point_in_box(*reached_box, b);
                }
            }
        }
        record_chain_pave_connected_stats(context,
                                          added,
                                          config.max_chain,
                                          connected_stats);
        return added;
    }

    return added;
}

}  // namespace rbf
