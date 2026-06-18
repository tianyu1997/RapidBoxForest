#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>
#include <SBF/connector.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_set>
#include <vector>

#include "planning_forest_audit.h"

namespace rbf {

namespace {

const BoxNode* find_box_by_id_local(const std::vector<BoxNode>& boxes, int box_id) {
    for (const auto& box : boxes) {
        if (box.id == box_id) {
            return &box;
        }
    }
    return nullptr;
}

std::uint64_t partition_segment_pair_key_local(int lhs, int rhs) {
    if (lhs > rhs) {
        std::swap(lhs, rhs);
    }
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(lhs)) << 32) |
           static_cast<std::uint32_t>(rhs);
}

std::unordered_set<std::uint64_t> partition_segment_pair_set_local(const SegmentEdgeList& edges) {
    std::unordered_set<std::uint64_t> pairs;
    pairs.reserve(edges.size() * 2);
    for (const auto& edge : edges) {
        pairs.insert(partition_segment_pair_key_local(edge.source_box_id, edge.target_box_id));
    }
    return pairs;
}

} // namespace

int RBFPlanningForest::add_offline_shortcut_edges(int max_edges,
                                                  int candidate_limit,
                                                  double min_gain_ratio,
                                                  double max_segment_length,
                                                  bool allow_segment_fallback) {
    if (max_edges <= 0 || candidate_limit < 2 || boxes_.size() < 2) {
        return 0;
    }
    if (partition_native_mode()) {
        if (!adaptive_partition_query_enabled_ || !adaptive_partition_ || adaptive_partition_->empty()) {
            last_build_.diagnostics["offline_shortcut.partition_native_missing_partition"] += 1.0;
            return 0;
        }
        const int limit = std::min<int>(candidate_limit, static_cast<int>(boxes_.size()));
        auto landmarks = adaptive_partition_->landmarks(true, limit);
        if (static_cast<int>(landmarks.size()) < 2) {
            landmarks = adaptive_partition_->landmarks(false, limit);
        }
        if (static_cast<int>(landmarks.size()) < 2) {
            last_build_.diagnostics["offline_shortcut.partition_native_insufficient_landmarks"] += 1.0;
            return 0;
        }
        auto existing_segment_pairs = partition_segment_pair_set_local(segment_edges_);

        struct ShortcutCandidate {
            int source = -1;
            int target = -1;
            Eigen::VectorXd source_center;
            Eigen::VectorXd target_center;
            double direct = 0.0;
            double partition_cost = 0.0;
            double score = 0.0;
        };
        std::vector<ShortcutCandidate> candidates;
        const double safe_min_gain = std::max(1.0, min_gain_ratio);
        const double safe_max_segment_length = std::max(0.0, max_segment_length);
        AdaptiveGridPartitionQueryOptions query_options;
        query_options.nearest_if_outside = false;
        query_options.shortcut_boxes = false;
        query_options.max_expansions = std::max(0, last_adaptive_partition_config_.grid_planning_max_expansions);
        query_options.adjacency_tolerance = config_.query.adjacency_tolerance;
        int tested_pairs = 0;
        for (std::size_t outer = 0; outer < landmarks.size(); ++outer) {
            const auto& lhs = landmarks[outer];
            for (std::size_t inner = outer + 1; inner < landmarks.size(); ++inner) {
                const auto& rhs = landmarks[inner];
                if (lhs.center.size() == 0 || lhs.center.size() != rhs.center.size()) {
                    continue;
                }
                ++tested_pairs;
                const double direct = (lhs.center - rhs.center).norm();
                if (direct <= 1e-9 ||
                    (safe_max_segment_length > 0.0 && direct > safe_max_segment_length) ||
                    existing_segment_pairs.count(
                        partition_segment_pair_key_local(lhs.box_id, rhs.box_id)) > 0) {
                    continue;
                }
                const auto route = adaptive_partition_->query(lhs.center, rhs.center, query_options);
                if (!route.found ||
                    !std::isfinite(route.total_cost) ||
                    route.total_cost <= direct * safe_min_gain) {
                    continue;
                }
                candidates.push_back({lhs.box_id,
                                      rhs.box_id,
                                      lhs.center,
                                      rhs.center,
                                      direct,
                                      route.total_cost,
                                      route.total_cost - direct});
            }
        }
        std::sort(candidates.begin(), candidates.end(), [](const ShortcutCandidate& lhs,
                                                           const ShortcutCandidate& rhs) {
            return lhs.score > rhs.score;
        });

        int added = 0;
        int portal_corridor_edges = 0;
        int portal_corridor_fail = 0;
        int box_corridor_edges = 0;
        int segment_edges = 0;
        int pave_added_total = 0;
        int pave_fail = 0;
        int audit_fail = 0;
        CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
        for (const auto& candidate : candidates) {
            if (added >= max_edges) {
                break;
            }
            if (candidate.source_center.size() == 0 ||
                candidate.source_center.size() != candidate.target_center.size()) {
                continue;
            }
            std::vector<Eigen::VectorXd> waypoints{candidate.source_center,
                                                   candidate.target_center};
            if (last_adaptive_partition_config_.hipac_portal_connectivity) {
                const int portal_added = add_partition_portal_corridor_overlay(candidate.source_center,
                                                                               candidate.target_center,
                                                                               waypoints,
                                                                               "offline_shortcut",
                                                                               false,
                                                                               false,
                                                                               -1,
                                                                               &last_build_);
                if (portal_added > 0) {
                    existing_segment_pairs.insert(
                        partition_segment_pair_key_local(candidate.source, candidate.target));
                    ++portal_corridor_edges;
                    ++added;
                    continue;
                }
                ++portal_corridor_fail;
            }
            const double audit_fail_before =
                last_build_.diagnostics["offline_shortcut.partition_box_corridor_overlay_audit_fail"];
            const int overlay_added = add_partition_box_corridor_overlay(candidate.source_center,
                                                                         candidate.target_center,
                                                                         waypoints,
                                                                         "offline_shortcut",
                                                                         false,
                                                                         false,
                                                                         -1,
                                                                         &last_build_);
            const double audit_fail_after =
                last_build_.diagnostics["offline_shortcut.partition_box_corridor_overlay_audit_fail"];
            if (audit_fail_after > audit_fail_before) {
                ++audit_fail;
            }
            if (overlay_added > 0) {
                existing_segment_pairs.insert(
                    partition_segment_pair_key_local(candidate.source, candidate.target));
                ++box_corridor_edges;
                ++added;
                continue;
            }
            ++pave_fail;
            if (allow_segment_fallback) {
                const auto audit = audit_waypoint_path(waypoints,
                                                       checker,
                                                       config_.query.audit_resolution,
                                                       config_.query.audit_segment_step);
                if (!audit.passed) {
                    ++audit_fail;
                    continue;
                }
                const int edge_id = add_segment_edge_partition_first(candidate.source,
                                                                     candidate.target,
                                                                     std::move(waypoints),
                                                                     SegmentEdgeType::QueryBridge,
                                                                     config_.query.audit_resolution,
                                                                     SegmentEdgeValidation::CollisionChecked,
                                                                     true,
                                                                     -1,
                                                                     nullptr,
                                                                     "offline_shortcut");
                if (edge_id >= 0) {
                    existing_segment_pairs.insert(
                        partition_segment_pair_key_local(candidate.source, candidate.target));
                    ++segment_edges;
                    ++added;
                }
            }
        }
        last_build_.diagnostics["offline_shortcut.partition_native"] += 1.0;
        last_build_.diagnostics["offline_shortcut.partition_native_direct_overlay"] += 1.0;
        last_build_.diagnostics["offline_shortcut.partition_native_query_bridge_skipped"] += 1.0;
        last_build_.diagnostics["offline_shortcut.tested_pairs"] += static_cast<double>(tested_pairs);
        last_build_.diagnostics["offline_shortcut.candidates"] += static_cast<double>(candidates.size());
        last_build_.diagnostics["offline_shortcut.audit_fail"] += static_cast<double>(audit_fail);
        last_build_.diagnostics["offline_shortcut.edges_added"] += static_cast<double>(added);
        last_build_.diagnostics["offline_shortcut.segment_edges_added"] += static_cast<double>(segment_edges);
        last_build_.diagnostics["offline_shortcut.portal_corridor_edges_added"] +=
            static_cast<double>(portal_corridor_edges);
        last_build_.diagnostics["offline_shortcut.portal_corridor_fail"] +=
            static_cast<double>(portal_corridor_fail);
        last_build_.diagnostics["offline_shortcut.box_corridor_edges_added"] += static_cast<double>(box_corridor_edges);
        last_build_.diagnostics["offline_shortcut.pave_boxes_added"] += static_cast<double>(pave_added_total);
        last_build_.diagnostics["offline_shortcut.pave_fail"] += static_cast<double>(pave_fail);
        sync_adaptive_partition_segment_edges(&last_build_, "offline_shortcut.partition_native");
        return added;
    }

    const int limit = std::min<int>(candidate_limit, static_cast<int>(boxes_.size()));
    std::vector<int> landmarks;
    landmarks.reserve(boxes_.size());
    for (int index = 0; index < static_cast<int>(boxes_.size()); ++index) {
        const auto graph_it = adjacency_.find(boxes_[static_cast<std::size_t>(index)].id);
        if (graph_it != adjacency_.end() && !graph_it->second.empty()) {
            landmarks.push_back(index);
        }
    }
    if (static_cast<int>(landmarks.size()) < 2) {
        landmarks.clear();
        for (int index = 0; index < static_cast<int>(boxes_.size()); ++index) {
            landmarks.push_back(index);
        }
    }
    std::sort(landmarks.begin(), landmarks.end(), [&](int lhs, int rhs) {
        return boxes_[static_cast<std::size_t>(lhs)].volume >
               boxes_[static_cast<std::size_t>(rhs)].volume;
    });
    if (static_cast<int>(landmarks.size()) > limit) {
        landmarks.resize(static_cast<std::size_t>(limit));
    }

    struct ShortcutCandidate {
        int source = -1;
        int target = -1;
        double direct = 0.0;
        double graph_cost = 0.0;
        double score = 0.0;
    };
    std::vector<ShortcutCandidate> candidates;
    const QueryGraphCache cache = build_query_graph_cache(boxes_, adjacency_, segment_edges_);
    const double safe_min_gain = std::max(1.0, min_gain_ratio);
    const double safe_max_segment_length = std::max(0.0, max_segment_length);
    int tested_pairs = 0;
    for (std::size_t outer = 0; outer < landmarks.size(); ++outer) {
        const BoxNode& lhs = boxes_[static_cast<std::size_t>(landmarks[outer])];
        for (std::size_t inner = outer + 1; inner < landmarks.size(); ++inner) {
            const BoxNode& rhs = boxes_[static_cast<std::size_t>(landmarks[inner])];
            if (lhs.n_dims() != rhs.n_dims()) {
                continue;
            }
            ++tested_pairs;
            const double direct = (lhs.center() - rhs.center()).norm();
            if (direct <= 1e-9 ||
                (safe_max_segment_length > 0.0 && direct > safe_max_segment_length) ||
                find_segment_edge(segment_edges_, lhs.id, rhs.id) != nullptr) {
                continue;
            }
            const auto route = dijkstra_search(cache, lhs.id, rhs.id, rhs.center());
            if (!route.found || !std::isfinite(route.total_cost) ||
                route.total_cost <= direct * safe_min_gain) {
                continue;
            }
            candidates.push_back({lhs.id, rhs.id, direct, route.total_cost, route.total_cost - direct});
        }
    }
    std::sort(candidates.begin(), candidates.end(), [](const ShortcutCandidate& lhs,
                                                       const ShortcutCandidate& rhs) {
        return lhs.score > rhs.score;
    });

    CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
    int added = 0;
    int box_corridor_edges = 0;
    int segment_edges = 0;
    int pave_added_total = 0;
    int pave_fail = 0;
    int audit_fail = 0;
    int next_id = next_box_id();
    StageContext context = StageContext::from_runtime(config_.runtime);
    for (const auto& candidate : candidates) {
        if (added >= max_edges) {
            break;
        }
        if (find_segment_edge(segment_edges_, candidate.source, candidate.target) != nullptr) {
            continue;
        }
        const BoxNode* source = find_box_by_id_local(boxes_, candidate.source);
        const BoxNode* target = find_box_by_id_local(boxes_, candidate.target);
        if (source == nullptr || target == nullptr) {
            continue;
        }
        std::vector<Eigen::VectorXd> waypoints{source->center(), target->center()};
        const auto audit = audit_waypoint_path(waypoints,
                                               checker,
                                               config_.query.audit_resolution,
                                               config_.query.audit_segment_step);
        if (!audit.passed) {
            ++audit_fail;
            continue;
        }
        int pave_added = 0;
        if (!partition_native_mode()) {
            ChainPaveConfig pave_config = config_.connector.pave;
            pave_config.fill_gaps = true;
            pave_config.require_connected_chain = true;
            pave_config.max_chain = std::max(pave_config.max_chain, 64);
            pave_config.max_steps_per_waypoint = std::max(pave_config.max_steps_per_waypoint, 16);
            pave_config.find_free_box.max_depth = std::min(
                std::max(1, config_.database.max_tree_depth),
                std::max(1,
                         config_.query_bridge_pave_depth > 0
                             ? config_.query_bridge_pave_depth
                             : pave_config.find_free_box.max_depth));
	            pave_config.gap_fill_time_budget_ms = std::max(pave_config.gap_fill_time_budget_ms, 75.0);
            pave_config.gap_fill_max_ffb_calls = std::max(pave_config.gap_fill_max_ffb_calls, 192);
            const std::size_t boxes_before_pave = boxes_.size();
            pave_added = chain_pave_along_path(waypoints,
                                               candidate.source,
                                               boxes_,
                                               *oracle_,
                                               adjacency_,
                                               next_id,
                                               context,
                                               pave_config);
            if (pave_added > 0) {
                append_adaptive_partition_boxes(boxes_before_pave,
                                                &last_build_,
                                                "offline_shortcut.pave");
            }
        } else {
            last_build_.diagnostics["offline_shortcut.partition_legacy_chain_pave_skipped"] += 1.0;
        }
        pave_added_total += pave_added;
        int edge_id = -1;
        if (pave_added > 0 &&
            box_only_path_connected_partition_first(candidate.source, candidate.target)) {
            edge_id = add_segment_edge_partition_first(                                       candidate.source,
                                       candidate.target,
                                       waypoints,
                                       SegmentEdgeType::BoxCorridor,
                                       config_.query.audit_resolution,
                                       SegmentEdgeValidation::CollisionChecked,
                                       false,
                                       -1);
            if (edge_id >= 0) {
                ++box_corridor_edges;
            }
        } else {
            ++pave_fail;
            edge_id = add_segment_edge_partition_first(                                       candidate.source,
                                       candidate.target,
                                       std::move(waypoints),
                                       SegmentEdgeType::QueryBridge,
                                       config_.query.audit_resolution,
                                       SegmentEdgeValidation::CollisionChecked,
                                       true,
                                       -1);
            if (edge_id >= 0) {
                ++segment_edges;
            }
        }
        if (edge_id >= 0) {
            ++added;
        }
    }
    if (added > 0) {
        invalidate_query_cache();
    }
    last_build_.diagnostics["offline_shortcut.tested_pairs"] += static_cast<double>(tested_pairs);
    last_build_.diagnostics["offline_shortcut.candidates"] += static_cast<double>(candidates.size());
    last_build_.diagnostics["offline_shortcut.audit_fail"] += static_cast<double>(audit_fail);
    last_build_.diagnostics["offline_shortcut.edges_added"] += static_cast<double>(added);
    last_build_.diagnostics["offline_shortcut.box_corridor_edges_added"] += static_cast<double>(box_corridor_edges);
    last_build_.diagnostics["offline_shortcut.segment_edges_added"] += static_cast<double>(segment_edges);
    last_build_.diagnostics["offline_shortcut.pave_boxes_added"] += static_cast<double>(pave_added_total);
    last_build_.diagnostics["offline_shortcut.pave_fail"] += static_cast<double>(pave_fail);
    sync_adaptive_partition_segment_edges(&last_build_, "offline_shortcut");
    return added;
}

} // namespace rbf
