#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>

#include "planning_forest_query_bridge_corridor_graph.h"
#include "planning_forest_diagnostics.h"
#include "planning_forest_query_bridge_path_utils.h"
#include "planning_forest_query_utils.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace rbf {

int RBFPlanningForest::try_promote_query_bridge_direct_transition(
    int source_box_id,
    int target_box_id,
    const std::vector<std::vector<int>>& sample_layers,
    std::size_t boxes_before_direct_corridor,
    StageContext& context,
    int query_index,
    int bridge_edge_query_index,
    const char* reason,
    bool& attempted) {
    using Clock = std::chrono::steady_clock;
    const auto adopt_t0 = Clock::now();
    auto& diagnostics = context.diagnostics();
    auto finish_adopt = [&](int value) {
        const double elapsed =
            std::chrono::duration<double, std::milli>(Clock::now() - adopt_t0).count();
        diagnostics.record_timing("query_bridge.hipac_promote_transition.ms_total",
                                  elapsed);
        diagnostics.add_counter("query_bridge.hipac_promote_transition.ms_total",
                                elapsed);
        query_bridge_set_task_value(context,
                                    query_index,
                                    "hipac_promote_transition_ms",
                                    elapsed);
        if (value > 0) {
            query_bridge_set_task_value(context,
                                        query_index,
                                        "hipac_promote_transition_added",
                                        static_cast<double>(value));
        }
        if (reason != nullptr) {
            diagnostics.set_value(std::string("query_bridge.hipac_promote_transition.reason.") +
                                      reason,
                                  1.0);
        }
        return value;
    };
    if (attempted) {
        diagnostics.add_counter("query_bridge.hipac_promote_transition.skipped_repeat");
        return finish_adopt(0);
    }
    attempted = true;
    const QueryBridgeHipacPromotionGate promotion_gate =
        query_bridge_hipac_promotion_gate(last_adaptive_partition_config_,
                                          partition_native_mode(),
                                          source_box_id,
                                          target_box_id,
                                          query_index);
    if (promotion_gate.disabled) {
        diagnostics.add_counter("query_bridge.hipac_promote_transition.disabled");
        return finish_adopt(0);
    }
    if (promotion_gate.target_rejected) {
        diagnostics.add_counter("query_bridge.hipac_promote_transition.target_rejects");
        return finish_adopt(0);
    }
    diagnostics.add_counter("query_bridge.hipac_promote_transition.attempts");

    const int min_boxes = promotion_gate.min_boxes;
    const int max_boxes = promotion_gate.max_boxes;
    const BoxNode* source_box_ptr = find_box_by_id(boxes_, source_box_id);
    const BoxNode* target_box_ptr = find_box_by_id(boxes_, target_box_id);
    if (source_box_ptr == nullptr || target_box_ptr == nullptr) {
        diagnostics.add_counter("query_bridge.hipac_promote_transition.missing_endpoint_box");
        return finish_adopt(0);
    }
    const auto source_it =
        std::find_if(boxes_.begin(), boxes_.end(), [&](const BoxNode& box) {
            return box.id == source_box_id;
        });
    const auto target_it =
        std::find_if(boxes_.begin(), boxes_.end(), [&](const BoxNode& box) {
            return box.id == target_box_id;
        });
    if (source_it == boxes_.end() || target_it == boxes_.end()) {
        diagnostics.add_counter("query_bridge.hipac_promote_transition.missing_endpoint_index");
        return finish_adopt(0);
    }
    const int source_index = static_cast<int>(std::distance(boxes_.begin(), source_it));
    const int target_index_box = static_cast<int>(std::distance(boxes_.begin(), target_it));

    std::unordered_map<int, int> first_sample_by_box;
    first_sample_by_box.reserve(boxes_.size() - boxes_before_direct_corridor + 8);
    for (std::size_t sample_index = 0; sample_index < sample_layers.size(); ++sample_index) {
        for (int box_index : sample_layers[sample_index]) {
            if (box_index < static_cast<int>(boxes_before_direct_corridor) ||
                box_index < 0 ||
                box_index >= static_cast<int>(boxes_.size())) {
                continue;
            }
            first_sample_by_box.emplace(box_index, static_cast<int>(sample_index));
        }
    }
    std::vector<int> candidate_indices;
    candidate_indices.reserve(first_sample_by_box.size());
    for (const auto& [box_index, sample_index] : first_sample_by_box) {
        (void)sample_index;
        if (box_index == source_index || box_index == target_index_box) {
            continue;
        }
        const auto& box = boxes_[static_cast<std::size_t>(box_index)];
        if (box.safety_status != BoxSafetyStatus::CertifiedFree ||
            box.strict_audit_required) {
            diagnostics.add_counter(
                "query_bridge.hipac_promote_transition.reject_non_certified");
            continue;
        }
        candidate_indices.push_back(box_index);
    }
    std::sort(candidate_indices.begin(),
              candidate_indices.end(),
              [&](int lhs, int rhs) {
        const int lhs_sample = first_sample_by_box.count(lhs) > 0
            ? first_sample_by_box[lhs]
            : std::numeric_limits<int>::max();
        const int rhs_sample = first_sample_by_box.count(rhs) > 0
            ? first_sample_by_box[rhs]
            : std::numeric_limits<int>::max();
        if (lhs_sample != rhs_sample) {
            return lhs_sample < rhs_sample;
        }
        return lhs < rhs;
    });
    if (static_cast<int>(candidate_indices.size()) < min_boxes) {
        diagnostics.add_counter("query_bridge.hipac_promote_transition.too_few_boxes");
        diagnostics.add_counter("query_bridge.hipac_promote_transition.candidate_boxes",
                                static_cast<double>(candidate_indices.size()));
        return finish_adopt(0);
    }
    if (static_cast<int>(candidate_indices.size()) > max_boxes) {
        candidate_indices.resize(static_cast<std::size_t>(max_boxes));
    }
    diagnostics.add_counter("query_bridge.hipac_promote_transition.candidate_boxes",
                            static_cast<double>(candidate_indices.size()));

    std::vector<int> local_indices;
    local_indices.reserve(candidate_indices.size() + 2);
    local_indices.push_back(source_index);
    local_indices.insert(local_indices.end(),
                         candidate_indices.begin(),
                         candidate_indices.end());
    local_indices.push_back(target_index_box);
    const int local_source = 0;
    const int local_target = static_cast<int>(local_indices.size()) - 1;
    std::vector<std::vector<int>> local_adj(local_indices.size());
    int exact_tests = 0;
    int exact_edges = 0;
    for (std::size_t lhs = 0; lhs < local_indices.size(); ++lhs) {
        for (std::size_t rhs = lhs + 1; rhs < local_indices.size(); ++rhs) {
            ++exact_tests;
            if (boxes_connected(boxes_[static_cast<std::size_t>(local_indices[lhs])],
                                boxes_[static_cast<std::size_t>(local_indices[rhs])],
                                config_.query.adjacency_tolerance)) {
                local_adj[lhs].push_back(static_cast<int>(rhs));
                local_adj[rhs].push_back(static_cast<int>(lhs));
                ++exact_edges;
            }
        }
    }
    diagnostics.add_counter("query_bridge.hipac_promote_transition.local_adj_tests",
                            static_cast<double>(exact_tests));
    diagnostics.add_counter("query_bridge.hipac_promote_transition.local_adj_edges",
                            static_cast<double>(exact_edges));
    auto promote_local_path = [&](const std::vector<int>& local_path,
                                  const char* mode) {
        if (local_path.size() < 3) {
            diagnostics.add_counter("query_bridge.hipac_promote_transition.short_chain");
            return 0;
        }
        const int source_local = local_path.front();
        const int target_local = local_path.back();
        const BoxNode& portal_source =
            boxes_[static_cast<std::size_t>(local_indices[static_cast<std::size_t>(source_local)])];
        const BoxNode& portal_target =
            boxes_[static_cast<std::size_t>(local_indices[static_cast<std::size_t>(target_local)])];
        std::vector<BoxNode> internal_boxes;
        internal_boxes.reserve(local_path.size());
        for (int local_node : local_path) {
            if (local_node == source_local || local_node == target_local) {
                continue;
            }
            internal_boxes.push_back(
                boxes_[static_cast<std::size_t>(local_indices[static_cast<std::size_t>(local_node)])]);
        }
        if (static_cast<int>(internal_boxes.size()) < min_boxes) {
            diagnostics.add_counter("query_bridge.hipac_promote_transition.short_chain");
            return 0;
        }
        const int edge_id = append_portal_corridor_edge(segment_edges_,
                                                        portal_source,
                                                        portal_target,
                                                        std::move(internal_boxes),
                                                        -1,
                                                        config_.query.adjacency_tolerance,
                                                        bridge_edge_query_index);
        if (edge_id < 0) {
            diagnostics.add_counter("query_bridge.hipac_promote_transition.edge_fail");
            return 0;
        }
        sync_adaptive_partition_segment_edges(&last_build_,
                                              "query_bridge.hipac_promote_transition");
        diagnostics.add_counter("query_bridge.hipac_promote_transition.added");
        diagnostics.add_counter(std::string("query_bridge.hipac_promote_transition.added_") +
                                    mode);
        diagnostics.add_counter("query_bridge.hipac_promote_transition.internal_boxes",
                                static_cast<double>(local_path.size() - 2));
        query_bridge_set_task_value(context,
                                    query_index,
                                    "hipac_promote_transition_internal_boxes",
                                    static_cast<double>(local_path.size() - 2));
        invalidate_query_cache();
        return 1;
    };
    std::vector<int> full_local_path =
        query_bridge_shortest_local_path(local_adj, local_source, local_target);
    if (!full_local_path.empty()) {
        const int promoted = promote_local_path(full_local_path, "full");
        if (promoted > 0) {
            return finish_adopt(promoted);
        }
    } else {
        diagnostics.add_counter("query_bridge.hipac_promote_transition.chain_fail");
    }

    const auto [component_id, component_count] =
        query_bridge_internal_local_components(local_adj,
                                               local_source,
                                               local_target);
    diagnostics.add_counter("query_bridge.hipac_promote_transition.slice_components",
                            static_cast<double>(component_count));
    const std::vector<QueryBridgeLocalSliceCandidate> slices =
        query_bridge_component_slice_candidates(component_id,
                                                component_count,
                                                local_indices,
                                                first_sample_by_box,
                                                min_boxes);
    for (const auto& slice : slices) {
        std::vector<int> slice_path =
            query_bridge_shortest_local_path(local_adj, slice.first, slice.last);
        if (slice_path.empty()) {
            diagnostics.add_counter("query_bridge.hipac_promote_transition.slice_chain_fail");
            continue;
        }
        const int promoted = promote_local_path(slice_path, "slice");
        if (promoted > 0) {
            return finish_adopt(promoted);
        }
    }
    diagnostics.add_counter("query_bridge.hipac_promote_transition.failures");
    return finish_adopt(0);
}

}  // namespace rbf
