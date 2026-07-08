#include "connector_pair_candidates.h"

#include "connector_internal.h"

#include <SBF/runtime.h>

#include <algorithm>
#include <utility>

namespace rbf {

BridgePairCandidateRoundPlan plan_bridge_pair_candidates(
    std::vector<std::vector<int>>& islands,
    const std::unordered_map<int, const BoxNode*>& map,
    const IslandConnectorConfig& config,
    StageContext& context) {
    BridgePairCandidateRoundPlan plan;
    std::sort(islands.begin(), islands.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.size() > rhs.size();
    });
    for (std::size_t isl = 0; isl < islands.size(); ++isl) {
        for (int box_id : islands[isl]) {
            plan.island_of[box_id] = static_cast<int>(isl);
        }
    }

    const int per_gap_limit = std::max(1, config.max_pairs_per_gap);
    for (std::size_t lhs_isl = 0; lhs_isl < islands.size(); ++lhs_isl) {
        for (std::size_t rhs_isl = lhs_isl + 1; rhs_isl < islands.size(); ++rhs_isl) {
            std::vector<BridgePairTask> gap_candidates =
                broadphase_bridge_pairs(map,
                                        islands[lhs_isl],
                                        islands[rhs_isl],
                                        per_gap_limit,
                                        std::max(4, config.max_pairs_per_gap));
            for (auto& task : gap_candidates) {
                task.task_id = static_cast<int>(plan.candidates.size());
                plan.candidates.push_back(std::move(task));
            }
        }
    }

    const int broadphase_pairs_before_prune = static_cast<int>(plan.candidates.size());
    const int global_candidate_limit =
        std::min(per_gap_limit,
                 std::max(4, 4 * std::max(1, static_cast<int>(islands.size()) - 1)));
    if (static_cast<int>(plan.candidates.size()) > global_candidate_limit) {
        std::nth_element(plan.candidates.begin(),
                         plan.candidates.begin() + global_candidate_limit,
                         plan.candidates.end(),
                         [](const BridgePairTask& lhs, const BridgePairTask& rhs) {
                             return lhs.score < rhs.score;
                         });
        plan.candidates.resize(static_cast<std::size_t>(global_candidate_limit));
    }
    std::sort(plan.candidates.begin(), plan.candidates.end(), [](const BridgePairTask& lhs,
                                                                 const BridgePairTask& rhs) {
        if (lhs.score == rhs.score) {
            if (lhs.source_box_id == rhs.source_box_id) {
                return lhs.target_box_id < rhs.target_box_id;
            }
            return lhs.source_box_id < rhs.source_box_id;
        }
        return lhs.score < rhs.score;
    });
    for (int index = 0; index < static_cast<int>(plan.candidates.size()); ++index) {
        plan.candidates[static_cast<std::size_t>(index)].task_id = index;
    }
    context.diagnostics().add_counter("connector.bridge_broadphase_pairs_raw",
                                      static_cast<double>(broadphase_pairs_before_prune));
    context.diagnostics().add_counter("connector.bridge_broadphase_pairs",
                                      static_cast<double>(plan.candidates.size()));
    context.diagnostics().add_counter("connector.bridge_broadphase_pairs_pruned",
                                      static_cast<double>(std::max(
                                          0,
                                          broadphase_pairs_before_prune -
                                              static_cast<int>(plan.candidates.size()))));
    return plan;
}

}  // namespace rbf
