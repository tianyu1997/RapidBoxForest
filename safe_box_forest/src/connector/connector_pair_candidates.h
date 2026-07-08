#pragma once

#include <SBF/connector_types.h>
#include <SBF/runtime_fwd.h>
#include <SBF/scene_types.h>

#include <unordered_map>
#include <vector>

namespace rbf {

struct BridgePairCandidateRoundPlan {
    std::vector<BridgePairTask> candidates;
    std::unordered_map<int, int> island_of;
};

BridgePairCandidateRoundPlan plan_bridge_pair_candidates(
    std::vector<std::vector<int>>& islands,
    const std::unordered_map<int, const BoxNode*>& map,
    const IslandConnectorConfig& config,
    StageContext& context);

}  // namespace rbf
