#pragma once

#include <SBF/connector_types.h>
#include <SBF/runtime_fwd.h>
#include <SBF/scene_types.h>

#include <unordered_map>
#include <vector>

namespace rbf {

class BoxOracle;
class CollisionChecker;

struct BridgePairExecutionResult {
    std::vector<BridgePairResult> successful_pairs;
    int attempted_pairs = 0;
};

BridgePairExecutionResult run_bridge_pair_tasks(
    const std::vector<BridgePairTask>& candidates,
    const std::unordered_map<int, const BoxNode*>& map,
    BoxOracle& oracle,
    const Robot& robot,
    const CollisionChecker& checker,
    const IslandConnectorConfig& config,
    StageContext& context);

}  // namespace rbf
