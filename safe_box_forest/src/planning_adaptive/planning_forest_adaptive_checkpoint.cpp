#include "planning_forest_adaptive_checkpoint.h"

#include <algorithm>

namespace rbf {

int adaptive_next_depth_checkpoint(int depth, int target_leaf_depth) {
    const int step = depth < 16 ? 1 : 2;
    return std::min(target_leaf_depth, depth + step);
}

AdaptiveDepthCheckpointDecision advance_adaptive_depth_checkpoint(
    AdaptiveDepthSnapshot& snapshot,
    int target_leaf_depth) {
    AdaptiveDepthCheckpointDecision decision;
    decision.next_checkpoint_depth = snapshot.depth;
    if (snapshot.readiness_met) {
        snapshot.stop_reason = "coverage_ready";
        decision.stop = true;
    } else if (snapshot.depth >= target_leaf_depth) {
        snapshot.stop_reason = "max_depth";
        decision.stop = true;
    } else {
        snapshot.stop_reason = "checkpoint";
        decision.next_checkpoint_depth =
            adaptive_next_depth_checkpoint(snapshot.depth, target_leaf_depth);
    }
    return decision;
}

}  // namespace rbf
