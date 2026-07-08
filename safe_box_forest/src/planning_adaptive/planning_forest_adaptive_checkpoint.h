#pragma once

#include "planning_forest_adaptive_cover_utils.h"

namespace rbf {

struct AdaptiveDepthCheckpointDecision {
    bool stop = false;
    int next_checkpoint_depth = 0;
};

int adaptive_next_depth_checkpoint(int depth, int target_leaf_depth);

AdaptiveDepthCheckpointDecision advance_adaptive_depth_checkpoint(
    AdaptiveDepthSnapshot& snapshot,
    int target_leaf_depth);

}  // namespace rbf
