#pragma once

#include "planning_forest_adaptive_merge.h"

#include <SBF/box_graph.h>
#include <SBF/build_config.h>
#include <SBF/planning_result.h>

#include <string>
#include <unordered_map>

namespace rbf {

void record_adaptive_merge_diagnostics(std::unordered_map<std::string, double>& diagnostics,
                                       const BudgetedMergeStats& stats);

void record_adaptive_partition_merge_diagnostics(
    std::unordered_map<std::string, double>& diagnostics,
    const std::unordered_map<std::string, double>& source);

void record_adaptive_adjacency_diagnostics(std::unordered_map<std::string, double>& diagnostics,
                                           const std::string& prefix,
                                           const AdjacencyBuildStats& stats);

void record_adaptive_depth_gate_diagnostics(
    std::unordered_map<std::string, double>& diagnostics,
    const AdaptiveLeafSweepConfig& config,
    bool adaptive_depth_enabled,
    int adaptive_depth_min,
    int target_leaf_depth);

void record_adaptive_probe_diagnostics(std::unordered_map<std::string, double>& diagnostics,
                                       const AdaptiveLeafSweepResult& result,
                                       int anchor_cap,
                                       int anchor_attempts);

}  // namespace rbf
