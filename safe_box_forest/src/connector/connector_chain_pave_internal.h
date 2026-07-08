#pragma once

#include "connector_internal.h"

#include <cstdint>
#include <vector>

namespace rbf {

void record_chain_pave_boundary_ffb_failure(const FindFreeBoxResult& result,
                                            const Eigen::VectorXd& seed,
                                            BoxOracle& oracle,
                                            const ChainPaveConfig& config,
                                            StageContext& context);

std::vector<Eigen::VectorXd> chain_pave_boundary_seed_candidates(const BoxNode& box,
                                                                 const Eigen::VectorXd& from,
                                                                 const Eigen::VectorXd& target,
                                                                 double requested_step,
                                                                 double adjacency_tolerance,
                                                                 double gap_fill_min_step);

Eigen::VectorXd chain_pave_closest_point_in_box(const BoxNode& box,
                                                const Eigen::VectorXd& point);

std::uint64_t chain_pave_boundary_seed_key(int parent_id,
                                           std::size_t segment_index,
                                           const BoxNode& parent_box,
                                           const Eigen::VectorXd& cursor,
                                           const Eigen::VectorXd& seed,
                                           double adjacency_tolerance,
                                           double gap_fill_min_step);

struct ChainPaveConnectedStats {
    int segments = 0;
    int steps = 0;
    int reach_failures = 0;
    int target_hits = 0;
};

void record_chain_pave_connected_stats(StageContext& context,
                                       int added,
                                       int max_chain,
                                       const ChainPaveConnectedStats& stats);

}  // namespace rbf
