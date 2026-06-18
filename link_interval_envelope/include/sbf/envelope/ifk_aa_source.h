#pragma once
/// @file ifk_aa_source.h
/// @brief AA-backed IFK and HIFK endpoint iAABB sources.

#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/core/aa_fk.h>
#include <sbf/envelope/endpoint_source.h>

#include <vector>

namespace rbf {

EndpointIAABBResult compute_endpoint_iaabb_ifk_aa(
    const Robot& robot,
    const std::vector<Interval>& intervals);

/// IFK (single AA-FK pass) endpoint source with incremental reuse. The result
/// is bit-identical to compute_endpoint_iaabb_ifk_aa; @p state caches the
/// kinematic-chain prefix so a subsequent single-dimension change recomputes
/// only the affected suffix. Intended for sequential parent->child descents on
/// a single thread (one @p state per descending oracle).
EndpointIAABBResult compute_endpoint_iaabb_ifk_aa_stateful(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    AaFkPrefixState& state,
    bool* used_incremental = nullptr);

struct HifkAaState {
    std::vector<AaFkPrefixState> leaf_states;
    std::vector<int> depth_dimensions;
    std::vector<Interval> root_intervals;
    HifkSplitStrategy split_strategy = HifkSplitStrategy::RoundRobin;
    int effective_max_depth = 0;
    int depth_offset = 0;
    int n_joints = 0;
    bool valid = false;
};

EndpointIAABBResult compute_endpoint_iaabb_hifk_aa(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    const EndpointSourceConfig& config);

EndpointIAABBResult compute_endpoint_iaabb_hifk_aa_stateful(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    const EndpointSourceConfig& config,
    HifkAaState& state,
    bool* used_incremental = nullptr);

EndpointIAABBResult compute_endpoint_iaabb_hifk_aa(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int max_depth = 7,
    int n_threads = 1,
    double vol_ratio_thresh = 0.0);

std::vector<int> aafk_volume_min_depth_schedule(
    const Robot& robot,
    const std::vector<Interval>& root_intervals,
    int max_depth,
    int sample_nodes_per_depth = 8);

// Seed/scene-independent split schedule that minimises the SupportHull envelope
// volume swept by each link (the envelope actually used at certification time),
// rather than the looser sum of endpoint AABBs. Pure function of
// (robot, canonical root intervals); never depends on a query seed or scene.
std::vector<int> support_hull_volume_min_depth_schedule(
    const Robot& robot,
    const std::vector<Interval>& root_intervals,
    int max_depth,
    int sample_nodes_per_depth = 8);
// Standalone HIFK defaults to round-robin splitting. Callers that already own
// a split policy can inject a depth-aligned schedule through EndpointSourceConfig.

}  // namespace rbf