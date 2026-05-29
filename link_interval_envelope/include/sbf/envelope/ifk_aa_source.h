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
    AaFkPrefixState& state);

EndpointIAABBResult compute_endpoint_iaabb_hifk_aa(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    const EndpointSourceConfig& config);

EndpointIAABBResult compute_endpoint_iaabb_hifk_aa(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int max_depth = 7,
    int n_threads = 1,
    double vol_ratio_thresh = 0.0);

std::vector<int> aafk_volume_min_depth_schedule(
    const Robot& robot,
    const std::vector<Interval>& root_intervals,
    int max_depth);
// Standalone HIFK defaults to round-robin splitting. Callers that already own
// a split policy can inject a depth-aligned schedule through EndpointSourceConfig.

}  // namespace rbf