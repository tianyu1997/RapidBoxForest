#pragma once
/// @file ifk_aa_source.h
/// @brief AA-backed IFK and HIFK endpoint iAABB sources.

#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/envelope/endpoint_source.h>

#include <vector>

namespace rbf {

EndpointIAABBResult compute_endpoint_iaabb_ifk_aa(
    const Robot& robot,
    const std::vector<Interval>& intervals);

EndpointIAABBResult compute_endpoint_iaabb_hifk_aa(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int max_depth = 7,
    int n_threads = 1,
    double vol_ratio_thresh = 0.0);
// HIFK uses a shared depth->dim schedule internally: each depth commits one
// split dimension on first visit and reuses it for all later nodes at that
// depth in the same evaluation.

}  // namespace rbf