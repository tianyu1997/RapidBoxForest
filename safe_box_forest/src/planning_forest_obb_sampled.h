#pragma once

#include "planning_forest_obb.h"
#include "planning_forest_obb_candidate.h"

namespace rbf {

bool validate_obb_sampled_support_candidate(const Robot& robot,
                                            const Scene& scene,
                                            const ObbPortalCandidate& candidate,
                                            double safety_epsilon,
                                            ObbPortalValidationStats& stats);

bool validate_obb_clearance_sampled_candidate(const Robot& robot,
                                              const Scene& scene,
                                              const ObbPortalCandidate& candidate,
                                              double safety_epsilon,
                                              ObbPortalValidationStats& stats,
                                              const ObbValidationOptions& options);

}  // namespace rbf
