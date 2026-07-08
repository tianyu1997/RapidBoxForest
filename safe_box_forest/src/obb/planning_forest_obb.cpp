#include "planning_forest_obb.h"

namespace rbf {

ObbValidationOptions obb_validation_options_from_config(const AdaptiveLeafSweepConfig& config) {
    ObbValidationOptions options;
    options.fast_primary_orientation = config.obb_fast_primary_orientation;
    options.fallback_orientations_on_primary_fail =
        config.obb_fallback_orientations_on_primary_fail;
    options.sampled_support_enabled = config.obb_sampled_support_enabled;
    options.clearance_sampled_support_enabled =
        config.obb_clearance_sampled_support_enabled;
    options.clearance_lateral_l1_max = config.obb_clearance_lateral_l1_max;
    options.clearance_samples = config.obb_clearance_samples;
    options.clearance_dense_line_l1_threshold =
        config.obb_clearance_dense_line_l1_threshold;
    options.clearance_dense_samples = config.obb_clearance_dense_samples;
    options.clearance_fast_samples = config.obb_clearance_fast_samples;
    options.clearance_first = config.obb_clearance_first;
    options.clearance_retry_attempts = config.obb_clearance_retry_attempts;
    options.clearance_retry_values = config.obb_clearance_retry_values;
    options.clearance_retry_iters = config.obb_clearance_retry_iters;
    options.clearance_retry_timeout_ms = config.obb_clearance_retry_timeout_ms;
    return options;
}

}  // namespace rbf
