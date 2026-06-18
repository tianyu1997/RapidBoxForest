#pragma once

#include "env_config.h"

#include <algorithm>
#include <vector>

namespace rbf {

inline bool obb_sampled_support_enabled_from_env() {
    return detail::env_flag_or_default("RBF_OBB_SAMPLED_SUPPORT", false);
}

inline bool obb_clearance_sampled_enabled_from_env() {
    return detail::env_flag_or_default("RBF_OBB_CLEARANCE_SAMPLED_SUPPORT", true);
}

inline double obb_clearance_lateral_l1_max_from_env() {
    return detail::env_double_or_default("RBF_OBB_CLEARANCE_LATERAL_L1_MAX", 5e-3);
}

inline int obb_clearance_samples_from_env() {
    return detail::env_int_or_default("RBF_OBB_CLEARANCE_SAMPLES", 17);
}

inline double obb_clearance_dense_line_l1_threshold_from_env() {
    return detail::env_double_or_default("RBF_OBB_CLEARANCE_DENSE_LINE_L1_THRESHOLD", 0.03);
}

inline int obb_clearance_dense_samples_from_env() {
    return detail::env_int_or_default("RBF_OBB_CLEARANCE_DENSE_SAMPLES", 17);
}

inline int obb_clearance_fast_samples_from_env() {
    return detail::env_int_or_default("RBF_OBB_CLEARANCE_FAST_SAMPLES", 0);
}

inline bool obb_clearance_first_from_env() {
    return detail::env_int_or_default("RBF_OBB_CLEARANCE_FIRST", 0) != 0;
}

inline bool obb_fast_primary_orientation_from_env() {
    return detail::env_flag_or_default("RBF_OBB_FAST_PRIMARY_ORIENTATION", true);
}

inline bool obb_fallback_orientations_on_primary_fail_from_env() {
    return detail::env_flag_or_default("RBF_OBB_FALLBACK_ORIENTATIONS_ON_PRIMARY_FAIL", false);
}

inline bool obb_metadata_only_from_env() {
    return detail::env_int_or_default("RBF_OBB_METADATA_ONLY", 0) != 0;
}

inline bool obb_metadata_only_require_cover_from_env() {
    return detail::env_int_or_default("RBF_OBB_METADATA_ONLY_REQUIRE_COVER", 0) != 0;
}

inline int obb_clearance_retry_attempts_from_env() {
    return std::max(0, detail::env_int_or_default("RBF_OBB_CLEARANCE_RETRY_ATTEMPTS", 0));
}

inline std::vector<double> obb_clearance_retry_values_from_env() {
    return detail::env_double_list_or_empty("RBF_OBB_CLEARANCE_RETRY_VALUES");
}

inline int obb_clearance_retry_iters_from_env(int fallback) {
    return detail::env_int_or_default("RBF_OBB_CLEARANCE_RETRY_ITERS", fallback);
}

inline double obb_clearance_retry_timeout_ms_from_env(double fallback) {
    return detail::env_double_or_default("RBF_OBB_CLEARANCE_RETRY_TIMEOUT_MS", fallback);
}

}  // namespace rbf
