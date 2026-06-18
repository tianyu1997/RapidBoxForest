#pragma once

#include "env_config.h"

#include <algorithm>
#include <vector>

namespace rbf {

inline std::vector<int> query_endpoint_anchor_ffb_depths_from_env() {
    return detail::env_int_list_or_empty("RBF_QUERY_ENDPOINT_ANCHOR_FFB_DEPTHS");
}

inline bool query_endpoint_point_anchor_enabled_from_env() {
    return detail::env_int_or_default("RBF_QUERY_ENDPOINT_POINT_ANCHOR", 0) != 0;
}

inline double endpoint_shortlink_max_length_from_env() {
    return std::max(0.0,
                    detail::env_double_or_default("RBF_ENDPOINT_SHORTLINK_MAX_LENGTH", 0.25));
}

}  // namespace rbf
